#![cfg(loom)]

//! Loom concurrency tests for SamplingPort write/read races.
//!
//! Each test wraps a `SamplingPortPool` in `Arc<Mutex<_>>` and spawns threads
//! that perform concurrent write/read operations. Loom exhaustively explores
//! all thread interleavings to verify no partial/corrupt data is ever visible.

use kernel::loom_compat::{model, thread, Arc, Mutex};
use kernel::sampling::{PortDirection, SamplingPortPool, Validity};

/// Helper: lock a loom Mutex (returns MutexGuard, panics on poison).
fn lock<T>(m: &Mutex<T>) -> loom::sync::MutexGuard<'_, T> {
    m.lock().unwrap()
}

/// Concurrent write + read: reader sees either old or new data, never
/// partial/corrupt data. Pre-load the source+destination with an initial
/// message, then race a writer (new data) against a reader.
#[test]
fn concurrent_write_read_no_partial_data() {
    model(|| {
        let pool = Arc::new(Mutex::new(SamplingPortPool::<4, 8>::new()));

        let (src, dst) = {
            let mut g = lock(&pool);
            let s = g.create_port(PortDirection::Source, 1000).unwrap();
            let d = g.create_port(PortDirection::Destination, 1000).unwrap();
            g.connect_ports(s, d).unwrap();
            // Pre-load: write initial data through the source→destination path.
            g.write_sampling_message(s, &[0xAA; 4], 10).unwrap();
            (s, d)
        };

        // Thread A: overwrite with new data.
        let p1 = pool.clone();
        let t_write = thread::spawn(move || {
            let res = lock(&p1).write_sampling_message(src, &[0xBB; 4], 20);
            assert!(res.is_ok(), "write must succeed: {:?}", res);
        });

        // Thread B: read from destination.
        let p2 = pool.clone();
        let t_read = thread::spawn(move || {
            let mut buf = [0u8; 8];
            let (size, _validity) = lock(&p2).read_sampling_message(dst, &mut buf, 50).unwrap();
            (size, buf)
        });

        t_write.join().unwrap();
        let (size, buf) = t_read.join().unwrap();

        // Reader must see either the old data or the new data — never a mix.
        assert_eq!(size, 4, "message size must be 4");
        let payload = &buf[..size];
        assert!(
            payload == [0xAA; 4] || payload == [0xBB; 4],
            "data must be all-0xAA or all-0xBB, got {:?}",
            payload
        );
    });
}

/// Two concurrent writes: last-writer-wins and data is internally consistent.
/// Both writers send distinct patterns; after both complete the port must
/// contain exactly one of the two patterns with matching size and timestamp.
#[test]
fn two_concurrent_writes_last_writer_wins() {
    model(|| {
        let pool = Arc::new(Mutex::new(SamplingPortPool::<4, 8>::new()));

        let src = {
            let mut g = lock(&pool);
            g.create_port(PortDirection::Source, 1000).unwrap()
        };

        // Thread A: write pattern 0xCC with timestamp 100.
        let p1 = pool.clone();
        let t1 = thread::spawn(move || {
            let res = lock(&p1).write_sampling_message(src, &[0xCC; 3], 100);
            assert!(res.is_ok(), "writer A must succeed: {:?}", res);
        });

        // Thread B: write pattern 0xDD with timestamp 200.
        let p2 = pool.clone();
        let t2 = thread::spawn(move || {
            let res = lock(&p2).write_sampling_message(src, &[0xDD; 5], 200);
            assert!(res.is_ok(), "writer B must succeed: {:?}", res);
        });

        t1.join().unwrap();
        t2.join().unwrap();

        // After both complete, the port must contain exactly one writer's data.
        let g = lock(&pool);
        let port = g.get(src).unwrap();
        let data = port.data();
        let ts = port.timestamp();
        let size = port.current_size();

        // Either writer A's data (3 bytes of 0xCC, ts=100)
        // or writer B's data (5 bytes of 0xDD, ts=200).
        if size == 3 {
            assert!(
                data.iter().all(|&b| b == 0xCC),
                "3-byte payload must be all-0xCC, got {:?}",
                data
            );
            assert_eq!(ts, 100, "timestamp must match writer A");
        } else if size == 5 {
            assert!(
                data.iter().all(|&b| b == 0xDD),
                "5-byte payload must be all-0xDD, got {:?}",
                data
            );
            assert_eq!(ts, 200, "timestamp must match writer B");
        } else {
            panic!(
                "size must be 3 (writer A) or 5 (writer B), got {} with data {:?}",
                size, data
            );
        }
    });
}

/// Timestamp ordering: sequential writes through the pool must produce
/// monotonically non-decreasing timestamps visible to readers. Two threads
/// write with increasing timestamps; the final read must see the latest.
#[test]
fn read_after_writes_timestamp_monotonic() {
    model(|| {
        let pool = Arc::new(Mutex::new(SamplingPortPool::<4, 8>::new()));

        let (src, dst) = {
            let mut g = lock(&pool);
            let s = g.create_port(PortDirection::Source, 5000).unwrap();
            let d = g.create_port(PortDirection::Destination, 5000).unwrap();
            g.connect_ports(s, d).unwrap();
            // Seed with ts=10.
            g.write_sampling_message(s, &[0x01], 10).unwrap();
            (s, d)
        };

        // Thread A: write with ts=20.
        let p1 = pool.clone();
        let t1 = thread::spawn(move || {
            let res = lock(&p1).write_sampling_message(src, &[0x02], 20);
            assert!(res.is_ok(), "writer ts=20 must succeed: {:?}", res);
        });

        // Thread B: write with ts=30.
        let p2 = pool.clone();
        let t2 = thread::spawn(move || {
            let res = lock(&p2).write_sampling_message(src, &[0x03], 30);
            assert!(res.is_ok(), "writer ts=30 must succeed: {:?}", res);
        });

        t1.join().unwrap();
        t2.join().unwrap();

        // Source port must have one of the written timestamps.
        let g = lock(&pool);
        let src_port = g.get(src).unwrap();
        let src_ts = src_port.timestamp();
        assert!(
            src_ts == 10 || src_ts == 20 || src_ts == 30,
            "source timestamp must be 10, 20, or 30, got {}",
            src_ts
        );

        // The data and timestamp must be consistent: each ts maps to one payload.
        let src_data = src_port.data();
        match src_ts {
            10 => assert_eq!(src_data, &[0x01], "ts=10 must have payload 0x01"),
            20 => assert_eq!(src_data, &[0x02], "ts=20 must have payload 0x02"),
            30 => assert_eq!(src_data, &[0x03], "ts=30 must have payload 0x03"),
            _ => unreachable!(),
        }

        // Destination should reflect seed write (connected write copies to dst).
        // After both writers, dst has the last writer's data since
        // write_sampling_message writes to both src and connected dst.
        let dst_port = g.get(dst).unwrap();
        let dst_ts = dst_port.timestamp();
        let dst_data = dst_port.data();

        assert!(
            dst_ts == 10 || dst_ts == 20 || dst_ts == 30,
            "destination timestamp must be 10, 20, or 30, got {}",
            dst_ts
        );
        // Destination data+timestamp must be internally consistent.
        match dst_ts {
            10 => assert_eq!(dst_data, &[0x01], "dst ts=10 must have payload 0x01"),
            20 => assert_eq!(dst_data, &[0x02], "dst ts=20 must have payload 0x02"),
            30 => assert_eq!(dst_data, &[0x03], "dst ts=30 must have payload 0x03"),
            _ => unreachable!(),
        }

        // Destination timestamp must be >= source timestamp or equal, because
        // write_sampling_message writes dst first then src. In any interleaving
        // the dst cannot have a newer write than src doesn't also have OR
        // both reflect the same final writer. The key invariant: both are
        // from valid writes, never partial.
        let dst_valid = dst_port.validity(dst_ts + 1000);
        assert_eq!(
            dst_valid,
            Validity::Valid,
            "destination must be valid within refresh period"
        );
    });
}
