use std::{
    io::{BufRead, BufReader, Read, Write},
    net::TcpStream,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::mpsc,
    time::{Duration, Instant},
};

const PROBE_VID_PID: &str = "0483:374b";
const CHIP: &str = "STM32F429ZITx";

// ST-LINK VCP (USART3 via PD8/PD9, SB5/SB6 closed) — detected by VID:PID at runtime.
const UART_VID: u16 = 0x0483;
const UART_PID: u16 = 0x374b;
const UART_BAUD: u32 = 115_200;
const HOST_CYCLES: usize = 20;

/// Optional host-side function spawned in a thread alongside probe-rs.
/// Returns Ok(()) when all cycles completed successfully, Err(msg) on failure.
type HostFn = fn() -> Result<(), String>;

/// Read and discard all bytes currently buffered on `port`.
fn drain_serial(port: &mut dyn serialport::SerialPort) {
    port.clear(serialport::ClearBuffer::Input).ok();
    // Wait briefly for any bytes still in-transit, then clear again.
    std::thread::sleep(Duration::from_millis(100));
    port.clear(serialport::ClearBuffer::Input).ok();
}

/// Read from `port` byte-by-byte until '\n', returning the trimmed line.
/// Discards leading noise lines that are not in `accept` set.
fn read_line(port: &mut dyn serialport::SerialPort) -> Result<String, String> {
    let mut buf = Vec::new();
    let mut b = [0u8; 1];
    loop {
        port.read_exact(&mut b)
            .map_err(|e| format!("serial read error: {e}"))?;
        if b[0] == b'\n' {
            break;
        }
        if b[0] != b'\r' {
            buf.push(b[0]);
        }
        if buf.len() > 128 {
            buf.clear(); // discard runaway garbage
        }
    }
    Ok(String::from_utf8_lossy(&buf).trim().to_string())
}

/// Find the ST-LINK VCP by VID:PID and open it.
fn open_uart_port() -> Result<Box<dyn serialport::SerialPort>, String> {
    let port_name = find_port_by_vid_pid(UART_VID, UART_PID, 3000)
        .ok_or_else(|| format!("ST-LINK VCP ({UART_VID:04x}:{UART_PID:04x}) not found"))?;
    eprintln!("  [uart] detected ST-LINK VCP: {port_name}");
    serialport::new(&port_name, UART_BAUD)
        .timeout(Duration::from_secs(5))
        .open()
        .map_err(|e| format!("cannot open {port_name}: {e}"))
}

/// Host protocol for uart_dma_demo: send 32-byte pattern each cycle,
/// triggered by "READY\n" from MCU, expect "OK\n" back.
fn host_uart_dma_demo() -> Result<(), String> {
    let pattern: Vec<u8> = (0u8..32).collect();
    let mut port = open_uart_port()?;

    // Trace: log port settings
    let trace = std::env::var("HW_TEST_TRACE").is_ok();
    if trace {
        eprintln!("  [dma-trace] port opened, timeout={:?}", port.timeout());
        eprintln!("  [dma-trace] baud={} data_bits={:?} stop_bits={:?} parity={:?} flow={:?}",
            port.baud_rate().unwrap_or(0),
            port.data_bits().ok(),
            port.stop_bits().ok(),
            port.parity().ok(),
            port.flow_control().ok());
    }

    drain_serial(port.as_mut());
    if trace {
        eprintln!("  [dma-trace] drain complete, starting protocol");
    }

    let mut ok = 0usize;
    let mut err = 0usize;
    let mut noise_lines = 0usize;
    let t0 = Instant::now();

    while ok < HOST_CYCLES {
        // Wait for "READY" — MCU has armed DMA. Skip TIMEOUT and other lines.
        let line = read_line(port.as_mut())?;
        let elapsed_ms = t0.elapsed().as_millis();

        if line != "READY" {
            noise_lines += 1;
            if trace {
                eprintln!("  [dma-trace] [{elapsed_ms:6}ms] noise #{noise_lines}: {:?} (len={})",
                    line, line.len());
            }
            continue;
        }

        if trace {
            eprintln!("  [dma-trace] [{elapsed_ms:6}ms] READY #{} (noise_before={noise_lines})",
                ok + err + 1);
        }

        // Send the 32-byte pattern.
        port.write_all(&pattern)
            .map_err(|e| format!("write error: {e}"))?;
        port.flush().map_err(|e| format!("flush error: {e}"))?;

        if trace {
            eprintln!("  [dma-trace] [{:6}ms] sent 32B + flush", t0.elapsed().as_millis());
        }

        // Read MCU response ("OK" or "ERR").
        let resp = read_line(port.as_mut())?;
        if trace {
            eprintln!("  [dma-trace] [{:6}ms] resp: {:?}", t0.elapsed().as_millis(), resp);
        }

        if resp == "OK" {
            ok += 1;
        } else {
            err += 1;
            eprintln!("  [dma-host] cycle {} unexpected: {:?} (ok={ok} err={err})",
                ok + err, resp);
            // Dump any buffered bytes to see what's left
            if trace {
                let avail = port.bytes_to_read().unwrap_or(0);
                eprintln!("  [dma-trace] bytes_to_read after ERR: {avail}");
                if avail > 0 && avail < 256 {
                    let mut dump = vec![0u8; avail as usize];
                    if port.read(&mut dump).is_ok() {
                        eprintln!("  [dma-trace] buffered: {:02x?}", &dump[..dump.len().min(64)]);
                    }
                }
            }
        }
        noise_lines = 0;
    }

    println!(
        "  [dma-host] done: OK={ok} ERR={err} cycles={} {:.1}s",
        ok + err,
        t0.elapsed().as_secs_f64()
    );
    if err > 0 {
        eprintln!("  [dma-host] WARNING: {err} verify failures in {ok} successful cycles");
    }
    Ok(())
}

/// Host protocol for uart_dma_echo: triggered by "READY\n", send 32B pattern,
/// read back the echo (MCU DMA-receives then DMA-sends same buf).
fn host_uart_dma_echo() -> Result<(), String> {
    let pattern: Vec<u8> = (0u8..32).collect();
    let mut port = open_uart_port()?;
    drain_serial(port.as_mut());

    let mut ok = 0usize;
    let mut err = 0usize;
    let t0 = Instant::now();

    while ok + err < HOST_CYCLES {
        // Wait for "READY" from MCU. Skip TIMEOUT and other noise.
        let line = read_line(port.as_mut())?;
        if line != "READY" {
            continue;
        }

        // Send 32-byte pattern — MCU RX DMA is already armed.
        port.write_all(&pattern)
            .map_err(|e| format!("write error: {e}"))?;

        // Read 32-byte echo — MCU TX DMA sends the same buffer back.
        let mut echo = vec![0u8; 32];
        port.read_exact(&mut echo)
            .map_err(|e| format!("read error for echo: {e}"))?;

        if echo == pattern {
            ok += 1;
        } else {
            err += 1;
            eprintln!("  [echo-host] echo mismatch (ok={ok} err={err})");
        }

        // Drain the trailing "OK\n" the MCU sends after the echo.
        read_line(port.as_mut()).ok();
    }

    println!(
        "  [echo-host] done: OK={ok} ERR={err} cycles={HOST_CYCLES} {:.1}s",
        t0.elapsed().as_secs_f64()
    );
    if err == 0 {
        Ok(())
    } else {
        Err(format!("echo host: OK={ok} ERR={err}"))
    }
}

/// Host protocol for uart_usb_pipeline: send 32B on UART, read back 32B on USB CDC.
/// Opens both ports, waits for READY from MCU, runs protocol.
fn host_uart_usb_pipeline() -> Result<(), String> {
    let pattern: Vec<u8> = (0u8..32).collect();

    // Wait for probe-rs to start and complete flash before opening VCP.
    // probe-rs resets the ST-LINK which momentarily drops the VCP.
    std::thread::sleep(Duration::from_secs(8));
    let mut uart = open_uart_port()?;
    uart.write_data_terminal_ready(true).ok();
    drain_serial(uart.as_mut());
    eprintln!("  [pipeline-host] UART open, waiting for first READY...");
    loop {
        let line = read_line(uart.as_mut())?;
        if line == "READY" { break; }
    }

    // NOW open USB CDC — device has re-enumerated after flash.
    let usb_port = find_port_by_vid_pid(0x16c0, 0x27dd, 5000)
        .ok_or("USB CDC port (16c0:27dd) not found")?;
    eprintln!("  [pipeline-host] USB CDC: {usb_port}");
    let mut usb = serialport::new(&usb_port, 115_200)
        .timeout(Duration::from_secs(5))
        .open()
        .map_err(|e| format!("cannot open USB CDC {usb_port}: {e}"))?;
    usb.write_data_terminal_ready(true).ok();
    drain_serial(usb.as_mut());

    // Send data for the first READY we already consumed.
    uart.write_all(&pattern)
        .map_err(|e| format!("UART write error: {e}"))?;
    let mut first = vec![0u8; 32];
    usb.read_exact(&mut first)
        .map_err(|e| format!("USB CDC read error (first): {e}"))?;

    let mut ok = if first == pattern { 1usize } else { 0 };
    let mut err = if first != pattern { 1usize } else { 0 };
    let t0 = Instant::now();

    while ok + err < HOST_CYCLES {
        let line = read_line(uart.as_mut())?;
        if line != "READY" {
            continue;
        }
        uart.write_all(&pattern)
            .map_err(|e| format!("UART write error: {e}"))?;
        let mut received = vec![0u8; 32];
        usb.read_exact(&mut received)
            .map_err(|e| format!("USB CDC read error: {e}"))?;
        if received == pattern {
            ok += 1;
        } else {
            err += 1;
            eprintln!("  [pipeline-host] data mismatch (ok={ok} err={err})");
        }
    }

    println!(
        "  [pipeline-host] done: OK={ok} ERR={err} cycles={HOST_CYCLES} {:.1}s",
        t0.elapsed().as_secs_f64()
    );
    if err == 0 { Ok(()) } else { Err(format!("pipeline: OK={ok} ERR={err}")) }
}

struct TestCase {
    name: &'static str,
    features: &'static str,
    success_pattern: &'static str,
    timeout_secs: u64,
    /// Use `--release` profile (and the release ELF path).
    release: bool,
    /// Host-side function variant.
    host: HostKind,
}

enum HostKind {
    /// No host-side counterpart needed (self-contained loopback or standalone).
    None,
    /// Run a host function in a background thread.
    Fn(HostFn),
    /// Like Fn, but first detect the new USB CDC ttyACM port that appears
    /// after firmware boots, then call `fn(usb_port: &str)`.
    UsbPipeline,
}

const TESTS: &[TestCase] = &[
    TestCase {
        name: "semaphore_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Semaphore",
        timeout_secs: 20,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "mutex_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Mutex",
        timeout_secs: 20,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "events_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Events working",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "events_dprint_demo",
        features: "partition-debug",
        success_pattern: "SUCCESS: events + dprint",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "msg_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Message queues working",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "queuing_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Queuing",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "blackboard_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Blackboard",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "sampling_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Sampling ports working",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "plib_demo",
        features: "partition-debug",
        success_pattern: "SUCCESS: plib",
        timeout_secs: 30,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        name: "mpu_kernel_demo",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: MPU",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    // --- UART / DMA / USB tests ---
    TestCase {
        // USART3 HDSEL loopback: no external host needed.
        name: "uart_partition",
        features: "kernel-irq-mpu-hal",
        success_pattern: "SUCCESS: UART IRQ partition working",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        // USART3 HDSEL loopback + DMA: self-contained, no host.
        name: "buf_dma_demo",
        features: "kernel-mpu-hal",
        success_pattern: "SUCCESS: HAL zero-copy DMA buffer pool working",
        timeout_secs: 20,
        release: true,
        host: HostKind::None,
    },
    TestCase {
        // MCU arms DMA → sends READY → host sends 32B → DMA fills kernel buf.
        name: "uart_dma_demo",
        features: "kernel-mpu-hal",
        success_pattern: "SUCCESS: real-wires UART DMA working",
        timeout_secs: 30,
        release: true,
        host: HostKind::Fn(host_uart_dma_demo),
    },
    TestCase {
        // Full-duplex DMA echo: host sends 32B, MCU echoes same buf via TX DMA.
        name: "uart_dma_echo",
        features: "kernel-mpu-hal",
        success_pattern: "SUCCESS: full-duplex UART DMA echo working",
        timeout_secs: 30,
        release: true,
        host: HostKind::Fn(host_uart_dma_echo),
    },
    TestCase {
        // UART DMA → kernel buf → USB CDC pipeline.
        // Host opens UART + USB CDC before probe-rs flashes, avoiding
        // protocol desync from late CDC detection.
        name: "uart_usb_pipeline",
        features: "kernel-usb-mpu",
        success_pattern: "SUCCESS: UART\u{2192}USB pipeline working",
        timeout_secs: 45,
        release: true,
        host: HostKind::Fn(host_uart_usb_pipeline),
    },
    TestCase {
        // FPU context isolation: two partitions load distinct s0-s31 patterns
        // and verify after each context switch. Tests both hardware-saved
        // (s0-s15, lazy stacking) and software-saved (s16-s31, PendSV) regs.
        name: "fpu_context_test",
        features: "kernel-fpu",
        success_pattern: "SUCCESS: FPU context switch working",
        timeout_secs: 15,
        release: false,
        host: HostKind::None,
    },
    TestCase {
        // IWDG watchdog owned by kernel partition under MPU enforcement.
        name: "wdt_partition",
        features: "kernel-mpu-hal",
        success_pattern: "SUCCESS: WDT partition running under MPU",
        timeout_secs: 15,
        release: true,
        host: HostKind::None,
    },
    TestCase {
        // Error handler: P0 faults, handler queries status, requests warm restart.
        name: "fault_restart_demo",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: fault restart demo working",
        // Post-fault tick rate is sensitive to code alignment (67-117 Hz
        // depending on layout). 30s accommodates the worst case.
        timeout_secs: 30,
        release: true,
        host: HostKind::None,
    },
    TestCase {
        // Health monitor: P2 queries run counts, schedule info, time.
        // Note: run_count returns 0 (bug 38) — test passes on other introspection.
        name: "health_monitor_demo",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: health monitor working",
        timeout_secs: 60, // probe-rs RTT slow on this binary (~11s per message)
        release: true,
        host: HostKind::None,
    },
];

fn check_openocd() {
    if TcpStream::connect_timeout(
        &"127.0.0.1:3333".parse().unwrap(),
        Duration::from_millis(200),
    )
    .is_ok()
    {
        eprintln!("Error: OpenOCD is running on port 3333. Stop it first, then re-run.");
        eprintln!("  sudo kill $(lsof -ti:3333)   # or Ctrl+C in your OpenOCD terminal");
        std::process::exit(1);
    }
}

fn f429zi_dir() -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    Path::new(manifest).parent().unwrap().to_path_buf()
}

fn elf_path(f429zi: &Path, name: &str, release: bool) -> PathBuf {
    let profile = if release { "release" } else { "debug" };
    // f429zi is excluded from the boards/ workspace so its artifacts land in
    // f429zi/target/, not boards/target/.
    f429zi.join("target")
        .join(format!("thumbv7em-none-eabihf/{profile}/examples"))
        .join(name)
}

fn build(f429zi: &Path, tc: &TestCase) -> Result<Duration, String> {
    let t0 = Instant::now();
    let mut args = vec![
        "build",
        "--example",
        tc.name,
        "--features",
        tc.features,
        "--no-default-features",
    ];
    if tc.release {
        args.push("--release");
    }
    let status = Command::new("cargo")
        .args(&args)
        .current_dir(f429zi)
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status()
        .map_err(|e| format!("failed to spawn cargo: {e}"))?;

    if status.success() {
        Ok(t0.elapsed())
    } else {
        Err(format!("cargo build exited with {status}"))
    }
}

/// Find a serial port matching a specific USB VID:PID, with retry up to `timeout_ms`.
/// This is more robust than detecting "new" ports because firmware re-enumeration
/// often reuses the same device node (e.g. after reflash with same VID:PID).
fn find_port_by_vid_pid(vid: u16, pid: u16, timeout_ms: u64) -> Option<String> {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    loop {
        if let Ok(ports) = serialport::available_ports() {
            for p in ports {
                if let serialport::SerialPortType::UsbPort(info) = &p.port_type {
                    if info.vid == vid && info.pid == pid {
                        return Some(p.port_name);
                    }
                }
            }
        }
        if Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
}

fn flash_and_run(f429zi: &Path, tc: &TestCase) -> Result<Duration, String> {
    let elf = elf_path(f429zi, tc.name, tc.release);
    let timeout = Duration::from_secs(tc.timeout_secs);
    let t0 = Instant::now();

    // (USB CDC detection now uses VID:PID, not "new port" comparison.)

    let mut child = Command::new("probe-rs")
        .args([
            "run",
            "--probe",
            PROBE_VID_PID,
            "--chip",
            CHIP,
            elf.to_str().unwrap(),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|e| format!("failed to spawn probe-rs: {e}"))?;

    let stdout = child.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel::<String>();

    std::thread::spawn(move || {
        let reader = BufReader::new(stdout);
        for line in reader.lines() {
            match line {
                Ok(l) => {
                    if tx.send(l).is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
    });

    // Channel for host thread result (if any).
    let (host_tx, host_rx) = mpsc::channel::<Result<(), String>>();

    // For Fn-style tests, spawn the host thread BEFORE probe-rs starts.
    // probe-rs takes ~3s to flash, giving the host time to open the serial
    // port and drain any stale bytes before the firmware sends its first READY.
    // This avoids the first-cycle DMA timeout caused by host connection latency.
    let has_host_thread = match &tc.host {
        HostKind::None => false,
        HostKind::Fn(f) => {
            let f = *f;
            let htx = host_tx.clone();
            std::thread::spawn(move || {
                let _ = htx.send(f());
            });
            true
        }
        HostKind::UsbPipeline => {
            // USB CDC port isn't known yet — will spawn after first RTT line.
            false
        }
    };

    let deadline = t0 + timeout;
    let mut success = false;
    let mut usb_spawned = false;

    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match rx.recv_timeout(remaining) {
            Ok(line) => {
                let elapsed = t0.elapsed();
                println!("    [{:6.0}ms] {line}", elapsed.as_secs_f64() * 1000.0);

                // For USB pipeline: spawn after first RTT line, detect CDC by VID:PID.

                if line.contains(tc.success_pattern) {
                    success = true;
                    break;
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => break,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    let has_host_thread = has_host_thread || usb_spawned;

    // Kill probe-rs.
    let _ = child.kill();
    let _ = child.wait();

    // If a host thread was spawned, report its outcome (non-blocking check).
    if has_host_thread {
        match host_rx.recv_timeout(Duration::from_secs(2)) {
            Ok(Ok(())) => {} // host completed successfully
            Ok(Err(e)) => eprintln!("  [host] error: {e}"),
            Err(_) => eprintln!("  [host] thread did not finish within grace period"),
        }
    }

    if success {
        Ok(t0.elapsed())
    } else {
        Err(format!("timeout after {:.1}s", timeout.as_secs_f64()))
    }
}

fn run_test(f429zi: &Path, tc: &TestCase, no_build: bool) -> bool {
    if !no_build {
        let profile = if tc.release { "release" } else { "debug" };
        print!("  Building ({profile})... ");
        match build(f429zi, tc) {
            Ok(d) => println!(" OK  ({:.1}s)", d.as_secs_f64()),
            Err(e) => {
                println!(" FAIL");
                println!("  Build error: {e}");
                return false;
            }
        }
    }

    println!("  Flashing and running (timeout {}s)...", tc.timeout_secs);
    match flash_and_run(f429zi, tc) {
        Ok(d) => {
            println!("  PASS  ({:.1}s)", d.as_secs_f64());
            true
        }
        Err(e) => {
            println!("  FAIL: {e}");
            false
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();

    if args.iter().any(|a| a == "--help" || a == "-h") {
        println!("Usage: hw-tests [--no-build] [--help] [test-name ...]");
        println!();
        println!("Options:");
        println!("  --no-build   Skip cargo build step (use existing ELFs)");
        println!("  --help       Show this help");
        println!();
        println!("Positional args: run only the named tests (default: all)");
        println!();
        println!("Available tests:");
        for tc in TESTS {
            let profile = if tc.release { "release" } else { "debug" };
            println!("  {} ({}, {})", tc.name, tc.features, profile);
        }
        return;
    }

    let no_build = args.iter().any(|a| a == "--no-build");
    let filter: Vec<&str> = args
        .iter()
        .filter(|a| !a.starts_with("--"))
        .map(String::as_str)
        .collect();

    let tests: Vec<&TestCase> = if filter.is_empty() {
        TESTS.iter().collect()
    } else {
        for name in &filter {
            if !TESTS.iter().any(|tc| tc.name == *name) {
                eprintln!("Unknown test: '{name}'");
                eprintln!("Run with --help to list available tests.");
                std::process::exit(1);
            }
        }
        TESTS.iter().filter(|tc| filter.contains(&tc.name)).collect()
    };

    check_openocd();

    let f429zi = f429zi_dir();

    println!("=== STM32F429ZI Hardware Regression Tests ===");
    println!("Chip: {CHIP}   Probe: {PROBE_VID_PID}");
    println!();

    let total = tests.len();
    let mut passed = 0usize;
    let mut results: Vec<(&str, bool)> = Vec::new();

    for (i, tc) in tests.iter().enumerate() {
        let profile = if tc.release { "release" } else { tc.features };
        println!("[{}/{}] {}  ({})", i + 1, total, tc.name, profile);
        let ok = run_test(&f429zi, tc, no_build);
        results.push((tc.name, ok));
        if ok {
            passed += 1;
        }
        println!();
    }

    println!("{}", "═".repeat(38));
    println!("  RESULTS");
    println!("{}", "═".repeat(38));
    for (name, ok) in &results {
        let mark = if *ok { "✓ PASS" } else { "✗ FAIL" };
        println!("  {mark}   {name}");
    }
    println!();
    println!("  {passed}/{total} passed");

    if passed < total {
        std::process::exit(1);
    }
}
