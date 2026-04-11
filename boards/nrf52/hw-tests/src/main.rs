use std::{
    io::{BufRead, BufReader, Read},
    net::TcpStream,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    },
    time::{Duration, Instant},
};

const PROBE_VID_PID: &str = "1366:1025";
const CHIP: &str = "nRF52833_xxAA";

// J-Link VCOM on nRF52833-DK (same VID:PID as debug probe, separate CDC interface).
const VCOM_VID: u16 = 0x1366;
const VCOM_PID: u16 = 0x1025;

type HostFn = fn(stop: Arc<AtomicBool>) -> Result<(), String>;

enum HostKind {
    None,
    Fn(HostFn),
}

struct TestCase {
    name: &'static str,
    features: &'static str,
    success_pattern: &'static str,
    timeout_secs: u64,
    host: HostKind,
}

// ---------------------------------------------------------------------------
// Host protocol: UARTE echo
//
// Firmware sends 4 bytes via UARTE0 EasyDMA, host reads them from J-Link
// VCOM and echoes back.  Firmware verifies the echo matches.
// Runs concurrently with probe-rs (J-Link debug + VCOM are independent).
// ---------------------------------------------------------------------------
fn find_vcom(timeout_ms: u64) -> Option<String> {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    loop {
        if let Ok(ports) = serialport::available_ports() {
            for p in ports {
                if let serialport::SerialPortType::UsbPort(info) = &p.port_type {
                    if info.vid == VCOM_VID && info.pid == VCOM_PID {
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

const UARTE_ECHO_CYCLES: usize = 20;

fn host_uarte_echo(stop: Arc<AtomicBool>) -> Result<(), String> {
    let port_name = find_vcom(5000).ok_or("J-Link VCOM not found")?;
    eprintln!("  [uart] VCOM: {port_name}");
    let mut port = serialport::new(&port_name, 115_200)
        .timeout(Duration::from_secs(2))
        .open()
        .map_err(|e| format!("cannot open {port_name}: {e}"))?;
    port.write_data_terminal_ready(true).ok();

    // Drain any stale data.
    port.clear(serialport::ClearBuffer::Input).ok();
    std::thread::sleep(Duration::from_millis(200));
    port.clear(serialport::ClearBuffer::Input).ok();

    let mut buf = [0u8; 4];
    let mut ok = 0usize;
    while !stop.load(Ordering::Relaxed) {
        match port.read_exact(&mut buf) {
            Ok(()) => {
                // Echo back what we received.
                port.write_all(&buf).map_err(|e| format!("write: {e}"))?;
                ok += 1;
                if ok >= UARTE_ECHO_CYCLES {
                    eprintln!("  [uart] {ok} echo cycles OK");
                    return Ok(());
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(format!("serial read: {e}")),
        }
    }
    if ok >= UARTE_ECHO_CYCLES { Ok(()) }
    else { Err(format!("only {ok}/{UARTE_ECHO_CYCLES} echo cycles before stop")) }
}

const TESTS: &[TestCase] = &[
    TestCase {
        name: "semaphore_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Semaphore",
        timeout_secs: 20,
        host: HostKind::None,
    },
    TestCase {
        name: "mutex_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Mutex",
        timeout_secs: 20,
        host: HostKind::None,
    },
    TestCase {
        name: "events_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Events working",
        timeout_secs: 15,
        host: HostKind::None,
    },
    TestCase {
        name: "msg_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Message queues working",
        timeout_secs: 15,
        host: HostKind::None,
    },
    TestCase {
        name: "queuing_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Queuing",
        timeout_secs: 15,
        host: HostKind::None,
    },
    TestCase {
        name: "blackboard_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Blackboard",
        timeout_secs: 15,
        host: HostKind::None,
    },
    TestCase {
        name: "sampling_demo",
        features: "kernel-example",
        success_pattern: "SUCCESS: Sampling ports working",
        timeout_secs: 15,
        host: HostKind::None,
    },
    TestCase {
        name: "fpu_context_test",
        features: "kernel-fpu",
        success_pattern: "SUCCESS: FPU context switch working",
        timeout_secs: 15,
        host: HostKind::None,
    },
    TestCase {
        name: "adversarial_test",
        features: "kernel-example",
        success_pattern: "SUCCESS: adversarial test passed",
        timeout_secs: 15,
        host: HostKind::None,
    },
    TestCase {
        name: "uarte_irq_partition",
        features: "kernel-irq",
        success_pattern: "SUCCESS: UARTE IRQ partition echo verified",
        timeout_secs: 30,
        host: HostKind::Fn(host_uarte_echo),
    },
];

fn check_openocd() {
    if TcpStream::connect_timeout(
        &"127.0.0.1:3333".parse().unwrap(),
        Duration::from_millis(200),
    )
    .is_ok()
    {
        eprintln!("Error: something is listening on port 3333 (OpenOCD?). Stop it first.");
        eprintln!("  sudo kill $(lsof -ti:3333)");
        std::process::exit(1);
    }
}

fn nrf52_dir() -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    Path::new(manifest).parent().unwrap().to_path_buf()
}

fn elf_path(nrf52: &Path, name: &str) -> PathBuf {
    nrf52.join("target")
        .join("thumbv7em-none-eabihf/debug/examples")
        .join(name)
}

fn build(nrf52: &Path, tc: &TestCase) -> Result<Duration, String> {
    let t0 = Instant::now();
    let status = Command::new("cargo")
        .args([
            "build",
            "--example",
            tc.name,
            "--features",
            tc.features,
            "--no-default-features",
        ])
        .current_dir(nrf52)
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

fn flash_and_run(nrf52: &Path, tc: &TestCase) -> Result<Duration, String> {
    let elf = elf_path(nrf52, tc.name);
    let timeout = Duration::from_secs(tc.timeout_secs);
    let t0 = Instant::now();

    // Serial tests need download+reset mode: probe-rs run locks the J-Link
    // VCOM, preventing the host serial thread from accessing ttyACM0.
    // Use download (flash) + reset (release probe) so VCOM is accessible.
    if let HostKind::Fn(f) = &tc.host {
        return flash_and_run_serial(nrf52, tc, &elf, timeout, *f);
    }

    // RTT-only tests: use probe-rs run for RTT output.
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

    let deadline = t0 + timeout;
    let mut success = false;

    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match rx.recv_timeout(remaining) {
            Ok(line) => {
                let elapsed = t0.elapsed();
                println!("    [{:6.0}ms] {line}", elapsed.as_secs_f64() * 1000.0);
                if line.contains(tc.success_pattern) {
                    success = true;
                    break;
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => break,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    let _ = child.kill();
    let _ = child.wait();

    if success {
        Ok(t0.elapsed())
    } else {
        Err(format!("timeout after {:.1}s", timeout.as_secs_f64()))
    }
}

/// Serial-protocol tests: download + reset (frees J-Link VCOM for serial access).
/// Success is determined by the host protocol completing within timeout.
/// No RTT available in this mode — firmware reports success via UART protocol.
fn flash_and_run_serial(
    _nrf52: &Path,
    tc: &TestCase,
    elf: &Path,
    timeout: Duration,
    host_fn: fn(Arc<AtomicBool>) -> Result<(), String>,
) -> Result<Duration, String> {
    let t0 = Instant::now();

    // Step 1: Download (MCU halted after flash).
    let dl = Command::new("probe-rs")
        .args(["download", "--probe", PROBE_VID_PID, "--chip", CHIP,
            elf.to_str().unwrap()])
        .output()
        .map_err(|e| format!("probe-rs download: {e}"))?;
    if !dl.status.success() {
        return Err(format!("download failed: {}", dl.status));
    }

    // Step 2: Open serial port while MCU is halted (no data flowing yet).
    // This ensures the port is ready before firmware starts.
    let stop = Arc::new(AtomicBool::new(false));
    let stop_clone = stop.clone();
    let host_handle = std::thread::spawn(move || host_fn(stop_clone));

    // Brief delay for serial port to open.
    std::thread::sleep(Duration::from_millis(500));

    // Step 3: Reset — firmware starts, begins TX+RX echo protocol.
    let rst = Command::new("probe-rs")
        .args(["reset", "--probe", PROBE_VID_PID, "--chip", CHIP])
        .output()
        .map_err(|e| format!("probe-rs reset: {e}"))?;
    if !rst.status.success() {
        stop.store(true, Ordering::Relaxed);
        let _ = host_handle.join();
        return Err(format!("reset failed: {}", rst.status));
    }

    eprintln!("  [uart] firmware running, host echo active...");

    // Step 4: Wait for timeout, then check result.
    // The firmware prints SUCCESS via RTT (which we can't see in this mode),
    // but we can verify the host protocol ran successfully.
    let remaining = timeout.saturating_sub(t0.elapsed());
    std::thread::sleep(remaining.min(Duration::from_secs(20)));

    stop.store(true, Ordering::Relaxed);
    match host_handle.join() {
        Ok(Ok(())) => {
            println!("    [host echo protocol OK]");
            Ok(t0.elapsed())
        }
        Ok(Err(e)) => Err(format!("host protocol: {e}")),
        Err(_) => Err("host thread panicked".into()),
    }
}

fn run_test(nrf52: &Path, tc: &TestCase, no_build: bool) -> bool {
    if !no_build {
        print!("  Building... ");
        match build(nrf52, tc) {
            Ok(d) => println!(" OK  ({:.1}s)", d.as_secs_f64()),
            Err(e) => {
                println!(" FAIL");
                println!("  Build error: {e}");
                return false;
            }
        }
    }

    println!("  Flashing and running (timeout {}s)...", tc.timeout_secs);
    match flash_and_run(nrf52, tc) {
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
            println!("  {} ({})", tc.name, tc.features);
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

    let nrf52 = nrf52_dir();

    println!("=== nRF52833 Hardware Regression Tests ===");
    println!("Chip: {CHIP}   Probe: {PROBE_VID_PID}");
    println!();

    let total = tests.len();
    let mut passed = 0usize;
    let mut results: Vec<(&str, bool)> = Vec::new();

    for (i, tc) in tests.iter().enumerate() {
        println!("[{}/{}] {}  ({})", i + 1, total, tc.name, tc.features);
        let ok = run_test(&nrf52, tc, no_build);
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
