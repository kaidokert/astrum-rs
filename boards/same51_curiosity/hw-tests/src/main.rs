//! Hardware regression tests for SAME51 Curiosity (ATSAME51J20A)
//!
//! Builds, strips .defmt section (avoids probe-rs defmt parsing confusion),
//! flashes via probe-rs, and validates RTT output. Tests with host protocols
//! (UART DMA, USB CDC) spawn serial threads alongside probe-rs.
//!
//! Run: cd boards/same51_curiosity/hw-tests && cargo run

use std::{
    io::{BufRead, BufReader, Read, Write},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::mpsc,
    time::{Duration, Instant},
};

const PROBE_VID_PID: &str = "03eb:2175";
const CHIP: &str = "ATSAME51J20A";
const SPEED: &str = "1000";

// nEDBG VCOM (SERCOM2 PA12/PA13) for UART DMA host protocols.
const VCOM_VID: u16 = 0x03eb;
const VCOM_PID: u16 = 0x2175;
const VCOM_BAUD: u32 = 115_200;
const HOST_CYCLES: usize = 20;

// USB CDC from SAME51 target (PA24/PA25).
const USB_CDC_VID: u16 = 0x16c0;
const USB_CDC_PID: u16 = 0x27dd;

type HostFn = fn(&mut dyn serialport::SerialPort) -> Result<(), String>;

enum HostKind {
    None,
    Fn(HostFn),
    UsbEcho,
}

struct TestCase {
    name: &'static str,
    features: &'static str,
    success_pattern: &'static str,
    timeout_secs: u64,
    release: bool,
    strip_defmt: bool,
    host: HostKind,
}

// ---------------------------------------------------------------------------
// Host protocol helpers
// ---------------------------------------------------------------------------
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

fn drain_serial(port: &mut dyn serialport::SerialPort) {
    port.clear(serialport::ClearBuffer::Input).ok();
    std::thread::sleep(Duration::from_millis(100));
    port.clear(serialport::ClearBuffer::Input).ok();
}

fn read_line(port: &mut dyn serialport::SerialPort) -> Result<String, String> {
    let mut buf = Vec::new();
    let mut b = [0u8; 1];
    loop {
        port.read_exact(&mut b).map_err(|e| format!("serial read: {e}"))?;
        if b[0] == b'\n' { break; }
        if b[0] != b'\r' { buf.push(b[0]); }
        if buf.len() > 128 { buf.clear(); }
    }
    Ok(String::from_utf8_lossy(&buf).trim().to_string())
}

fn open_vcom() -> Result<Box<dyn serialport::SerialPort>, String> {
    let port_name = find_port_by_vid_pid(VCOM_VID, VCOM_PID, 3000)
        .ok_or("nEDBG VCOM not found")?;
    let mut port = serialport::new(&port_name, VCOM_BAUD)
        .timeout(Duration::from_secs(5))
        .open()
        .map_err(|e| format!("cannot open {port_name}: {e}"))?;
    // Assert DTR — some CDC bridges (nEDBG) won't forward data without it.
    port.write_data_terminal_ready(true).ok();
    Ok(port)
}

/// Run a Python UART host protocol. The script opens the serial port,
/// calls probe-rs reset (which starts the firmware), then runs the protocol.
/// This ensures the port is open BEFORE READY is sent.
/// UART DMA host: open port (MCU halted after download), caller does reset
/// after we return the port. Then protocol runs: READY → 32B → OK.
fn host_uart_dma(port: &mut dyn serialport::SerialPort) -> Result<(), String> {
    let pattern: Vec<u8> = (0u8..32).collect();
    let mut ok = 0usize;
    for _ in 0..(HOST_CYCLES + 10) {
        let line = read_line(port)?;
        if line == "READY" {
            port.write_all(&pattern).map_err(|e| format!("write: {e}"))?;
            let resp = read_line(port)?;
            if resp == "OK" { ok += 1; }
            if ok >= HOST_CYCLES { break; }
        }
    }
    if ok >= HOST_CYCLES { Ok(()) }
    else { Err(format!("only {ok}/{HOST_CYCLES} cycles")) }
}

/// UART DMA echo host: READY → 32B → echo 32B → OK.
fn host_uart_dma_echo(port: &mut dyn serialport::SerialPort) -> Result<(), String> {
    let pattern: Vec<u8> = (0u8..32).collect();
    let mut ok = 0usize;
    for _ in 0..(HOST_CYCLES + 10) {
        let line = read_line(port)?;
        if line == "READY" {
            port.write_all(&pattern).map_err(|e| format!("write: {e}"))?;
            let mut echo = vec![0u8; 32];
            port.read_exact(&mut echo).map_err(|e| format!("echo read: {e}"))?;
            if echo == pattern {
                let resp = read_line(port)?;
                if resp == "OK" { ok += 1; }
            }
            if ok >= HOST_CYCLES { break; }
        }
    }
    if ok >= HOST_CYCLES { Ok(()) }
    else { Err(format!("only {ok}/{HOST_CYCLES} echo cycles")) }
}

/// USB CDC echo host: send text, expect prefixed echo back.
fn host_usb_echo(prefix: &str) -> Result<(), String> {
    let port_name = find_port_by_vid_pid(USB_CDC_VID, USB_CDC_PID, 10_000)
        .ok_or("USB CDC port not found")?;
    eprintln!("  [usb] detected CDC: {port_name}");
    let mut port = serialport::new(&port_name, 115_200)
        .timeout(Duration::from_secs(3))
        .open()
        .map_err(|e| format!("cannot open {port_name}: {e}"))?;
    std::thread::sleep(Duration::from_millis(500));
    drain_serial(port.as_mut());

    let mut ok = 0;
    for i in 0..5 {
        let msg = format!("t{i}\n");
        port.write_all(msg.as_bytes()).map_err(|e| format!("write: {e}"))?;
        port.flush().ok();
        std::thread::sleep(Duration::from_millis(300));
        let mut buf = [0u8; 128];
        let n = port.read(&mut buf).unwrap_or(0);
        let resp = String::from_utf8_lossy(&buf[..n]);
        if resp.contains(prefix) { ok += 1; }
    }
    if ok >= 3 { Ok(()) } else { Err(format!("only {ok}/5 echoes with {prefix}")) }
}

// ---------------------------------------------------------------------------
// Test definitions
// ---------------------------------------------------------------------------
const TESTS: &[TestCase] = &[
    // --- RTT-only self-contained tests ---
    TestCase {
        name: "astrum_blinky",
        features: "partition-debug",
        success_pattern: "SUCCESS: SAME51 astrum_blinky",
        timeout_secs: 35,
        release: true,
        strip_defmt: true,
        host: HostKind::None,
    },
    TestCase {
        name: "mpu_kernel_demo",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: SAME51 MPU",
        timeout_secs: 35,
        release: true,
        strip_defmt: true,
        host: HostKind::None,
    },
    TestCase {
        name: "wdt_partition",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: WDT partition",
        timeout_secs: 35,
        release: true,
        strip_defmt: true,
        host: HostKind::None,
    },
    // --- UART DMA tests (host protocol via nEDBG VCOM) ---
    TestCase {
        name: "uart_dma_demo",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: SAME51 UART DMA",
        timeout_secs: 45,
        release: true,
        strip_defmt: true,
        host: HostKind::Fn(host_uart_dma),
    },
    TestCase {
        name: "uart_dma_transfer_demo",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: SAME51 UART DMA transfer",
        timeout_secs: 45,
        release: true,
        strip_defmt: true,
        host: HostKind::Fn(host_uart_dma),
    },
    TestCase {
        name: "uart_dma_echo",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: SAME51 full-duplex DMA echo",
        timeout_secs: 45,
        release: true,
        strip_defmt: true,
        host: HostKind::Fn(host_uart_dma_echo),
    },
    // --- USB CDC tests ---
    TestCase {
        name: "usb_cdc_partition_d",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: Option D USB CDC",
        timeout_secs: 30,
        release: true,
        strip_defmt: true,
        host: HostKind::UsbEcho,
    },
    TestCase {
        name: "usb_cdc_partition_ack",
        features: "kernel-mpu",
        success_pattern: "SUCCESS: USB CDC PartitionAcks",
        timeout_secs: 30,
        release: true,
        strip_defmt: true,
        host: HostKind::UsbEcho,
    },
    // --- FPU + MPU isolation tests ---
    TestCase {
        name: "fpu_context_test",
        features: "kernel-fpu",
        success_pattern: "SUCCESS: FPU context switch working",
        timeout_secs: 35,
        release: true,
        strip_defmt: true,
        host: HostKind::None,
    },
    TestCase {
        name: "adversarial_test",
        features: "kernel-example",
        success_pattern: "SUCCESS: adversarial test passed",
        timeout_secs: 35,
        release: true,
        strip_defmt: true,
        host: HostKind::None,
    },
];

// ---------------------------------------------------------------------------
// Runner infrastructure
// ---------------------------------------------------------------------------
fn strip_defmt_section(elf: &Path) -> PathBuf {
    let stripped = elf.with_extension("stripped");
    let status = Command::new("arm-none-eabi-objcopy")
        .args(["--remove-section", ".defmt"])
        .arg(elf)
        .arg(&stripped)
        .status()
        .expect("arm-none-eabi-objcopy");
    assert!(status.success(), "objcopy failed");
    stripped
}

fn same51_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

fn elf_path(same51: &Path, name: &str, release: bool) -> PathBuf {
    let profile = if release { "release" } else { "debug" };
    same51
        .join("target")
        .join(format!("thumbv7em-none-eabihf/{profile}/examples"))
        .join(name)
}

fn build(same51: &Path, tc: &TestCase) -> Result<Duration, String> {
    let t0 = Instant::now();
    let mut args = vec!["build", "--example", tc.name, "--features", tc.features];
    if tc.release {
        args.push("--release");
    }
    let status = Command::new("cargo")
        .args(&args)
        .current_dir(same51)
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status()
        .map_err(|e| format!("cargo: {e}"))?;
    if status.success() {
        Ok(t0.elapsed())
    } else {
        Err(format!("build failed: {status}"))
    }
}

/// Flash with `probe-rs download` + `probe-rs reset`, then use `probe-rs run`
/// for RTT. For tests with host serial protocols, use download+reset mode
/// (no RTT) because `probe-rs run` locks the nEDBG VCOM.
fn flash_and_run(same51: &Path, tc: &TestCase) -> Result<Duration, String> {
    let elf = elf_path(same51, tc.name, tc.release);
    let flash_elf = if tc.strip_defmt {
        strip_defmt_section(&elf)
    } else {
        elf.clone()
    };
    let timeout = Duration::from_secs(tc.timeout_secs);
    let t0 = Instant::now();

    let needs_serial = !matches!(tc.host, HostKind::None);

    if needs_serial {
        // Serial tests: download + reset (frees nEDBG VCOM), verify via host protocol.
        return flash_and_run_serial(same51, tc, &flash_elf, timeout);
    }

    // RTT-only tests: use probe-rs run for RTT output.
    let mut child = Command::new("probe-rs")
        .args([
            "run",
            "--probe", PROBE_VID_PID,
            "--chip", CHIP,
            "--speed", SPEED,
            "--no-catch-reset",
            "--no-catch-hardfault",
            flash_elf.to_str().unwrap(),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|e| format!("probe-rs: {e}"))?;

    let stdout = child.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel::<String>();

    std::thread::spawn(move || {
        for line in BufReader::new(stdout).lines().map_while(Result::ok) {
            if tx.send(line).is_err() { break; }
        }
    });

    let deadline = t0 + timeout;
    let mut success = false;

    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() { break; }
        match rx.recv_timeout(remaining) {
            Ok(line) => {
                println!("    [{:6.0}ms] {line}", t0.elapsed().as_millis());
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

    if success { Ok(t0.elapsed()) }
    else { Err(format!("timeout after {:.1}s", timeout.as_secs_f64())) }
}

/// Serial-protocol tests: download + reset (frees nEDBG VCOM for serial access).
/// Success is determined by the host protocol completing, not RTT.
///
/// nEDBG quirk: `probe-rs run` locks the VCOM while attached for RTT.
/// So we use `download` + `reset` (which release the probe), then the host
/// thread opens the serial port and runs the protocol.
fn flash_and_run_serial(_same51: &Path, tc: &TestCase, flash_elf: &Path, timeout: Duration) -> Result<Duration, String> {
    let t0 = Instant::now();

    // Step 1: Download (MCU halted after flash — no READY sent yet).
    let dl = Command::new("probe-rs")
        .args(["download", "--probe", PROBE_VID_PID, "--chip", CHIP, "--speed", SPEED,
            flash_elf.to_str().unwrap()])
        .output()
        .map_err(|e| format!("probe-rs download: {e}"))?;
    if !dl.status.success() {
        return Err(format!("probe-rs download failed: {}", dl.status));
    }

    // Step 2: For UART tests — open port while MCU is halted, then reset.
    // For USB tests — reset first, then spawn echo thread.
    match &tc.host {
        HostKind::Fn(f) => {
            // Open serial port now (MCU halted, no READY yet).
            let mut port = open_vcom()?;
            drain_serial(port.as_mut());
            eprintln!("  [uart] port open, resetting...");

            // Reset — firmware starts, sends READY into open port.
            let rst = Command::new("probe-rs")
                .args(["reset", "--probe", PROBE_VID_PID, "--chip", CHIP])
                .output()
                .map_err(|e| format!("probe-rs reset: {e}"))?;
            if !rst.status.success() {
                return Err(format!("probe-rs reset failed: {}", rst.status));
            }

            // Run protocol.
            let result = f(port.as_mut());
            return match result {
                Ok(()) => { println!("    [host protocol OK]"); Ok(t0.elapsed()) }
                Err(e) => Err(format!("host protocol: {e}")),
            };
        }
        HostKind::UsbEcho => {
            // USB: reset first, then spawn echo thread.
            let rst = Command::new("probe-rs")
                .args(["reset", "--probe", PROBE_VID_PID, "--chip", CHIP])
                .output()
                .map_err(|e| format!("probe-rs reset: {e}"))?;
            if !rst.status.success() {
                return Err(format!("probe-rs reset failed: {}", rst.status));
            }

            let (host_tx, host_rx) = mpsc::channel::<Result<(), String>>();
            let htx = host_tx.clone();
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_secs(5));
                let _ = htx.send(host_usb_echo("[ACK]").or_else(|_| host_usb_echo("[RTOS-D]")));
            });

            println!("    [reset done, USB host running...]");
            let remaining = timeout.saturating_sub(t0.elapsed());
            match host_rx.recv_timeout(remaining) {
                Ok(Ok(())) => { println!("    [host protocol OK]"); Ok(t0.elapsed()) }
                Ok(Err(e)) => Err(format!("host protocol: {e}")),
                Err(_) => Err(format!("timeout after {:.1}s", timeout.as_secs_f64())),
            }
        }
        HostKind::None => unreachable!(),
    }
}

fn run_test(same51: &Path, tc: &TestCase, no_build: bool) -> bool {
    if !no_build {
        let profile = if tc.release { "release" } else { "debug" };
        print!("  Building ({profile})... ");
        match build(same51, tc) {
            Ok(d) => println!(" OK  ({:.1}s)", d.as_secs_f64()),
            Err(e) => { println!(" FAIL\n  {e}"); return false; }
        }
    }
    println!("  Flashing and running (timeout {}s)...", tc.timeout_secs);
    match flash_and_run(same51, tc) {
        Ok(d) => { println!("  PASS  ({:.1}s)", d.as_secs_f64()); true }
        Err(e) => { println!("  FAIL: {e}"); false }
    }
}

fn check_openocd() {
    for port in [3333, 3334] {
        if std::net::TcpStream::connect_timeout(
            &format!("127.0.0.1:{port}").parse().unwrap(),
            Duration::from_millis(200),
        ).is_ok() {
            eprintln!("Error: OpenOCD running on port {port}. Stop it first.");
            std::process::exit(1);
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();

    if args.iter().any(|a| a == "--help" || a == "-h") {
        println!("Usage: hw-tests [--no-build] [test-name ...]");
        println!("\nAvailable tests:");
        for tc in TESTS {
            println!("  {} ({}, {})", tc.name, tc.features,
                if tc.release { "release" } else { "debug" });
        }
        return;
    }

    let no_build = args.iter().any(|a| a == "--no-build");
    let filter: Vec<&str> = args.iter().filter(|a| !a.starts_with("--")).map(String::as_str).collect();

    let tests: Vec<&TestCase> = if filter.is_empty() {
        TESTS.iter().collect()
    } else {
        TESTS.iter().filter(|tc| filter.contains(&tc.name)).collect()
    };

    check_openocd();

    let same51 = same51_dir();
    println!("=== SAME51 Curiosity Hardware Regression Tests ===");
    println!("Chip: {CHIP}   Probe: {PROBE_VID_PID}\n");

    let total = tests.len();
    let mut passed = 0;
    let mut results = Vec::new();

    for (i, tc) in tests.iter().enumerate() {
        println!("[{}/{}] {}  ({})", i + 1, total, tc.name, tc.features);
        let ok = run_test(&same51, tc, no_build);
        results.push((tc.name, ok));
        if ok { passed += 1; }
        // nEDBG recovery time between tests — probe degrades under rapid cycling.
        std::thread::sleep(Duration::from_secs(2));
        println!();
    }

    println!("{}", "═".repeat(38));
    println!("  RESULTS");
    println!("{}", "═".repeat(38));
    for (name, ok) in &results {
        println!("  {} {name}", if *ok { "✓ PASS  " } else { "✗ FAIL  " });
    }
    println!("\n  {passed}/{total} passed");

    if passed < total { std::process::exit(1); }
}
