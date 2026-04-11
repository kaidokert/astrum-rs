# STM32F429ZI USB CDC Examples

Embedded Rust examples for USB CDC communication on NUCLEO-F429ZI board.

## Hardware Requirements

- NUCLEO-F429ZI board
- JP4 jumper **must be closed** (default)
- USB cable connected to CN13 (User USB connector)

## Examples

### 1. `usb_serial.rs` - Basic Echo

Simple USB CDC echo example that demonstrates:
- Interrupt-driven USB communication
- Basic data echo functionality
- Atomic counters for debugging (IRQ, POLL, RX)

**Run:**
```bash
cargo run --release --example usb_serial
```

**Test:**
```bash
# Find device
ls -l /dev/ttyACM*

# Send data (will be echoed back)
echo "Hello USB" > /dev/ttyACM0
cat /dev/ttyACM0
```

### 2. `usb_commands.rs` - Command Interface

Full command-line interface over USB CDC with:
- Text-based command protocol
- LED control (on/off/toggle)
- Status reporting (uptime, IRQ count)
- Echo command
- Help system

**Run:**
```bash
cargo run --release --example usb_commands
```

**Available Commands:**
- `help` - Show available commands
- `led on` - Turn LED on
- `led off` - Turn LED off
- `led toggle` - Toggle LED state
- `status` - Show device status
- `echo <text>` - Echo back text

**Using the Python Host Script:**

Interactive mode:
```bash
./host_serial.py
# Opens interactive prompt
stm32> help
stm32> led on
stm32> status
stm32> quit
```

Single command:
```bash
./host_serial.py "status"
./host_serial.py "led toggle"
```

Specify port:
```bash
./host_serial.py -p /dev/ttyACM0
```

**Manual Testing:**
```bash
# Using screen
screen /dev/ttyACM0 115200

# Using minicom
minicom -D /dev/ttyACM0 -b 115200

# Using Python directly
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',115200); \
s.write(b'help\n'); import time; time.sleep(0.1); print(s.read(s.in_waiting).decode())"
```

## Clock Configuration

- **System Clock:** 168 MHz
- **USB Clock:** 48 MHz (PLL Q divider)
- **HSE:** 8 MHz (from ST-LINK MCO)

## USB Configuration

- **Peripheral:** OTG_FS (Full Speed)
- **Pins:**
  - PA11: USB_DM
  - PA12: USB_DP
  - PA9: VBUS (sensing)
  - PA10: ID (OTG)
- **VID:PID:** 0x16c0:0x27dd
- **Clock:** Manually enabled in RCC AHB2ENR

## Debugging

Both examples use RTT (Real-Time Transfer) for debug output via probe-rs:
```bash
# In another terminal
probe-rs attach --chip STM32F429ZITx
```

## Troubleshooting

**Device not enumerated:**
- Check JP4 jumper is closed
- Verify USB cable is connected to CN13 (User USB), not CN1 (ST-LINK)
- Check `dmesg | tail` for USB enumeration messages

**No interrupts firing:**
- Run `cargo clean` and rebuild
- Verify USB clock is enabled (RCC AHB2ENR)
- Check NVIC interrupt is unmasked

**Permission denied on /dev/ttyACM*:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```
