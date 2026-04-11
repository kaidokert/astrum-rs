# `same51_curiosity`

Bring-up crate for the Microchip Graphics and Touch Curiosity board
(`EV14C17A`, `ATSAME51J20A`).

Current working demos:

- `cargo run --bin hello`
- `cargo run --bin blinky`
- `cargo run --bin vcom_uart`
- `cargo run --bin usb_cdc`

## Host Notes

Validated on:

- `macOS 15.5 (24F74)`
- `probe-rs 0.31.0 (git commit: v0.30.0-193-g489060db-modified)`

For this board's `nEDBG CMSIS-DAP` probe on macOS, stock `probe-rs` was not
enough. The patched build needed the HID handling change from commit:

- `f6160c538ea6c49c68a2f86f7931d079e76628ae`

Relevant patch context:

```rust
fn hid_report_size(device: &hidapi::DeviceInfo) -> usize {
    // EDBG are 512-bytes and don't respond until you give them 512 bytes.
    if device.vendor_id() == 0x03eb
        && device.product_id() != 0x2175
        && let Some(s) = device.product_string()
        && s.contains("EDBG")
    {
```

Practical note:

- `probe-rs` on this board is still somewhat flaky on macOS
- `--speed 1000` is currently the most reliable setting for `probe-rs run`
- default `probe-rs run` reset/hardfault vector catch caused unreliable
  attach/init on this board; `--no-catch-reset --no-catch-hardfault` is the
  current stable runner configuration

## USB Notes

The MCU USB CDC demo required the SAME5x board-helper clock pattern, not the
minimal `gclk0` path. The working setup uses:

- `GCLK2`
- source `DFLL`
- `clocks.usb(&usb_gclk)`
