---
name: zmk-build
description: Procedures for ZMK firmware compilation, UF2 flashing, and serial troubleshooting on macOS.
---

# ZMK Build & Debug Procedures
- **Compilation:**
  ```bash
  ZEPHYR_BASE=/Users/reed/dev/zmk-config-pnohty/zephyr \
  ZEPHYR_SDK_INSTALL_DIR=/Users/reed/zephyr-sdk-0.16.8 \
  CMAKE_PREFIX_PATH=/Users/reed/zephyr-sdk-0.16.8/cmake:/Users/reed/dev/zmk-config-pnohty/zephyr/share/zephyr-package/cmake \
  python3 -m west build -b rpi_pico -s zmk/app -- -DSHIELD=sweep_bling_left -DZMK_CONFIG="$(pwd)/config"
  ```
- **Flashing:**
  1. Boot controller into bootloader mode (RPI-RP2 drive appears).
  2. Copy `build/zephyr/zmk.uf2` to the drive.
- **Serial Debug:**
  - Use `tio /dev/tty.usbmodem*` to monitor output.
  - Watch for `[ps2_pio]` logs to verify PIO RX frame alignment and `0xF4` init success.
- **Clean Build:** If errors persist, delete the `build` directory and re-run the full `west build` command.
