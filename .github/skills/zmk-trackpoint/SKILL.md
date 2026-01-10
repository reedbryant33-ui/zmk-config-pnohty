---
name: zmk-trackpoint
description: Detailed hardware mapping and PIO PS/2 driver logic for the Sweeq MX (RP2040-Zero) and IBM Trackpoint.
---

# ZMK Trackpoint Reference
- **Controller:** Raspberry Pi RP2040-Zero.
- **Board Target:** `rpi_pico` (Selected to support PIO feature set).
- **Trackpoint:** IBM/Lenovo Blue Module (PS/2).
- **Pinout:** SDA/Data = GP2, SCL/Clock = GP3.
- **Driver Logic:**
    - **RX:** Custom PIO-based driver (`gpio-ps2-pio`) for jitter-free data capture.
    - **TX:** Bit-banged implementation for initialization commands (e.g., `0xF4` enable).
- **Keymap:** Ensure `CONFIG_ZMK_POINTING=y` and `&ps2` nodes are defined in `sweep_bling.overlay`.
