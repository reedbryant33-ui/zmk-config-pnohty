# ZMK Trackpoint Debug Log: Sweeq MX

## Hardware Source of Truth
- **SDA (Data)**: GP2
- **SCL (Clock)**: GP3
- **VCC**: 3.3V (Note: Ensure the module isn't 5V-only)
- **GND**: Common ground established

## Current Status Summary
- **Matrix**: Functional (Keys working).
- **Build System**: GitHub Actions (Remote).
- **Board Target**: `rpi_pico` (Required for Zephyr 3.5.0 compatibility).
- **Active Driver**: Pete Johanson's PIO UART.
- **Issue**: Build passes, but no movement. WebSerial debug logging is currently enabled.

---

# ZMK Trackpoint Debug Log: Sweeq MX

## Current Status Summary
- **Matrix**: Functional (Keys working).
- **Build System**: GitHub Actions (Remote).
- **Board Target**: `rpi_pico` (Zephyr 3.5.0 compatibility).
- **Active Driver**: Pete Johanson's PIO UART (Built-in to pointer branch).
- **Issue**: Firmware builds/flashes, but no trackpoint movement/detection.

---

## Active Debugging Entries (Newest First)

### 2026-01-05 | PIO UART Configuration & Serial Debugging
- **Commit**: [Insert Last Hash] | **Result**: PASS (Build) / FAIL (Hardware Init)
- **Objective**: Establish hardware-timed communication and enable logging.
- **Technical Changes**:
    - Defined PIO UART nodes in overlay for **GP2 (Data)** and **GP3 (Clock)**.
    - Enabled `CONFIG_ZMK_INPUT_LISTENER=y`.
    - Disabled Physical Layouts to resolve `'layouts' undeclared` compiler error.
    - Re-enabled USB CDC ACM nodes for Serial Console access.
- **Lesson Learned**: The `zmk,pointing_device` chosen node must point to the `input_listener`, not the raw hardware device.

---

## Backlog of Previous Attempts

### 1. Matrix & Pin Discovery
- **Result**: SUCCESS.
- **Outcome**: Verified physical row/column mapping. The Sweeq MX matrix is now stable.

### 2. Community Driver Rotation (`infused-kim`, `badjeff`, etc.)
- **Outcome**: Switched to **Pete Johanson's pointers branch** as the "God Tier" solution for RP2040. It provides the most native integration for the ZMK input subsystem.

### 3. GPIO Bit-Banging (`ps2_gpio.c`)
- **Outcome**: ABANDONED. 
- **Lesson Learned**: Timing jitter from ZMK interrupts caused packet misalignment and "phantom clicks." Hardware-timed PIO is required for "Laptop Smooth" movement.

### 4. Early PIO Integration
- **Outcome**: SUCCESS (Build Path).
- **Lesson Learned**: Zephyr 3.5.0 (Pete's branch) requires specific `pinctrl` syntax for PIO that differs from mainline Zephyr. Manual pinmuxing in the overlay is more reliable.