# ZMK Trackpoint Debug Log: Sweeq MX

## Hardware Source of Truth
- **SDA (Data)**: GP2
- **SCL (Clock)**: GP3
- **VCC**: 3.3V (Note: Ensure the module isn't 5V-only)
- **GND**: Common ground established

## Current Status Summary
- **Matrix**: Functional (Keys working).
- **Build System**: GitHub Actions (Remote).
- **Board Target**: `rpi_pico` (Zephyr 3.5.0 compatibility).
- **Active Driver**: Pete Johanson's PIO UART (Built-in to pointer branch).
- **Issue**: Firmware builds/flashes, but no trackpoint movement/detection.

---

## Active Debugging Entries (Newest First)

### 2026-01-07 | Configuration Cleanup & Build System Validation
- **Commit**: c1d7c42 | **Result**: FAIL (Build - Undefined Symbols & Compilation)
- **Objective**: Clean up invalid Kconfig symbols and achieve a successful build.
- **Technical Changes**:
    - Removed undefined Kconfig symbols: `CONFIG_ZMK_POINTING_DEVICE`, `CONFIG_INPUT_MOUSE`, `CONFIG_INPUT_MOUSE_PS2_PIO`, `CONFIG_ZMK_INPUT_LISTENER`.
    - Re-enabled `CONFIG_ZMK_PHYSICAL_LAYOUTS=y` to fix `'layouts' undeclared` error in `physical_layouts.c`.
    - Environment setup: Activated `(zmk)` conda environment and exported `ZEPHYR_BASE`, `ZEPHYR_SDK_INSTALL_DIR`, `CMAKE_PREFIX_PATH`.
- **Build Errors**:
    - **First Attempt**: Kconfig warnings for undefined symbols → Build aborted.
    - **Second Attempt**: Compilation failed with `'layouts' undeclared` in `zmk/app/src/physical_layouts.c:120`.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION.
- **Next Steps**: 
    1. Verify that `CONFIG_ZMK_PHYSICAL_LAYOUTS=y` resolves the layouts compilation error.
    2. Check if additional keymap/shield configuration is required to populate the layouts array.
    3. Consider building without pointing device features initially to establish a baseline, then incrementally add features.

### 2026-01-05 | PIO UART Configuration & Serial Debugging
- **Commit**: 00134371d135ec9feca498b4fbb16bf20b5a6049 | **Result**: FAIL (Build)
- **Objective**: First build attempt following new instructions.
- **Technical Changes**:
    - Added new instruction file.
    - Staged and committed specified files.
- **Lesson Learned**: Build command requires `-s` flag to specify source directory and `_left` or `_right` suffix for the shield. The build is failing due to a Kconfig warning related to `ZMK_PHYSICAL_LAYOUTS`.

### 2026-01-05 | PIO UART Configuration & Serial Debugging
- **Commit**: d811f2a | **Result**: FAIL (Build)
- **Objective**: Fix Kconfig warnings and enable pointing device.
- **Technical Changes**:
    - Enabled pointing device options in `sweep_bling.conf`.
    - Commented out `CONFIG_ZMK_PHYSICAL_LAYOUTS=n`.
- **Lesson Learned**: The build is still failing due to undefined Kconfig symbols related to the pointing device. This suggests a problem with how the Kconfig files are being sourced or defined.

### 2026-01-05 | PIO UART Configuration & Serial Debugging
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