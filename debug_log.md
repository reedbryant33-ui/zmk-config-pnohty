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

### 2026-01-07 | Baseline Build Success & Trackpoint Feature Deferral
- **Commit**: 3706150 | **Result**: PASS (Build)
- **Objective**: Establish a working baseline build by deferring trackpoint features, then incrementally re-enable them.
- **Technical Changes**:
    - Added `zmk,physical-layout` and `zmk,kscan` chosen nodes to `sweep_bling_left.overlay`.
    - Defined `matrix_physical_layout` with required `display-name` and `transform` properties.
    - **Disabled** PIO UART, PS/2 device, and trackpoint listener definitions (commented out) to eliminate build errors.
    - All undefined Kconfig symbols remain commented.
- **Build Result**: SUCCESS
    - Firmware built: `zmk.uf2` (122880 bytes)
    - Memory usage: FLASH 2.91%, RAM 10.26%
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**:
    1. Flash the baseline firmware to test keyboard matrix functionality.
    2. Once baseline is confirmed working, re-enable PIO UART and PS/2 device definitions one at a time.
    3. Verify the `petejohanson,ps2-uart` binding exists in the ZMK devicetree bindings.
    4. If binding is missing, create custom binding file or use alternative PS/2 driver.

### 2026-01-07 | Configuration Cleanup & Build System Validation
- **Commit**: dc1e174 | **Result**: FAIL (Build - Undefined Symbols & Compilation)
- **Objective**: Clean up invalid Kconfig symbols and achieve a successful build.
- **Technical Changes**:
    - Removed undefined Kconfig symbols: `CONFIG_ZMK_POINTING_DEVICE`, `CONFIG_INPUT_MOUSE`, `CONFIG_INPUT_MOUSE_PS2_PIO`, `CONFIG_ZMK_INPUT_LISTENER`.
    - Re-enabled `CONFIG_ZMK_PHYSICAL_LAYOUTS=y` to fix `'layouts' undeclared` error in `physical_layouts.c`.
    - Environment setup: Activated `(zmk)` conda environment and exported `ZEPHYR_BASE`, `ZEPHYR_SDK_INSTALL_DIR`, `CMAKE_PREFIX_PATH`.
- **Build Errors**:
    - **First Attempt**: Kconfig warnings for undefined symbols → Build aborted.
    - **Second Attempt**: Compilation failed with `'layouts' undeclared` in `zmk/app/src/physical_layouts.c:120`.
    - **Third Attempt**: `ZMK_PHYSICAL_LAYOUTS` itself is undefined.
- **Lesson Learned**: Physical layouts require proper devicetree definition with `display-name`, `transform`, and chosen node reference, not just Kconfig.

### 2026-01-05 | PIO UART Configuration & Serial Debugging
- **Commit**: 00134371d135ec9feca498b4fbb16bf20b5a6049 | **Result**: FAIL (Build)

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