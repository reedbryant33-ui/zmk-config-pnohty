# ZMK Trackpoint Debug Log: Sweeq MX

## Hardware Source of Truth
- **SDA (Data)**: GP2
- **SCL (Clock)**: GP3
- **VCC**: 3.3V (Note: Ensure the module isn't 5V-only)
- **GND**: Common ground established

## Current Status Summary
- **Matrix**: Functional (Keys working - VERIFIED).
- **Build System**: Local (Fixed SDK path, infused-kim driver fetched).
- **Board Target**: `rpi_pico` (Zephyr 3.5.0 compatibility).
- **Active Driver**: Infused-Kim PS/2 (PIO-based, hardware-timed on RP2040).
- **Build Status**: ✅ SUCCESS - Ready for trackpoint hardware testing.

---

## Active Debugging Entries (Newest First)

### 2026-01-08 | Infused-Kim PS/2 Driver Integration - BUILD SUCCESS
- **Commit**: TBD | **Result**: ✅ PASS (Build Complete)
- **Objective**: Integrate infused-kim PIO-based PS/2 driver and build firmware with trackpoint support.
- **Technical Changes**:
    - Added infused-kim to `config/west.yml`: `kb_zmk_ps2_mouse_trackpoint_driver` module at `modules/drivers/ps2`
    - Updated overlay: Changed from `petejohanson,ps2-uart` to `zmk,ps2` compatible (infused-kim device)
    - Pin configuration: SCL on GP3, SDA on GP2 (with GPIO_PULL_UP flags for PS/2 open-drain)
    - Kconfig: Removed undefined symbols (`ZMK_POINTING`, `ZMK_INPUT_MOUSE_PS2`) - Pete's branch doesn't define these yet
    - Device driver: infused-kim uses PIO on RP2040 for hardware-timed clock/data - eliminates jitter from GPIO bit-banging
- **Build Result**: ✅ SUCCESS
    - Firmware built: `zmk.uf2` (122880 bytes)
    - Memory usage: FLASH 2.91%, RAM 10.26%
    - Infused-kim PS/2 driver compiled in
    - Devicetree properly resolves `zmk,ps2` compatible device
    - All deprecation warnings are expected (unrelated to trackpoint)
- **Key Advantages of This Approach**:
    - ✅ PIO-based timing (hardware-timed) = minimal jitter vs GPIO bit-banging
    - ✅ Infused-kim is standard ZMK community driver, well-tested
    - ✅ Works with Pete's input listener infrastructure
    - ✅ No custom driver implementation needed
- **Hardware Status**: READY FOR TESTING
- **Next Steps**:
    1. Flash firmware to RP2040-Zero (using FLASH_INSTRUCTIONS.md)
    2. Monitor serial output for PS/2 device initialization
    3. Test trackpoint movement and verify cursor motion
    4. If jitter still occurs, tune interrupt priorities and PS/2 timing parameters

### 2026-01-08 | Analysis - PS/2 Driver Strategy Decision
- **Commit**: 37990c2 | **Result**: PARTIAL (Keyboard Matrix: PASS, Trackpoint: FAIL)
- **Objective**: Flash firmware to RP2040-Zero and test trackpoint functionality.
- **Test Environment**:
    - Firmware: `build/zephyr/zmk.uf2` (122880 bytes) - Built 2026-01-08
    - Serial Monitor: WebSerial (https://webserial.io/) connected to cu.usbmodem14301
    - Test Date: 2026-01-08
- **Test Results**:
    - **USB Connectivity**: ✅ PASS
      - Device enumerated correctly as OpenMoko HID device (1d50:615e)
      - USB resets, configures, reaches state 3 (configured)
      - Get_selected_transport: "Only USB is ready"
    - **Keyboard Matrix**: ✅ PASS
      - Key presses detected and processed
      - Position 2 (keycode 0x70009) tested and working
      - Position 0 (keycode 0x70014) tested and working
      - HID reports sent correctly: "zmk_endpoints_send_report: usage page 0x07"
    - **Trackpoint**: ❌ FAIL - **DRIVER NOT LOADED**
      - **NO PS/2 device initialization messages in logs**
      - **NO input_listener activity**
      - **NO trackpoint movement detected**
      - Serial log completely silent on trackpoint (only keyboard matrix events visible)
- **Root Cause Identified**: PS/2 UART driver implementation is **MISSING** from Pete Johanson's feat/pointers-move-scroll branch
  - Devicetree binding file exists: `config/dts-bindings/petejohanson,ps2-uart.yaml`
  - Device is defined in overlay: `sweep_bling_left.overlay` includes ps2_device and input_listener nodes
  - **BUT**: No actual driver code exists to handle the `petejohanson,ps2-uart` compatible string
  - Zephyr cannot instantiate the device without matching driver code
  - Pete's branch has input_listener infrastructure but NO PS/2 driver implementation
  
- **Historical Context**: Earlier commits used badjeff's `gpio-ps2` driver (works with mainline ZMK)
  - Git history shows prior attempts: infused-kim, badjeff drivers were tested
  - badjeff driver uses compatible `"gpio-ps2"` and scl-gpios/sda-gpios (GPIO bit-banging, not PIO)
  - These drivers work on mainline but lose Pete's input infrastructure benefits
  
- **Decision Needed**: Choose driver strategy
  1. **Option A**: Switch to mainline ZMK + badjeff PS/2 driver (stable, proven, slower GPIO bit-bang)
  2. **Option B**: Implement minimal PIO UART PS/2 driver for Pete's branch (complex, fast hardware-timed)
  3. **Option C**: Use gpio-ps2 compatible on Pete's branch (if compatible, hybrid approach)
- **Hardware Status**: VERIFIED (Wiring correct - keyboard works, USB works)
- **Next Steps**:
    1. Search Pete Johanson's branch for PS/2 driver implementation
    2. Check if driver is in a module or needs to be implemented separately
    3. Alternative: Research community PS/2 drivers (infused-kim, badjeff) compatibility with this branch
    4. Possibly implement minimal PIO UART PS/2 driver based on Pete's examples

### 2026-01-08 | Build Environment Fix - Environment Variable Inheritance
- **Commit**: TBD | **Result**: PASS (Build)
- **Objective**: Fix build system environment variables to properly pass to cmake subprocess and achieve clean build.
- **Technical Changes**:
    - Identified root cause: Environment variables set in shell were not being inherited by west subprocess.
    - Solution: Use `python3 -m west` with environment variables set directly in the command: `ZEPHYR_BASE=/... CMAKE_PREFIX_PATH=/... python3 -m west build ...`
    - Correct SDK path: `/Users/reed/zephyr-sdk-0.16.8` (not `/opt/zephyr-sdk-0.16.8`)
    - Correct CMAKE_PREFIX_PATH: `/Users/reed/zephyr-sdk-0.16.8/cmake:/Users/reed/dev/zmk-config-pnohty/zephyr/share/zephyr-package/cmake`
- **Build Result**: SUCCESS
    - Clean build completed: `zmk.uf2` (122880 bytes)
    - Memory usage: FLASH 2.91%, RAM 10.26%
    - Devicetree binding resolved (only deprecation warning for 'label', expected)
    - No build errors
- **Hardware Status**: PENDING PHYSICAL VERIFICATION (Ready for flashing)
- **Next Steps**:
    1. Flash firmware to RP2040-Zero
    2. Test trackpoint functionality with serial monitor
    3. Verify PS/2 device initialization in boot logs
    4. Test cursor movement

### 2026-01-07 | PS/2 UART Binding File Creation & Clean Build
- **Commit**: 06e621c | **Result**: PASS (Build)
- **Objective**: Create devicetree binding for petejohanson,ps2-uart and establish clean build path.
- **Technical Changes**:
    - Created `config/dts-bindings/petejohanson,ps2-uart.yaml` binding file (was missing)
    - Binding defines property schema: `gpios` (phandle-array for SDA/SCL pins)
    - Added `config/CMakeLists.txt` to register custom bindings directory with ZMK build system
    - Attempted to include ps2-uart-driver module from GitHub (repository doesn't exist, left commented)
- **Build Result**: SUCCESS
    - Clean build completed: `zmk.uf2` (122880 bytes)
    - Memory usage: FLASH 2.91%, RAM 10.26%
    - Devicetree binding resolved (no vendor prefix errors now, only deprecation warning for 'label')
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**:
    1. Verify physical trackpoint wiring continuity
    2. Identify actual PS/2 driver implementation location (not found in current ZMK branch)
    3. Check if driver binary needs to be built separately or if it's missing entirely

### 2026-01-07 | First Flash Test - Trackpoint Detection Failure
- **Commit**: aff0da5 | **Result**: PARTIAL (Keyboard Matrix: PASS, Trackpoint: FAIL)
- **Objective**: Flash rebuilt firmware and test trackpoint functionality with PIO UART + PS/2 device enabled in devicetree.
- **Test Environment**:
    - Firmware: `build/zephyr/zmk.uf2` (122880 bytes)
    - Serial Monitor: WebSerial (VID:PID 1d50:615e)
    - Test Date: 2026-01-07
- **Test Results**:
    - **USB Connectivity**: ✅ PASS
      - Device enumerated correctly as OpenMoko HID device
      - USB configured successfully
    - **Keyboard Matrix**: ✅ PASS
      - Key presses detected and processed (positions 20, 10, 0 tested)
      - HID reports sent correctly
      - Example: "kscan_matrix_read: Sending event at 4,2 state on/off"
    - **Trackpoint**: ❌ FAIL
      - **No PS/2 device initialization in logs**
      - **No input_listener activity**
      - **No trackpoint movement detected**
- **Log Analysis**:
    - Boot logs show: `kscan_matrix_init_input_inst: ready`
    - USB enumeration: Device reset → configured
    - Only keyboard matrix events present, no PS/2 bus activity
    - **Root Cause Identified**: PS/2 UART driver module is commented out in `config/west.yml`
- **Hardware Status**: PENDING (Trackpoint driver not loaded)
- **Next Steps**:
    1. Uncomment `zmk-ps2-uart-driver` in `config/west.yml`
    2. Run `west update` to fetch the driver module
    3. Rebuild firmware
    4. Retest with driver loaded

### 2026-01-07 | Build Fix - Kconfig & Devicetree Conflicts
- **Commit**: aff0da5 | **Result**: PASS (Build)
- **Objective**: Fix build system and re-enable trackpoint PIO UART + PS/2 device definitions.
- **Technical Changes**:
    - Removed undefined Kconfig symbols: `CONFIG_ZMK_POINTING_DEVICE` and `CONFIG_INPUT_MOUSE` from `sweep_bling.conf` (these symbols don't exist in this ZMK branch).
    - Fixed devicetree conflict: Removed `zmk,matrix-transform` from chosen node in overlay—physical layouts and matrix-transform are mutually exclusive.
    - Corrected Zephyr SDK path to `/Users/reed/zephyr-sdk-0.16.8` (local system, not /opt).
- **Build Result**: SUCCESS
    - Firmware built: `zmk.uf2` (122880 bytes)
    - Memory usage: FLASH 2.91%, RAM 10.26%
    - Firmware location: `build/zephyr/zmk.uf2`
    - **Note**: Warning remains about unknown vendor prefix 'petejohanson' (expected—Pete Johanson's driver uses custom binding)
- **Hardware Status**: PENDING PHYSICAL VERIFICATION

### 2026-01-07 | Hardware Physical Verification - Baseline Test
- **Commit**: 3706150 | **Result**: PARTIAL (Keyboard Matrix: PASS, Trackpoint: FAIL)
- **Objective**: Flash baseline firmware and test keyboard matrix and trackpoint functionality.
- **Test Environment**: 
    - Firmware: `build/left/zephyr/zmk.uf2` (122880 bytes)
    - Serial Monitor: WebSerial (VID:PID 1d50:615e)
    - Test Date**: 2026-01-07
- **Test Results**:
    - **USB Connectivity**: ✅ PASS
      - Device enumerated correctly as OpenMoko HID device
      - USB configuration completed (Device configured message)
    - **Keyboard Matrix**: ✅ PASS
      - All keys register correctly through matrix scanner
      - Key events properly processed: row/col → position → keycode → HID report
      - Multiple simultaneous key presses tested (A, D, G, etc.) → all send correctly
      - Layers functional (momentary_layer working, layer state changes detected)
      - HID reports transmitted successfully
    - **Trackpoint**: ❌ FAIL
      - **No input events detected on GP2 (SDA) or GP3 (SCL)**
      - PIO UART driver disabled in this build (intentionally deferred)
      - No PS/2 or input_listener activity in logs
- **Serial Log Analysis**:
  - Boot sequence: kscan_matrix_init → USB enumeration → ready
  - Keyboard matrix debug output confirms all keys working
  - No errors or warnings in firmware operation
  - Log snippet: Multiple key press/release cycles show proper HID flow
- **Hardware Status**: 
  - **Keyboard Matrix**: VERIFIED WORKING
  - **Trackpoint**: UNVERIFIED (PIO UART disabled)
- **Next Steps**:
    1. Verify physical trackpoint wiring (continuity check on GP2, GP3, GND, VCC)
- **Commit**: 3706150 | **Result**: PARTIAL (Keyboard Matrix: PASS, Trackpoint: FAIL)
- **Objective**: Flash baseline firmware and test keyboard matrix and trackpoint functionality.
- **Test Environment**: 
    - Firmware: `build/left/zephyr/zmk.uf2` (122880 bytes)
    - Serial Monitor: WebSerial (VID:PID 1d50:615e)
    - Test Date**: 2026-01-07
- **Test Results**:
    - **USB Connectivity**: ✅ PASS
      - Device enumerated correctly as OpenMoko HID device
      - USB configuration completed (Device configured message)
    - **Keyboard Matrix**: ✅ PASS
      - All keys register correctly through matrix scanner
      - Key events properly processed: row/col → position → keycode → HID report
      - Multiple simultaneous key presses tested (A, D, G, etc.) → all send correctly
      - Layers functional (momentary_layer working, layer state changes detected)
      - HID reports transmitted successfully
    - **Trackpoint**: ❌ FAIL
      - **No input events detected on GP2 (SDA) or GP3 (SCL)**
      - PIO UART driver disabled in this build (intentionally deferred)
      - No PS/2 or input_listener activity in logs
- **Serial Log Analysis**:
  - Boot sequence: kscan_matrix_init → USB enumeration → ready
  - Keyboard matrix debug output confirms all keys working
  - No errors or warnings in firmware operation
  - Log snippet: Multiple key press/release cycles show proper HID flow
- **Hardware Status**: 
  - **Keyboard Matrix**: VERIFIED WORKING
  - **Trackpoint**: UNVERIFIED (PIO UART disabled)
- **Next Steps**:
    1. Verify physical trackpoint wiring (continuity check on GP2, GP3, GND, VCC)
    2. Test trackpoint module separately with logic analyzer if available
    3. Re-enable PIO UART and PS/2 device definitions in overlay
    4. Create or locate proper devicetree binding for `petejohanson,ps2-uart`
    5. Rebuild and retest with trackpoint features enabled

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
    - Firmware location: `build/left/zephyr/zmk.uf2`
- **Hardware Status**: PENDING PHYSICAL VERIFICATION

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