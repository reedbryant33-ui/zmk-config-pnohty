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
- **Active Driver**: Custom PIO-Based PS/2 Driver (New implementation).
- **Build Status**: ✅ SUCCESS - Ready for trackpoint testing.
- **Firmware**: `zmk.uf2` (122,880 bytes) ready for flashing.

---

## Active Debugging Entries (Newest First)

### 2026-01-10 | Workspace Migration & Skills Setup
- **Commit**: Pending | **Result**: ✅ SUCCESS
- **Objective**: Transition workspace to "Skill-Enabled" architecture. Create `.roo/skills` and `.github/skills` for Trackpoint and Build procedures. Update `.clinerules` with strict agent protocols.
- **Observations**:
    - Creating skill files for `zmk-trackpoint` and `zmk-build`.
    - Mirroring skills to `.github` for cross-agent compatibility.
    - Updating `.clinerules` to enforce "No Filler" and strict logging.
    - **Verification**: 
        - `.clinerules` updated with new protocols.
        - `.roo/skills/zmk-trackpoint/SKILL.md` exists.
        - `.roo/skills/zmk-build/SKILL.md` exists.
        - `.github/skills/zmk-trackpoint/SKILL.md` exists.
        - `.github/skills/zmk-build/SKILL.md` exists.

### 2026-01-09 | PIO-Based PS/2 Driver - Physical Test 4
- **Commit**: `ea6f056` | **Result**: 🟡 PROGRESS (Data Captured)
- **Objective**: Verify if bit-banged TX implementation triggers trackpoint responses.
- **Observations**:
    - **Logs**:
        ```
        PS2 PIO Driver Init: Starting initialization for TRACKPOINT...
        ...
        [00:00:05.606,000] ps2_pio: ps2_pio_work_handler: Processing byte: 0x00
        [00:00:05.607,000] <wrn> zmk_input_mouse_ps2: PS2 Sync Error: Byte 1 bit 3 not set (00)
        ```
    - **Keyboard Matrix**: ✅ Functional.
    - **Trackpoint**: 🟡 Partially Responsive. The driver is now receiving a stream of data bytes (`0x00`) from the hardware!
    - **Conclusion**: This is a major breakthrough. The bit-banged `ps2_write` (TX) is successfully enabling the trackpoint. The PIO state machine is successfully capturing data frames and triggering the ISR. The "Sync Error" and `0x00` values suggest we need to tune the PIO RX frame alignment or clock sampling, as it's likely shifting in bits incorrectly or sampling a floating line.
- **Next Steps**: 
    1. Tune PIO RX program to ensure it correctly aligns the 11-bit PS/2 frame.
    2. Verify the `0x00` stream—check if pins are floating or if the trackpoint is in a mode we don't expect.

### 2026-01-09 | PIO-Based PS/2 Driver - Physical Test 3
- **Commit**: Pending | **Result**: ❌ FAIL (Communication Fail)
- **Objective**: Verify if driver initialization triggers the enable command.
- **Observations**:
    - **Logs**:
        ```
        PS2 PIO Driver Init: Starting initialization for TRACKPOINT...
        [00:00:00.001,000] ps2_pio: PIO RX configured on SM 0
        PS2 PIO Driver Init: Success
        ...
        [00:00:00.002,000] ps2_pio: TX not yet implemented
        [00:00:00.002,000] zmk_input_mouse_ps2: Failed to enable data reporting (F4) - Write might not be supported
        *** Booting Zephyr OS build dacab4875df7 ***
        ```
    - **Keyboard Matrix**: ✅ Functional.
    - **Trackpoint**: ❌ Unresponsive.
    - **Conclusion**: The driver is active, but the trackpoint is "sleeping." It requires an `0xF4` (Enable Data Reporting) command to start sending motion data. Because the `ps2_pio` driver lacked `ps2_write` (TX) implementation at the time of this test, the initialization failed to wake it up.
- **Next Steps**: Implement bit-banged TX in `ps2_pio.c` (Done) and re-test.

### 2026-01-09 | PS/2 Driver Build Fixes & Success
- **Commit**: [TBD] | **Result**: ✅ BUILD SUCCESS
- **Objective**: Resolve compilation errors and ensure all PS/2 driver components are correctly included in the build.
- **Observations/Fixes**:
    - **API Typo**: Fixed a compilation error in `zmk_input_mouse_ps2.c` where `ps2_configure` was called instead of the standard Zephyr `ps2_config` function.
    - **Devicetree Binding Resolution**: 
        - Discovered that the `gpio-ps2-pio` compatible string was failing to resolve because the binding file was named `ps2_pio.yaml` and was not in a standard search path.
        - Renamed the binding to `gpio-ps2-pio.yaml` to match the compatible string.
        - Created `config/dts/bindings/` and copied the PS/2 bindings there to ensure ZMK's build system finds them reliably.
    - **Module Configuration**: Updated `modules/drivers/ps2/zephyr/module.yml` with the correct `dts_root` setting (`..`) to help Zephyr find the local bindings.
    - **Build Status**: The build now completes successfully with `CONFIG_PS2_PIO=y` and `CONFIG_ZMK_INPUT_MOUSE_PS2=y`.
- **Next Steps**:
    1. Flash `zmk.uf2` to the left half (RP2040-Zero).
    2. Monitor serial logs to confirm driver initialization (`ps2_pio_init`).
    3. Verify trackpoint movement and button clicks.

### 2026-01-09 | PIO-Based PS/2 Driver Debugging - Init Verification
- **Commit**: Pending | **Result**: ❌ FAIL (Trackpoint Unresponsive)
- **Objective**: Verify driver initialization by adding direct `printk` output and ensuring correct init priority.
- **Observations**:
    - **Logs**:
        ```
        PS2 PIO Driver Init: Starting initialization for TRACKPOINT...
        [00:00:00.001,000] ps2_pio: PIO RX configured on SM 0
        PS2 PIO Driver Init: Success
        ...
        [00:00:00.002,000] ps2_pio: TX not yet implemented
        [00:00:00.002,000] zmk_input_mouse_ps2: Failed to enable data reporting (F4) - Write might not be supported
        *** Booting Zephyr OS build dacab4875df7 ***
        ```
    - **Keyboard Matrix**: ✅ Functional.
    - **Trackpoint**: ❌ Unresponsive.
    - **Conclusion**: The driver is now successfully initializing (`PS2 PIO Driver Init: Success`), but it fails to enable data reporting because `ps2_pio: TX not yet implemented`. Most PS/2 trackpoints require a `0xF4` command to start sending data.
- **Next Steps**: Implement basic TX support in `ps2_pio` driver to allow sending the enable command.

### 2026-01-09 | PIO-Based PS/2 Driver Refinement - Physical Test 2
- **Commit**: Pending | **Result**: ❌ FAIL (Trackpoint Unresponsive)
- **Objective**: Verify functionality of the refined PIO-based PS/2 driver on actual hardware.
- **Observations**:
    - **Keyboard Matrix**: ✅ Functional (Keys 'd', 'x', 'r', 'c', 't', 'a', 'w', 'f' detected).
    - **Trackpoint**: ❌ Unresponsive. No cursor movement.
    - **Logs**: **CRITICAL MISSING INFO**. 
        - Still NO logs from `ps2_pio` driver (e.g., "PIO RX configured on SM 0").
        - The logs show standard ZMK initialization (`kscan_matrix_init`, `zmk_usb_get_conn_state`, etc.) but completely skip the `gpio-ps2-pio` device initialization.
    - **Root Cause Analysis**:
        - Since `ps2_pio.c` uses `LOG_MODULE_REGISTER`, and `CONFIG_PS2_LOG_LEVEL` is set, we *should* see logs if `ps2_pio_init` is called.
        - Absence of logs strongly suggests `ps2_pio_init` is **NEVER CALLED**.
        - Why?
            1. **Device Tree Status**: Is the node enabled? `status = "okay"` is in the overlay.
            2. **Driver Compatibility**: `DT_DRV_COMPAT gpio_ps2_pio` matches `compatible = "gpio-ps2-pio"`.
            3. **Initialization Priority**: `POST_KERNEL` with `CONFIG_PS2_INIT_PRIORITY`.
            4. **Kconfig**: `CONFIG_PS2_PIO` might not be enabled in the final build.
            5. **Device Binding**: If the device is not referenced by any active driver/application logic, Zephyr might optimize it out? 
                - `trackpoint` node references `&trackpoint_device`.
                - `trackpoint` is in `zmk,pointing`.
                - So it should be active.
- **Hypothesis**: `CONFIG_PS2` or `CONFIG_PS2_PIO` is not actually set to 'y' in the final `.config`.

### 2026-01-08 | PIO-Based PS/2 Driver - Physical Test 1
- **Commit**: Pending | **Result**: ❌ FAIL (Trackpoint Unresponsive)
- **Objective**: Verify functionality of the new PIO-based PS/2 driver on actual hardware.
- **Observations**:
    - **Keyboard Matrix**: ✅ Functional. Serial logs show successful key detection for "c", "s", "f" (Row 2, Cols 2, 1, 0).
    - **Trackpoint**: ❌ Unresponsive. No cursor movement.
    - **Logs**: **CRITICAL MISSING INFO**. The serial logs show *no* output from the `ps2_pio` driver.
        - Expected: `[00:00:00.xxx,xxx] <inf> ps2_pio: PIO RX configured on SM 0...`
        - Actual: No mention of `ps2_pio` or the device initialization.
    - **Conclusion**: The driver code is likely not being initialized. This could be due to:
        1. Kconfig symbol `CONFIG_PS2_PIO` not being set to 'y' in the final build.
        2. Device Tree compatibility string mismatch (though they look correct).
        3. Driver initialization priority/level issue.
        4. Logs being filtered out (though `<inf>` should show up by default).

### 2026-01-10 | PIO-Based PS/2 Driver - Phase 1 & 2 Execution
- **Commit**: Pending | **Result**: ✅ BUILD SUCCESS
- **Objective**: Execute Phase 1 (PIO Program Fix) and Phase 2 (Driver Logging & Verification) of `PIO_FIX_GAMEPLAN.md` and build the firmware.
- **Observations**:
    - PIO program `ps2_pio_rx.pio` updated to sample on clock low and push after stop bit.
    - `LOG_INF("PIO RX RAW: 0x%08x", rx_raw);` added to `ps2_pio_isr` in `ps2_pio.c`.
    - The build completed successfully, generating `zmk.uf2`.
    - Warnings observed: `unused variable 'data'` in `ps2_pio_enable_callback` and `ps2_pio_disable_callback`, deprecated `label` in DTS bindings, and `No SOURCES given to Zephyr library: drivers__ps2`. These are non-critical warnings.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: Flash the `zmk.uf2` firmware and monitor serial output for `PIO RX RAW` logs.

### 2026-01-10 | PIO-Based PS/2 Driver - Physical Test 5
- **Commit**: Pending | **Result**: ❌ FAIL (Incorrect Start Bit, Data 0x00)
- **Objective**: Verify PIO program and C-level data extraction with detailed logging after Phase 1 & 2.
- **Observations**:
    - `ps2_pio_write: Sending 0xf4` confirms TX functionality.
    - `PIO RX RAW` values (e.g., `0xfe800000`, `0xca000000`, `0xffc00000`) are consistently observed.
    - **Critical Issue**: The most significant bit of `rx_raw` (interpreted as the Start bit due to `sm_config_set_in_shift(..., true, ...)` for MSB-first shifting) is **always 1**. PS/2 protocol dictates the Start bit must be 0.
    - The `ps2_pio_work_handler: Processing byte: 0x00` and `PS2 Sync Error: Byte 1 bit 3 not set (00)` indicate that the extracted data byte is `0x00` and the high-level PS/2 input processor is not receiving valid data.
    - **Conclusion**: The PIO state machine is receiving data, but the frames are fundamentally malformed at the hardware level (Start bit is 1). This points to a synchronization problem within the PIO program itself, or an unexpected signal on the data line, rather than just an incorrect C-level parsing of a valid raw frame.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: 
    1. Correct C-level data extraction in `ps2_pio.c` to properly decode MSB-first shifted data and add more detailed logging (Start, Parity, Stop bits).
    2. Rebuild and re-flash to verify the extracted bits. This will help confirm if the PIO is *actually* reading a '1' for the start bit, or if our interpretation of `rx_raw` combined with `sm_config_set_in_shift` is still flawed. If Start is still '1', then the PIO program or physical wiring needs a deeper look.

### 2026-01-10 | PIO-Based PS/2 Driver - Build after Data Extraction & Logging Fix
- **Commit**: Pending | **Result**: ✅ BUILD SUCCESS
- **Objective**: Rebuild firmware after correcting C-level data extraction and adding detailed bit-level logging in `ps2_pio.c`.
- **Observations**:
    - The build completed successfully, generating `zmk.uf2`.
    - The same non-critical warnings regarding `unused variable 'data'`, deprecated `label` in DTS bindings, and `No SOURCES given to Zephyr library: drivers__ps2` were observed. These are expected and do not prevent the build.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: Flash the `zmk.uf2` firmware and monitor serial output. Pay close attention to the new `PIO RX Decoded: Start=0x%x, Data=0x%02x, Parity=0x%x, Stop=0x%x` logs to precisely identify the values of each bit in the received frames.

### 2026-01-10 | PIO-Based PS/2 Driver - Physical Test 6
- **Commit**: Pending | **Result**: ❌ FAIL (Mixed Start/Stop Bits, Data 0x00)
- **Objective**: Verify PIO program and C-level data extraction with detailed logging after LSB-first interpretation.
- **Observations**:
    - `ps2_pio_write: Sending 0xf4` continues to confirm TX functionality.
    - `PIO RX RAW` values are still observed.
    - `PIO RX Decoded` now shows some frames with `Start=0x0` (correct) but many still with `Start=0x1` (incorrect).
    - Even when `Start=0x0`, a frequent warning `PS/2 Protocol Warning: Stop bit is 0 (expected 1)` is observed.
    - The `ps2_pio_work_handler: Processing byte: 0x00` and `PS2 Sync Error: Byte 1 bit 3 not set (00)` persist.
    - **Conclusion**: The previous C-level data extraction was indeed incorrect for LSB-first shifting. While the detailed logging now better reflects the PIO's raw output, the fundamental issue of malformed Start and Stop bits from the PIO persists. This strongly indicates a timing or sampling problem within the PIO program (`ps2_pio_rx.pio`) itself. The PIO is not consistently synchronizing to the PS/2 data frames correctly.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: Re-evaluate and potentially modify the `ps2_pio_rx.pio` assembly code. The inconsistent Start and Stop bits suggest the PIO is not capturing the bits at the correct points in the PS/2 clock cycle. Introducing a small delay (NOP) for sampling or adjusting `wait` conditions might be necessary to stabilize bit capture. Also, confirm the clock and data pin definitions in the PIO program against the hardware configuration.

### 2026-01-10 | PIO-Based PS/2 Driver - Build after NOP insertion
- **Commit**: Pending | **Result**: ✅ BUILD SUCCESS
- **Objective**: Rebuild firmware after introducing NOP delays in `ps2_pio_rx.pio` for improved bit sampling stability.
- **Observations**:
    - The build completed successfully, generating `zmk.uf2`.
    - The same non-critical warnings regarding `unused variable 'data'`, deprecated `label` in DTS bindings, and `No SOURCES given to Zephyr library: drivers__ps2` were observed. These are expected and do not prevent the build.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: Flash the `zmk.uf2` firmware and monitor serial output. Observe the `PIO RX Decoded` logs to see if the Start and Stop bit consistency has improved with the `nop` delays.

### 2026-01-10 | PIO-Based PS/2 Driver - Physical Test 7
- **Commit**: Pending | **Result**: PENDING PHYSICAL VERIFICATION
- **Objective**: Verify PIO program with corrected 11-bit sampling loop and fixed CMake source inclusion.
- **Observations**:
    - Confirmed `ps2_pio.c.obj` is finally being generated and linked.
    - PIO program updated to a cleaner loop: `wait 0 gpio 3` -> `in pins, 1` -> `wait 1` -> `wait 0` (repeat).
    - Added `in null, 21` to align 11 bits to the LSBs of the 32-bit FIFO word.
    - Corrected C-level decoding to match the new bit alignment.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: User to flash `zmk.uf2` and provide serial logs to confirm if `Start=0` and `Stop=1` are consistently captured.

### 2026-01-10 | PIO-Based PS/2 Driver - Physical Test 8
- **Commit**: Pending | **Result**: PENDING PHYSICAL VERIFICATION
- **Objective**: Align PIO capture and C-level decoding based on RAW capture analysis.
- **Observations**:
    - Analyzed `rx_raw` logs (e.g., `0xfe800000`) and confirmed they contain valid PS/2 frames shifted to the MSB bits (31:21).
    - Simplified `ps2_pio_rx.pio` to a standard loop with `autopush` at 11 bits.
    - Updated `ps2_pio.c` to shift `rx_raw >>= 21` before decoding, ensuring `Start=0` and `Stop=1` are aligned to bits 0 and 10.
    - Cleaned up `CMakeLists.txt` to ensure consistent driver inclusion.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: User to flash and verify if "Sync Error" is resolved. Valid frames are expected in the logs.

### 2026-01-10 | PIO-Based PS/2 Driver - Physical Test 9
- **Commit**: Pending | **Result**: PENDING PHYSICAL VERIFICATION
- **Objective**: Fix "Sync Errors" caused by ghost zero frames and rapid wrap-around.
- **Observations**:
    - Confirmed real mouse packets were arriving (`0x18`, `0xFF`, `0x00`) but intermixed with `0x00000000` ghost frames.
    - Identified root cause: PIO program wrapping around while the clock was still Low from the Stop bit, triggering a false frame start.
    - Updated `ps2_pio_rx.pio` with `wait 1 gpio 3` at the end to ensure the line returns to High/Idle before a new frame can start.
    - Updated `ps2_pio.c` ISR to drain the FIFO in a loop and explicitly discard `0x00000000` frames.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: User to flash and verify if cursor movement is now smooth and sync errors are gone.

### 2026-01-10 | PIO-Based PS/2 Driver - Physical Test 10
- **Commit**: Pending | **Result**: PENDING PHYSICAL VERIFICATION
- **Objective**: Eliminate remaining ghost frames and verify movement smoothness.
- **Observations**:
    - Confirmed valid 3-byte packets (`0x28 0xXX 0xYY`) are arriving but intermixed with `0x00000000` causing sync errors.
    - Updated `ps2_pio_rx.pio` with `[2]` debounce delay to prevent multi-sampling same bit.
    - Updated `ps2_pio.c` with a distinctive `T10 RAW` log to verify code execution.
    - Moved noise filtering to the very start of the ISR.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: User to flash and check for `T10 RAW` logs. Valid frames are expected to flow without zero-frame interruptions.

### 2026-01-10 | PIO-Based PS/2 Driver - Physical Test 11
- **Commit**: Pending | **Result**: ❌ FAIL
- **Objective**: Verify fix for `PS2 Sync Error` by implementing instant re-sync logic in `zmk_input_mouse_ps2.c`.
- **Observations**:
    - The sync error fix works as intended; the driver now recovers gracefully from malformed packets.
    - The driver is correctly processing PS/2 data and logging `Mouse Packet` events with X/Y values.
    - **However, the host OS is not showing any cursor movement.** This indicates the data is being lost somewhere between the `zmk_input_mouse_ps2` driver and the USB HID endpoint.
- **Hardware Status**: PENDING PHYSICAL VERIFICATION
- **Next Steps**: Investigate the ZMK configuration (`.conf`, `.overlay`) to ensure the pointing device is correctly enabled and routed to the main pointing subsystem.


