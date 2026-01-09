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

### 2026-01-09 | PIO-Based PS/2 Driver - Physical Test 4
- **Commit**: [TBD] | **Result**: 🟡 PROGRESS (Data Captured)
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


