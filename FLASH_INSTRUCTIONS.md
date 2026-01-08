# Flashing Instructions - Sweeq MX Trackpoint Build

## Pre-Flash Checklist
1. **Keyboard disconnected** from computer
2. **USB cable ready** for connection
3. **zmk.uf2 file available** in repo root

## Flashing Steps (RP2040-Zero)

### Bootloader Mode Entry
1. **Double-click the RESET button** on the RP2040-Zero microcontroller
2. On the **second press, hold the reset button down** (don't release)
3. **Plug in the keyboard** using USB-C cable
4. **Release the reset button**
5. A mass-storage device should appear on your system (e.g., `/Volumes/RPI-RP2` on macOS)

### Flashing the Firmware (GUI Method)
1. Once the mass-storage device appears, **drag and drop `zmk.uf2`** onto the device volume
2. The device will automatically disconnect once flashing completes
3. The keyboard reboots automatically

### Flashing the Firmware (Command Line, macOS)
```bash
# After device appears in bootloader mode
cp zmk.uf2 /Volumes/RPI-RP2/ && sleep 2 && echo "Flashing complete!"
```

## Post-Flash Testing

### 1. Connect & Monitor Logs
Open a serial terminal to monitor boot and trackpoint activity:
- **VID:PID**: `1d50:615e` (OpenMoko HID device)
- **Baud Rate**: 115200
- **WebSerial**: Use https://webserial.io/ for browser-based monitoring
- **CLI Option**: Use `screen /dev/tty.usbmodem* 115200`

### 2. Test Keyboard Matrix
- Press any key - should see matrix events in logs
- Example: `kscan_matrix_read: Sending event at 4,2 state on/off`

### 3. Test Trackpoint
- Move trackpoint - should see PS/2 device initialization and input events
- Look for:
  - `ps2_device` initialization message
  - `input_listener` activity
  - Mouse cursor movement (if USB HID active)

### 4. Expected Log Messages
**Successful bootup should include:**
```
kscan_matrix_init_input_inst: ready
USB device enumerated
```

**Trackpoint device initialization (NEW):**
```
ps2_device: initializing PIO UART PS/2 bus
input_listener: ready
```

## Troubleshooting

### No Device in Bootloader Mode
- Try pressing BOOT button **before and after** inserting USB
- Check USB cable is functional
- Try different USB port

### Flashing Appears to Fail
- Make sure RPI-RP2 volume is fully mounted before copying
- Some systems may require `sync` command before ejecting
- Check file transfer completed: `ls /Volumes/RPI-RP2/`

### No Trackpoint Events After Flash
1. Check wiring: GP2 (SDA) and GP3 (SCL) connected properly
2. Check serial logs for PS/2 device errors
3. Verify module power: 3.3V on VCC, GND connected
4. Check continuity with multimeter on scl/sda lines

## Build Information
- **Firmware**: `zmk.uf2` (122880 bytes)
- **Board**: `rpi_pico` (RP2040-based)
- **Zephyr Version**: 3.5.0
- **Pete Johanson Branch**: `feat/pointers-move-scroll`
- **Flash Usage**: 2.91%
- **RAM Usage**: 10.26%
