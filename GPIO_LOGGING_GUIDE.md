# GPIO Logging Configuration Guide

## Current Status
GPIO logging for PS/2 driver **IS available** but **NOT enabled** in your current build.

The `infused-kim/kb_zmk_ps2_mouse_trackpoint_driver` includes a built-in GPIO interrupt debug feature that's disabled by default.

## Enabling GPIO Interrupt Logging

### In ps2_gpio.c (Existing Code)
The driver has commented code for interrupt logging:

```c
#if IS_ENABLED(CONFIG_PS2_GPIO_INTERRUPT_LOG_ENABLED)
    // Detailed logging of every GPIO interrupt
    k_work_init(&interrupt_log_print_worker, ps2_gpio_interrupt_log_print_worker);
#endif
```

### To Enable It

**Add to `config/sweep_bling.conf`**:
```conf
# Debug: Enable GPIO interrupt logging for PS/2 debugging
CONFIG_PS2_GPIO_INTERRUPT_LOG_ENABLED=y
CONFIG_LOG_PROCESS_THREAD_STARTUP_DELAY_MS=3000
```

The second option delays log processing to ensure PS/2 init messages appear early.

### What You'll See When Enabled

**Expected output (successful RX)**:
```
[00:00:00.404,663] <inf> ps2_gpio: Interrupt on SCL (GP3): state=0
[00:00:00.404,670] <inf> ps2_gpio: Read SDA (GP2): bit=1
[00:00:00.404,677] <inf> ps2_gpio: Interrupt on SCL (GP3): state=0
[00:00:00.404,684] <inf> ps2_gpio: Read SDA (GP2): bit=0
...
[00:00:00.404,734] <inf> ps2_gpio: Complete PS/2 frame received: 0xaa (self-test pass)
```

**What to look for**:
- SCL interrupts are firing (proves GPIO is connected)
- SDA bit readings alternate (proves clock/data synchronization)
- Frame boundaries detected correctly
- Device ID response received (0xaa = self-test success)

**If interrupts DON'T fire** → GPIO pins not receiving PS/2 signals → Check wiring

### Additional Debug Options

**More verbose logging** (add to sweep_bling.conf):
```conf
# Increase log level for PS/2 module
CONFIG_ZMK_LOG_LEVEL=4        # 4=DEBUG, 3=INFO
CONFIG_PS2_LOG_LEVEL=4         # Debug-level logging in PS/2 drivers

# Show all available log messages early
CONFIG_LOG_PROCESS_THREAD_STARTUP_DELAY_MS=5000
```

**Memory for logs** (if output truncated):
```conf
CONFIG_LOG_BUFFER_SIZE=8192    # Increase circular log buffer
CONFIG_LOG_STRDUP_MAX_STRING=256  # Allow longer log messages
```

## How to Use This Information

1. **Quick test**: Add GPIO logging options to `sweep_bling.conf`
2. **Rebuild**: Run build command from instructions
3. **Flash & Monitor**: Connect serial monitor (https://webserial.io)
4. **Analyze**:
   - If SCL interrupts appear → GPIO wiring is correct
   - If SDA bits appear → PS/2 protocol is being detected
   - If no interrupts → Physical wiring issue (measure with multimeter)

## GPIO Logging vs PIO Approach

| Aspect | GPIO Logging | PIO Driver |
|--------|-------------|-----------|
| Setup Time | 5 minutes | 3-5 days |
| Purpose | Diagnose existing issues | Production solution |
| CPU Overhead | Still high (just visible) | Very low |
| Insight | See exact bit pattern | See why it works (or doesn't) |

**Recommendation**: Use GPIO logging for 30 minutes to diagnose wiring/signal issues first, then consider PIO if you want production-grade reliability.

## Key Files for Reference

- **Driver code with logging**: `modules/drivers/ps2/ps2_gpio.c` (in your build/zephyr)
- **Logging macros**: `#include <zephyr/logging/log.h>`
- **Config options**: `modules/drivers/ps2/Kconfig`
- **This exploration**: `PIO_PS2_DRIVER_EXPLORATION.md`

## Next Steps

1. Add the `CONFIG_PS2_GPIO_INTERRUPT_LOG_ENABLED=y` option
2. Rebuild firmware
3. Flash and monitor serial output
4. Check if GPIO interrupts are firing
5. If interrupts fire but trackpoint doesn't work → PIO or frame reconstruction issue
6. If no interrupts → Wiring/power issue on GPIO pins

---

**Your Situation**: Keyboard matrix works perfectly (USB/HID all good), but PS/2 is completely silent. GPIO logging will tell you if it's a **wiring issue** (no interrupts) or a **driver issue** (interrupts happen but bad data).
