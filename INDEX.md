# Trackpoint Troubleshooting & Implementation Guide

**Your situation**: Keyboard matrix works perfectly ✅ | Trackpoint completely silent ❌

## Quick Navigation

### 🎯 Start Here
1. **[RESEARCH_SUMMARY.md](RESEARCH_SUMMARY.md)** - Overview of what was researched & created
2. **[GPIO_LOGGING_GUIDE.md](GPIO_LOGGING_GUIDE.md)** - 30-minute diagnostic (Tier 1)
3. **[debug_log.md](debug_log.md)** - Complete test results & findings

### 🛠️ Implementation Resources
- **[PIO_PS2_DRIVER_EXPLORATION.md](PIO_PS2_DRIVER_EXPLORATION.md)** - Full PIO driver design (Tier 3, 3-5 days)

---

## What You Get

### ✅ GPIO Logging Guide
- Shows how to enable existing debug output
- 30 minutes to determine: **wiring issue OR driver issue**
- Tells you if GPIO interrupts are even firing
- [Jump to guide](GPIO_LOGGING_GUIDE.md)

### ✅ PIO Driver Deep Dive
- Complete architecture for deterministic PS/2 on RP2040
- Zephyr integration examples
- Testing strategy
- Performance comparison vs GPIO/UART
- 3-phase development roadmap (3-5 days)
- [Jump to guide](PIO_PS2_DRIVER_EXPLORATION.md)

### ✅ Updated Debug Log
- Your serial test results documented
- Research from infused-kim & badjeff repos
- Hardware compatibility matrix
- [Jump to log](debug_log.md)

---

## Decision Framework

### The Problem
Your GPIO-PS2 driver isn't responding. Possible causes:
1. **Wiring** - GPIO pins not connected properly
2. **Power** - TrackPoint not powered
3. **Configuration** - Driver not initialized
4. **Timing** - PS/2 clock/data synchronization failing
5. **Firmware** - GPIO-PS2 approach fundamentally doesn't work on RP2040 under load

### The Diagnostic Path
```
Enable GPIO logging (30 min)
    ↓
Do GPIO interrupts fire?
    ├─ NO  → Wiring/power issue (check with multimeter)
    └─ YES → Driver/timing issue (proceed to Tier 2)
        ↓
    Add frame-level logging (1-2 days)
        ↓
    Does self-test succeed (0xaa)?
        ├─ NO  → Timing/signal issue (fine-tune driver)
        └─ YES → Configuration issue (easy fix)
```

### The Solution Paths
- **Quick Fix** (1-2 days): Debug & fix GPIO driver if wiring is good
- **Production** (3-5 days): Build PIO driver for deterministic performance

---

## File Structure

```
.
├── FLASH_INSTRUCTIONS.md           (how to flash - unchanged)
├── RESEARCH_SUMMARY.md             (this summary of research)
├── GPIO_LOGGING_GUIDE.md           (enable debug output)
├── PIO_PS2_DRIVER_EXPLORATION.md   (build new PIO driver)
├── debug_log.md                    (test results & findings)
└── config/
    └── sweep_bling.conf            (edit here for GPIO logging test)
```

---

## Quick Checklist

### To Enable GPIO Logging (30 min)
- [ ] Open `config/sweep_bling.conf`
- [ ] Add `CONFIG_PS2_GPIO_INTERRUPT_LOG_ENABLED=y`
- [ ] Add `CONFIG_LOG_PROCESS_THREAD_STARTUP_DELAY_MS=3000`
- [ ] Rebuild firmware
- [ ] Flash to RP2040-Zero
- [ ] Monitor serial output on https://webserial.io
- [ ] Check for GPIO interrupt messages

### To Investigate PIO Driver (3-5 days)
- [ ] Read [PIO_PS2_DRIVER_EXPLORATION.md](PIO_PS2_DRIVER_EXPLORATION.md)
- [ ] Understand PIO state machine architecture
- [ ] Create ps2_pio.c driver file
- [ ] Write PIO assembly programs (RX/TX)
- [ ] Implement ps2_driver_api callbacks
- [ ] Test with logic analyzer
- [ ] Integrate with zmk_mouse_ps2 module

---

## Key Facts

### About GPIO Logging
- **Status**: Already in code, just needs to be enabled
- **Time**: 30 minutes
- **Insight**: Tells you if GPIO is receiving signals at all
- **Next**: If it works, you're 99% there; if not, it's hardware

### About PIO Approach
- **Status**: Fully architected, ready for implementation
- **Hardware**: RP2040 has dedicated PIO blocks for this
- **Performance**: <1% CPU vs 70-100% for GPIO bit-banging
- **Zephyr**: Already has PIO support in 3.5.0
- **Timeline**: 3-5 days development + testing

### About Your Hardware
- **Matrix**: Works perfectly (USB enumeration, HID reports)
- **TrackPoint**: IBM/Lenovo Blue Module (SK790902-A2)
- **Controller**: RP2040-Zero (Zephyr 3.5.0 compatible)
- **Wiring**: GP3=SCL, GP2=SDA (verified in code)

---

## What Changed

### New Files Created
1. **RESEARCH_SUMMARY.md** - This summary
2. **GPIO_LOGGING_GUIDE.md** - Diagnostic guide
3. **PIO_PS2_DRIVER_EXPLORATION.md** - 9000+ word technical spec

### Updated Files
1. **debug_log.md** - Added test results, findings, action plan

### No Changes To
- `config/` files (except when you add GPIO logging config)
- `FLASH_INSTRUCTIONS.md`
- Firmware source (until you decide to implement PIO)

---

## Recommended First Action

> **Do this in the next 30 minutes:**
> 
> 1. Open [GPIO_LOGGING_GUIDE.md](GPIO_LOGGING_GUIDE.md)
> 2. Add the config option
> 3. Rebuild and flash
> 4. Check serial output
> 5. You'll know immediately if it's wiring or software

This will save you days of debugging if the answer is "GPIO isn't connected."

---

## Resources Linked

- [Zephyr PIO Documentation](https://docs.zephyrproject.org/latest/hardware/peripherals/pio.html)
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [infused-kim PS/2 Driver](https://github.com/infused-kim/kb_zmk_ps2_mouse_trackpoint_driver)
- [IBM TrackPoint Spec v4.0](https://blogs.epfl.ch/icenet/documents/Ykt3Eext.pdf)
- [Your Custom Hardware](https://github.com/idank/keyboards/tree/main/sweeq%20mx)

---

## Questions?

Each guide has:
- Quick summary at the top
- Detailed sections below
- Code examples
- Testing strategies
- References

**Start with [RESEARCH_SUMMARY.md](RESEARCH_SUMMARY.md) if you just want the highlights.**

**Jump to [GPIO_LOGGING_GUIDE.md](GPIO_LOGGING_GUIDE.md) if you want to start debugging immediately.**

**Read [PIO_PS2_DRIVER_EXPLORATION.md](PIO_PS2_DRIVER_EXPLORATION.md) if you're ready to build a production solution.**

---

**Last Updated**: 2026-01-08  
**Status**: Research & planning phase complete. Ready for Tier 1 testing.
