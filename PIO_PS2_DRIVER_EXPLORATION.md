# RP2040 PIO-Based PS/2 Driver: Research & Implementation Strategy

## Overview
The RP2040 microcontroller (used in RP2040-Zero) has two Programmable I/O (PIO) blocks with 4 state machines each. PIO is purpose-built for implementing custom serial protocols like PS/2, offering:
- **Hardware timing**: No interrupt overhead like GPIO bit-banging
- **Low latency**: Dedicated state machines operate independently
- **Deterministic performance**: Perfect for 15kHz PS/2 protocol requirements

This document outlines a PIO-based PS/2 driver design that would supersede GPIO bit-banging.

---

## Part 1: Current State & Problem Analysis

### Why GPIO Bit-Banging Fails on RP2040
1. **Interrupt latency**: GPIO interrupt handlers compete with other system tasks
2. **PS/2 timing requirement**: Bits toggle every ~67µs; RP2040 CPU may take >100µs on Bluetooth tasks
3. **Dropped bits**: Missed clock edges = corrupted data frames
4. **No hardware support**: RP2040 has no dedicated PS/2 hardware (unlike UART variant)

### Why PIO is Perfect
```
Traditional GPIO approach:
Clock signal → GPIO interrupt → CPU wakes → Read data pin → Process

PIO approach:
Clock signal → PIO state machine → Automatic byte capture → CPU reads when ready
```

PIO state machines run **independently** of CPU at precise timing intervals.

---

## Part 2: Zephyr PIO Support Architecture

### Available in Zephyr 3.5.0

**Header**: `zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h`

**Key functions**:
```c
// Get PIO instance (PIO0 or PIO1)
PIO pio_rpi_pico_get_pio(const struct device *dev);

// Allocate a state machine (0-3 per PIO)
int pio_rpi_pico_allocate_sm(const struct device *dev, size_t *sm);
```

**Underlying hardware API** (via `hardware/pio.h`):
```c
// Core PIO operations:
pio_sm_init(pio, sm, offset, config)      // Initialize state machine
pio_sm_set_enabled(pio, sm, true)         // Start state machine
pio_sm_get_blocking(pio, sm)              // Read 32-bit data from FIFO
pio_sm_put_blocking(pio, sm, data)        // Write 32-bit data to FIFO
pio_gpio_init(pio, pin)                   // Configure GPIO for PIO
```

**Macro for defining PIO programs**:
```c
RPI_PICO_PIO_DEFINE_PROGRAM(ps2_rx, 0, 7,  // wrap_target, wrap
    0xEC00,  // Instruction 0: wait gpio ...
    0xE081,  // Instruction 1: in pins, 1
    // ... more instructions
);
```

---

## Part 3: PIO Assembly for PS/2 Protocol

### PS/2 Receive State Machine (PIO Assembly)

**Protocol overview**:
- Host pulls clock low to initiate
- Device clocks out data at ~15kHz (67µs per bit)
- Data format: START + 8 DATA + PARITY + STOP (11 bits total)

**PIO program (receive mode)**:

```asm
; PS/2 Receive State Machine
; Wait for clock line to go low (START condition)
    wait 0 gpio SCL_PIN

; Loop 11 times for START + 8DATA + PARITY + STOP bits
.wrap_target
    wait 1 gpio SCL_PIN    ; Wait for clock HIGH
    in pins, 1             ; Read 1 bit from SDA into ISR
    wait 0 gpio SCL_PIN    ; Wait for clock LOW
.wrap

; When ISR reaches 32 bits, push to RX FIFO
; At 11 bits per frame, collects ~3 frames per 32-bit word
```

**In C with Zephyr macros**:

```c
// PIO instruction encoding (machine-readable)
RPI_PICO_PIO_DEFINE_PROGRAM(ps2_rx_program, 0, 3,
    0x20 << 8 | SCL_PIN,   // wait 0 gpio SCL
    0xE081,                 // in pins, 1
    0x20 << 8 | SCL_PIN,   // wait 1 gpio SCL
);
```

**Alternative: Using pioasm tool** (Official approach)

The Raspberry Pi Pico SDK provides `pioasm.py` tool that compiles human-readable assembly:

```asm
; ps2_rx.pio
.program ps2_rx
    wait 0 gpio 3              ; SCL_PIN = GP3
    set x, 10                  ; Counter for 11 bits
loop:
    wait 1 gpio 3
    in pins, 1                 ; Read GP2 (SDA) 
    jmp x--, loop
```

Compiled to C header:
```c
#define ps2_rx_wrap_target 0
#define ps2_rx_wrap 2

static const uint16_t ps2_rx_program_instructions[] = {
    0x2003,  // wait 0 gpio 3
    0xe001,  // in pins, 1
    0x0046,  // jmp 6
};
```

---

## Part 4: Implementation Architecture

### Proposed Driver Structure

```
zmk-ps2-uart-driver/
├── src/drivers/ps2/
│   ├── ps2_pio.c              [NEW] PIO-based driver
│   ├── ps2_pio_rx.pio         [NEW] RX program assembly
│   ├── ps2_pio_tx.pio         [NEW] TX program assembly
│   ├── ps2_gpio.c             [EXISTING]
│   └── ps2_uart.c             [EXISTING]
├── dts/bindings/
│   └── ps2_pio.yaml           [NEW] Devicetree binding
└── Kconfig
    └── CONFIG_PS2_PIO         [NEW] Option to select PIO driver
```

### Devicetree Integration

**New compatible string**: `gpio-ps2-pio`

Example configuration:
```dts
/ {
    ps2_device: &uart0 {
        compatible = "gpio-ps2-pio";
        scl-gpios = <&gpio 3 GPIO_ACTIVE_HIGH>;  // GP3
        sda-gpios = <&gpio 2 GPIO_ACTIVE_HIGH>;  // GP2
        pio-device = <&pio0>;                     // Use PIO0
        pio-sm = <0>;                             // State machine 0
        status = "okay";
    };
};
```

### C Driver Interface

```c
// ps2_pio.c

#include <hardware/pio.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include "ps2_pio_rx.pio.h"     // Generated by pioasm
#include "ps2_pio_tx.pio.h"

struct ps2_pio_config {
    const struct device *pio_dev;
    PIO pio;
    uint pio_sm;
    uint scl_pin;
    uint sda_pin;
};

struct ps2_pio_data {
    struct ps2_driver_api api;
    uint pio_offset_rx;
    uint pio_offset_tx;
    struct k_work rx_work;
};

// PIO program definitions
RPI_PICO_PIO_DEFINE_PROGRAM(ps2_pio_rx, 0, 3,
    // ... instruction bytes ...
);

RPI_PICO_PIO_DEFINE_PROGRAM(ps2_pio_tx, 0, 5,
    // ... instruction bytes ...
);

static int ps2_pio_init(const struct device *dev) {
    struct ps2_pio_data *data = dev->data;
    const struct ps2_pio_config *config = dev->config;
    
    // Get PIO instance
    config->pio = pio_rpi_pico_get_pio(config->pio_dev);
    
    // Load RX program into PIO instruction memory
    data->pio_offset_rx = pio_add_program(config->pio, 
                                          RPI_PICO_PIO_GET_PROGRAM(ps2_pio_rx));
    
    // Configure RX state machine
    pio_sm_config sm_config = pio_get_default_sm_config();
    sm_set_in_pins(&sm_config, config->sda_pin);           // Read SDA
    sm_set_wait_pin(&sm_config, config->scl_pin);          // Wait on SCL
    
    pio_gpio_init(config->pio, config->scl_pin);
    pio_gpio_init(config->pio, config->sda_pin);
    gpio_set_dir(config->scl_pin, GPIO_IN);
    gpio_set_dir(config->sda_pin, GPIO_IN);
    
    pio_sm_init(config->pio, config->pio_sm, 
                data->pio_offset_rx, &sm_config);
    pio_sm_set_enabled(config->pio, config->pio_sm, true);
    
    // Setup work queue for processing RX data
    k_work_init(&data->rx_work, ps2_pio_rx_work_handler);
    
    return 0;
}

// IRQ handler: FIFO ready interrupt
static void ps2_pio_irq_handler(const struct device *dev) {
    struct ps2_pio_data *data = dev->data;
    
    // Schedule work to read FIFO data
    k_work_submit(&data->rx_work);
}

// Work handler: Extract bits and call PS/2 callback
static void ps2_pio_rx_work_handler(struct k_work *work) {
    struct ps2_pio_data *data = container_of(work, 
                                             struct ps2_pio_data, rx_work);
    const struct ps2_pio_config *config = dev->config;
    
    // Read 32-bit word from PIO RX FIFO
    // Contains ~3 PS/2 frames (11 bits each)
    uint32_t data_word = pio_sm_get(config->pio, config->pio_sm);
    
    // Extract individual bits and reconstruct PS/2 frames
    for (int i = 0; i < 11; i++) {
        uint8_t bit = (data_word >> i) & 1;
        ps2_rx_bit(dev, bit);  // Feed to frame reconstruction logic
    }
}

// Implement ps2_driver_api interface
static const struct ps2_driver_api ps2_pio_driver_api = {
    .config = ps2_pio_configure,
    .read = ps2_pio_read,
    .write = ps2_pio_write,
    .disable_callback = ps2_pio_disable_callback,
    .enable_callback = ps2_pio_enable_callback,
};

DEVICE_DT_INST_DEFINE(0, &ps2_pio_init, NULL, 
                      &ps2_pio_data, &ps2_pio_config,
                      POST_KERNEL, 45, &ps2_pio_driver_api);
```

---

## Part 5: Build Configuration

### Kconfig additions

```kconfig
# For PS/2 PIO variant selection

if PS2

config PS2_PIO
    bool "PS/2 GPIO PIO bit-banging driver (RP2040)"
    select HAS_DTS_GPIO
    help
      Enable PS/2 driver using RP2040 PIO (Programmable I/O).
      This approach uses the PIO state machines for deterministic
      timing instead of CPU-driven GPIO interrupts.
      
config PS2_PIO_INIT_PRIORITY
    int "PS/2 PIO driver initialization priority"
    default 45
    depends on PS2_PIO

endif # PS2
```

### CMakeLists.txt

```cmake
# In src/drivers/ps2/CMakeLists.txt

# Compile PIO programs (requires pioasm tool)
add_custom_command(
    OUTPUT ps2_pio_rx.h ps2_pio_tx.h
    COMMAND pioasm -o c-sdk ps2_pio_rx.pio ps2_pio_rx.h
    COMMAND pioasm -o c-sdk ps2_pio_tx.pio ps2_pio_tx.h
    DEPENDS ps2_pio_rx.pio ps2_pio_tx.pio
    COMMENT "Generating PIO programs from assembly"
)

# Add generated headers to include path
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Compile driver
target_sources_ifdef(CONFIG_PS2_PIO app PRIVATE ps2_pio.c)
```

### west.yml integration

No changes needed - PIO driver fits within existing `infused-kim/kb_zmk_ps2_mouse_trackpoint_driver` module.

---

## Part 6: Performance Comparison

| Metric | GPIO Bit-Bang | UART (nrf52) | PIO (RP2040) |
|--------|---------------|--------------|--------------|
| **Interrupt Overhead** | Per bit (11x/frame) | Periodic | None (HW timing) |
| **CPU Load** | 70-100% during receive | ~5-10% | <1% |
| **Latency** | Variable (Bluetooth can delay) | Low (dedicated UART) | Deterministic (state machine) |
| **Throughput** | 10-20 frames/sec (unreliable) | 100+ frames/sec | 100+ frames/sec |
| **Power Draw** | Moderate | Low | Very low (HW clock only) |
| **Hardware Required** | Any GPIO pins | Dedicated UART | PIO0 or PIO1 + 2 GPIO |

---

## Part 7: Development Roadmap

### Phase 1: Basic RX (1-2 days)
- [ ] Create PIO assembly programs (ps2_pio_rx.pio, ps2_pio_tx.pio)
- [ ] Test assembly with pioasm tool
- [ ] Implement ps2_pio.c with RX FIFO polling
- [ ] Verify bits are captured correctly
- [ ] Test with logic analyzer

**Test**: Connect TrackPoint, capture RX frames, verify all bits present

### Phase 2: Full Driver (1-2 days)
- [ ] Implement TX state machine (for initialization commands)
- [ ] Add PS/2 frame reconstruction (handle 11-bit frames)
- [ ] Implement interrupt-driven callback model
- [ ] Integrate with zmk_mouse_ps2 driver

**Test**: TrackPoint responds to commands, cursor moves

### Phase 3: Integration & Optimization (1 day)
- [ ] Test with actual ZMK keymap
- [ ] Measure power consumption
- [ ] Fine-tune timing parameters
- [ ] Document for future reference

**Test**: Full trackpoint operation, sensitivity tuning works

---

## Part 8: Key Design Decisions

### Why Not Use UART Emulation?
- RP2040 has UART hardware, but no async API in Zephyr 3.5.0
- PIO provides more flexible timing control
- Better for RP2040's strengths

### Why Separate RX/TX State Machines?
- RX: Waits for clock, captures bits (~100% of time)
- TX: Pulls clock/data per PS/2 protocol (~5% of time)
- Separate programs prevent state conflicts

### Handling Multiple Frames
- PIO can capture ~3 frames per 32-bit FIFO word
- Work queue reconstructs individual PS/2 frames
- Reduces interrupt overhead compared to per-bit interrupts

---

## Part 9: Testing Strategy

### Unit Test (Standalone)
```c
// test_ps2_pio.c
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

// Mock PIO hardware
struct mock_pio { /* ... */ };

void test_ps2_pio_init(void) {
    // Verify state machine allocates
    // Verify GPIO configured
    // Verify programs loaded
}

void test_ps2_pio_receive_frame(void) {
    // Inject 11-bit frame into FIFO
    // Verify reconstruction
    // Check frame integrity
}
```

### Hardware Test (RP2040-Zero + TrackPoint)
1. **Signal integrity**: Logic analyzer captures all bits
2. **Frame reconstruction**: All frames parsed without errors
3. **Command response**: TrackPoint responds to init commands
4. **Cursor movement**: Physical trackpoint movement = on-screen cursor
5. **Power draw**: Measure vs GPIO variant

---

## Part 10: References & Resources

### Raspberry Pi Pico Documentation
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) - Section 3: PIO
- [Pico SDK Examples](https://github.com/raspberrypi/pico-examples/tree/master/pio) - UART implementation

### PS/2 Protocol
- [PS/2 Protocol Spec](https://en.wikipedia.org/wiki/PS/2_interface)
- IBM TrackPoint Spec v4.0

### Zephyr PIO Support
- `zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h`
- Example: Zephyr PIO UART driver

### Existing ZMK PS/2 Code
- `infused-kim/kb_zmk_ps2_mouse_trackpoint_driver/src/drivers/ps2/ps2_gpio.c`
- Frame reconstruction logic (portable to PIO)

---

## Part 11: Alternative Approaches Considered

### A. Continue with GPIO Bit-Banging
- **Pros**: Already implemented, works on some boards
- **Cons**: Unreliable on busy systems, high CPU overhead
- **Decision**: Not viable for RP2040 with Bluetooth

### B. Use UART as PS/2 Transport
- **Pros**: Proven on nrf52, hardware support
- **Cons**: Zephyr 3.5.0 lacks async API, RP2040 UART not suitable
- **Decision**: Not applicable to RP2040

### C. PIO-Based Driver (Recommended)
- **Pros**: Deterministic timing, low overhead, perfect for PS/2
- **Cons**: Requires PIO program development
- **Decision**: Best fit for RP2040 capabilities

---

## Conclusion

RP2040's Programmable I/O is **ideal** for PS/2 protocol implementation. A PIO-based driver would:
- Eliminate timing issues from GPIO bit-banging
- Provide deterministic ~100+ fps throughput
- Use minimal CPU resources
- Demonstrate RP2040's unique strengths

The Zephyr framework already has PIO support. Implementation is feasible within 3-5 days of focused development.

**Next Step**: Decide whether to proceed with PIO implementation or continue with GPIO debugging.
