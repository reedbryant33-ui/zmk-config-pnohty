#define DT_DRV_COMPAT gpio_ps2_pio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <hardware/pio.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

#include <drivers/ps2.h>
#include "ps2_pio_rx.pio.h"
#include "ps2_pio_tx.pio.h"

LOG_MODULE_REGISTER(ps2_pio, CONFIG_PS2_LOG_LEVEL);

struct ps2_pio_config {
    const struct device *pio_dev;
    const struct gpio_dt_spec scl_gpio;
    const struct gpio_dt_spec sda_gpio;
    uint32_t pio_sm;
};

struct ps2_pio_data {
    const struct device *dev;
    ps2_callback_t callback;
    PIO pio;
    uint pio_offset_rx;
    uint pio_offset_tx;
    struct k_work rx_work;
};

// --- PIO Program Loading (using generated headers) ---
// Note: We're using the "raw" programs from the headers

static void ps2_pio_configure_rx(const struct device *dev) {
    const struct ps2_pio_config *config = dev->config;
    struct ps2_pio_data *data = dev->data;
    PIO pio = data->pio;
    uint sm = config->pio_sm;

    // Load RX program
    if (!pio_can_add_program(pio, &ps2_rx_program)) {
        LOG_ERR("Failed to add PIO RX program");
        return;
    }
    data->pio_offset_rx = pio_add_program(pio, &ps2_rx_program);

    // Configure State Machine
    pio_sm_config sm_config = ps2_rx_program_get_default_config(data->pio_offset_rx);
    
    // Map SCL to Wait Pin (wait 0 gpio SCL)
    // Map SDA to In Pin (in pins, 1)
    
    // Set In pin to SDA
    sm_config_set_in_pins(&sm_config, config->sda_gpio.pin);
    
    // Set JMP pin (not used in this simple program but good practice)
    sm_config_set_jmp_pin(&sm_config, config->sda_gpio.pin);

    // Initialize GPIOs for PIO control
    pio_gpio_init(pio, config->scl_gpio.pin);
    pio_gpio_init(pio, config->sda_gpio.pin);

    // Set directions (Input for RX)
    pio_sm_set_consecutive_pindirs(pio, sm, config->scl_gpio.pin, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, config->sda_gpio.pin, 1, false);
    
    // Initialize SM
    pio_sm_init(pio, sm, data->pio_offset_rx, &sm_config);
    pio_sm_set_enabled(pio, sm, true);
    
    LOG_INF("PIO RX configured on SM %d", sm);
}

// --- Driver API ---

static int ps2_pio_configure(const struct device *dev, ps2_callback_t callback) {
    struct ps2_pio_data *data = dev->data;
    data->callback = callback;
    return 0;
}

static int ps2_pio_write(const struct device *dev, uint8_t value) {
    // TODO: Implement TX using TX state machine
    // For now, trackpoints usually just send data, so RX is priority.
    // If we need to send commands (reset, sensitivity), we need the TX program.
    LOG_WRN("PS/2 Write not yet fully implemented via PIO");
    return -ENOTSUP;
}

static int ps2_pio_read(const struct device *dev, uint8_t *value) {
    // This driver is interrupt/callback driven, direct read not typical
    return -ENOTSUP;
}

static void ps2_pio_isr(const struct device *dev);

static int ps2_pio_enable_callback(const struct device *dev) {
    struct ps2_pio_data *data = dev->data;
    const struct ps2_pio_config *config = dev->config;
    
    // Enable FIFO interrupt
    pio_set_irq0_source_enabled(data->pio, pis_sm0_rx_fifo_not_empty + config->pio_sm, true);
    
    return 0;
}

static int ps2_pio_disable_callback(const struct device *dev) {
     struct ps2_pio_data *data = dev->data;
    const struct ps2_pio_config *config = dev->config;
    
    // Disable FIFO interrupt
    pio_set_irq0_source_enabled(data->pio, pis_sm0_rx_fifo_not_empty + config->pio_sm, false);
    
    return 0;
}

// --- Work Handler ---

static void ps2_pio_rx_work_handler(struct k_work *work) {
    struct ps2_pio_data *data = container_of(work, struct ps2_pio_data, rx_work);
    
    // Read from FIFO
    while (!pio_sm_is_rx_fifo_empty(data->pio, data->dev->config->pio_sm)) {
        uint32_t raw_data = pio_sm_get(data->pio, data->dev->config->pio_sm);
        
        // raw_data is 32 bits, but our PIO loop collects 11 bits.
        // Bit 0: Start (Always 0)
        // Bit 1-8: Data (LSB first)
        // Bit 9: Parity (Odd)
        // Bit 10: Stop (Always 1)
        
        uint8_t start_bit = raw_data & 0x1;
        uint8_t data_byte = (raw_data >> 1) & 0xFF;
        uint8_t parity_bit = (raw_data >> 9) & 0x1;
        uint8_t stop_bit = (raw_data >> 10) & 0x1;
        
        // Validation
        if (start_bit != 0) {
             LOG_WRN("PS/2 Frame Error: Start bit not 0 (Raw: %x)", raw_data);
             continue; 
        }
        
        if (stop_bit != 1) {
            LOG_WRN("PS/2 Frame Error: Stop bit not 1 (Raw: %x)", raw_data);
            continue;
        }
        
        // Parity Check (Odd Parity)
        // # of 1s in (Data + Parity) must be Odd.
        // So (popcount(Data) + Parity) % 2 == 1
        
        int ones = 0;
        for (int i=0; i<8; i++) {
            if ((data_byte >> i) & 1) ones++;
        }
        
        if ((ones + parity_bit) % 2 != 1) {
            LOG_WRN("PS/2 Parity Error (Data: %x, P: %d)", data_byte, parity_bit);
            continue;
        }
        
        // Valid Frame!
        LOG_DBG("PS/2 Valid Byte: %02x", data_byte);
        
        if (data->callback) {
             data->callback(data->dev, data_byte);
        }
    }
}

// Modified work handler to re-enable IRQ
static void ps2_pio_rx_work_handler_wrapper(struct k_work *work) {
    struct ps2_pio_data *data = container_of(work, struct ps2_pio_data, rx_work);
    const struct ps2_pio_config *config = data->dev->config; // Access config via stored dev pointer

    ps2_pio_rx_work_handler(work); // Process data
    
    // Re-enable IRQ
    pio_set_irq0_source_enabled(data->pio, pis_sm0_rx_fifo_not_empty + config->pio_sm, true);
}

// --- ISR ---

static void ps2_pio_isr(const struct device *dev) {
    struct ps2_pio_data *data = dev->data;
    
    // Disable IRQ, Submit Work
    pio_set_irq0_source_enabled(data->pio, pis_sm0_rx_fifo_not_empty + ((struct ps2_pio_config*)dev->config)->pio_sm, false);
    k_work_submit(&data->rx_work);
}

// --- Init ---

static int ps2_pio_init(const struct device *dev) {
    const struct ps2_pio_config *config = dev->config;
    struct ps2_pio_data *data = dev->data;

    data->dev = dev; // Store backpointer
    data->pio = pio_rpi_pico_get_pio(config->pio_dev);
    if (!data->pio) {
        LOG_ERR("Failed to get PIO instance");
        return -ENODEV;
    }

    k_work_init(&data->rx_work, ps2_pio_rx_work_handler_wrapper);

    ps2_pio_configure_rx(dev);

    // Setup interrupt
    // Hook PIO0_IRQ_0.
    
    IRQ_CONNECT(DT_IRQ_BY_NAME(DT_NODELABEL(pio0), pio0, irq), 
                DT_IRQ_BY_NAME(DT_NODELABEL(pio0), pio0, priority),
                ps2_pio_isr,
                DEVICE_DT_GET(DT_NODELABEL(trackpoint_device)), // Pass OUR device
                0);
    
    irq_enable(DT_IRQ_BY_NAME(DT_NODELABEL(pio0), pio0, irq));
    
    return 0;
}

static const struct ps2_driver_api ps2_pio_driver_api = {
    .config = ps2_pio_configure,
    .read = ps2_pio_read,
    .write = ps2_pio_write,
    .disable_callback = ps2_pio_disable_callback,
    .enable_callback = ps2_pio_enable_callback,
};

#define PS2_PIO_INIT(n)                                                     \
    static struct ps2_pio_data ps2_pio_data_##n;                            \
                                                                            \
    static const struct ps2_pio_config ps2_pio_config_##n = {               \
        .pio_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, pio_device)),           \
        .scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),                    \
        .sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),                    \
        .pio_sm = DT_INST_PROP(n, pio_sm),                                  \
    };                                                                      \
                                                                            \
    DEVICE_DT_INST_DEFINE(n, &ps2_pio_init, NULL,                           \
                          &ps2_pio_data_##n, &ps2_pio_config_##n,           \
                          POST_KERNEL, CONFIG_PS2_INIT_PRIORITY,            \
                          &ps2_pio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS2_PIO_INIT)
