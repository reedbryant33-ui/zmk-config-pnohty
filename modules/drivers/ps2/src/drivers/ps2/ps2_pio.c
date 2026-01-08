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
    // NOTE: The PIO assembly expects SCL at a specific location for 'wait'
    // and SDA at a specific location for 'in'.
    
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
    const struct device *dev = NULL; // Need a way to get dev from data... typically stored in data
    // Assuming single instance for now or we need a backpointer
    
    // Hack: reconstruct dev pointer or restructure data
    // For this implementation, let's just use the stored callback
    
    // Read from FIFO
    while (!pio_sm_is_rx_fifo_empty(data->pio, 0)) { // 0 is placeholder for sm
        uint32_t raw_data = pio_sm_get(data->pio, 0);
        
        // Process bits...
        // The raw_data contains the 11-bit frame (or parts of it)
        // ps2_rx program pushes 32 bits, but we only shift 11
        // We need to adjust the PIO program to push per frame or mask here
        
        // TODO: Frame reconstruction
        // For now, let's just log
        LOG_DBG("PIO RX: %x", raw_data);
        
        if (data->callback) {
             // data->callback(dev, byte_value);
        }
    }
}

// --- ISR ---

static void ps2_pio_isr(const struct device *dev) {
    struct ps2_pio_data *data = dev->data;
    // Clear IRQ?
    k_work_submit(&data->rx_work);
}


// --- Init ---

static int ps2_pio_init(const struct device *dev) {
    const struct ps2_pio_config *config = dev->config;
    struct ps2_pio_data *data = dev->data;

    data->pio = pio_rpi_pico_get_pio(config->pio_dev);
    if (!data->pio) {
        LOG_ERR("Failed to get PIO instance");
        return -ENODEV;
    }

    k_work_init(&data->rx_work, ps2_pio_rx_work_handler);

    ps2_pio_configure_rx(dev);

    // Setup interrupt (this part needs Zephyr-specific IRQ connection which is tricky for shared PIO)
    // Zephyr's PIO driver handles the NVIC IRQ, we just need to register a callback?
    // Or we poll?
    // pio_rpi_pico driver doesn't seem to expose a generic IRQ callback mechanism easily.
    // We might have to poll for now or hook into the IRQ.
    
    // For MVP: Polling via timer? Or just check if Zephyr exposes IRQ.
    
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
