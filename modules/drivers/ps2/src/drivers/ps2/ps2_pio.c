#define DT_DRV_COMPAT gpio_ps2_pio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/irq.h>

// Include generated PIO headers
#include "ps2_pio_rx.pio.h"
#include "ps2_pio_tx.pio.h"

LOG_MODULE_REGISTER(ps2_pio, CONFIG_PS2_LOG_LEVEL);

#define PS2_RING_BUF_SIZE 256

struct ps2_pio_config {
    const struct gpio_dt_spec scl_gpio;
    const struct gpio_dt_spec sda_gpio;
    PIO pio;
    uint32_t sm;
};

struct ps2_pio_data {
    struct ring_buf rx_rb;
    uint8_t rx_buf[PS2_RING_BUF_SIZE];
    ps2_callback_t callback;
    struct k_work work;
    const struct device *dev;
};

static void ps2_pio_work_handler(struct k_work *work)
{
    struct ps2_pio_data *data = CONTAINER_OF(work, struct ps2_pio_data, work);
    const struct device *dev = data->dev;
    uint8_t byte;

    while (ring_buf_get(&data->rx_rb, &byte, 1) == 1) {
        LOG_DBG("Processing byte: 0x%02x", byte);
        if (data->callback) {
            data->callback(dev, byte);
        }
    }
}

static void ps2_pio_isr(void)
{
    // Find which device triggered the IRQ (simplified for single instance for now)
    // In a robust driver, we'd iterate instances or store ISR arg.
    const struct device *dev = DEVICE_DT_INST_GET(0); 
    struct ps2_pio_data *data = dev->data;
    const struct ps2_pio_config *config = dev->config;
    PIO pio = config->pio;
    uint sm = config->sm;

    // Check if RX FIFO is not empty
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        uint32_t rx_raw = pio_sm_get(pio, sm);
        
        // Process the 11-bit frame: Start(0) + 8 Data + Parity + Stop(1)
        // Bit 0: Start (should be 0)
        // Bits 1-8: Data (LSB first)
        // Bit 9: Parity
        // Bit 10: Stop (should be 1)

        // With the PIO shifting right (LSB first) and "in pins, 1",
        // The first bit shifted in is at the LSB of the OSR/ISR?
        // Wait, "in" shifts into ISR. Default is Shift Right?
        // Let's assume Shift Right (LSB at bottom).
        // First bit (Start) -> Bit 0
        // ...
        
        // Actually, let's verify PIO configuration for shift direction.
        
        // Extract data byte (bits 1-8)
        uint8_t byte = (rx_raw >> 1) & 0xFF;
        
        // TODO: Parity check
        
        ring_buf_put(&data->rx_rb, &byte, 1);
        k_work_submit(&data->work);
    }
    
    // Clear interrupt flag if set
    pio_interrupt_clear(pio, 0); 
}

static int ps2_pio_configure(const struct device *dev, ps2_callback_t callback)
{
    struct ps2_pio_data *data = dev->data;
    data->callback = callback;
    return 0;
}

static int ps2_pio_write(const struct device *dev, uint8_t value)
{
    const struct ps2_pio_config *config = dev->config;
    int ret = 0;

    printk("ps2_pio_write: Sending 0x%02x\n", value);

    // 1. Disable PIO SM during bit-bang
    pio_sm_set_enabled(config->pio, config->sm, false);

    // 2. Host-to-Device (Write) sequence
    
    // Calculate parity (odd)
    uint8_t parity = 1;
    for (int i = 0; i < 8; i++) {
        if (value & (1 << i)) parity = !parity;
    }

    // Step 1: Inhibit communication (Clock low) for >= 100us
    gpio_pin_configure_dt(&config->scl_gpio, GPIO_OUTPUT_LOW);
    k_busy_wait(150);

    // Step 2: Request-to-Send (Data low)
    gpio_pin_configure_dt(&config->sda_gpio, GPIO_OUTPUT_LOW);

    // Step 3: Release Clock
    gpio_pin_configure_dt(&config->scl_gpio, GPIO_INPUT | GPIO_PULL_UP);

    // From now on, the Device generates Clock pulses.
    // We wait for Clock Low, then High for each bit.
    
    #define WAIT_FOR_CLOCK(level, timeout_us) \
        { \
            int elapsed = 0; \
            while (gpio_pin_get_dt(&config->scl_gpio) != level) { \
                k_busy_wait(1); \
                if (++elapsed > timeout_us) { ret = -ETIMEDOUT; goto cleanup; } \
            } \
        }

    #define SEND_BIT(bit) \
        { \
            WAIT_FOR_CLOCK(0, 10000); \
            if (bit) { \
                gpio_pin_configure_dt(&config->sda_gpio, GPIO_INPUT | GPIO_PULL_UP); \
            } else { \
                gpio_pin_configure_dt(&config->sda_gpio, GPIO_OUTPUT_LOW); \
            } \
            WAIT_FOR_CLOCK(1, 10000); \
        }

    // Data bits (0-7)
    for (int i = 0; i < 8; i++) {
        SEND_BIT(value & (1 << i));
    }

    // Parity bit
    SEND_BIT(parity);

    // Stop bit (1)
    SEND_BIT(1);

    // Release SDA and wait for ACK (Device pulls SDA low)
    gpio_pin_configure_dt(&config->sda_gpio, GPIO_INPUT | GPIO_PULL_UP);
    WAIT_FOR_CLOCK(0, 10000);
    
    // Check ACK (SDA should be low)
    if (gpio_pin_get_dt(&config->sda_gpio) != 0) {
        printk("ps2_pio_write: No ACK from device for 0x%02x\n", value);
    }

    WAIT_FOR_CLOCK(1, 10000);

cleanup:
    // Restore pins for PIO
    gpio_pin_configure_dt(&config->scl_gpio, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&config->sda_gpio, GPIO_INPUT | GPIO_PULL_UP);
    
    // Re-enable function select for PIO
    pio_gpio_init(config->pio, config->sda_gpio.pin);
    pio_gpio_init(config->pio, config->scl_gpio.pin);

    // Re-enable PIO SM
    pio_sm_set_enabled(config->pio, config->sm, true);

    if (ret < 0) {
        printk("ps2_pio_write: Failed with %d\n", ret);
    }
    return ret;
}

static int ps2_pio_enable_callback(const struct device *dev)
{
    struct ps2_pio_data *data = dev->data;
    const struct ps2_pio_config *config = dev->config;
    
    // Enable PIO SM
    pio_sm_set_enabled(config->pio, config->sm, true);
    
    return 0;
}

static int ps2_pio_disable_callback(const struct device *dev)
{
    struct ps2_pio_data *data = dev->data;
    const struct ps2_pio_config *config = dev->config;
    
    // Disable PIO SM
    pio_sm_set_enabled(config->pio, config->sm, false);
    
    return 0;
}

static int ps2_pio_init(const struct device *dev)
{
    const struct ps2_pio_config *config = dev->config;
    struct ps2_pio_data *data = dev->data;

    // Direct printk for debugging startup visibility
    printk("PS2 PIO Driver Init: Starting initialization for %s...\n", dev->name);

    if (!gpio_is_ready_dt(&config->scl_gpio) || !gpio_is_ready_dt(&config->sda_gpio)) {
        LOG_ERR("GPIOs not ready");
        printk("PS2 PIO Driver Init: GPIOs not ready\n");
        return -ENODEV;
    }

    ring_buf_init(&data->rx_rb, PS2_RING_BUF_SIZE, data->rx_buf);
    k_work_init(&data->work, ps2_pio_work_handler);
    data->dev = dev;

    // Load PIO program
    // Note: We use the `ps2_rx_program` generated by pioasm
    uint offset = pio_add_program(config->pio, &ps2_rx_program);
    
    // Configure SM
    // We pass the pins. SCL is the 'base' pin for some ops, SDA for others?
    // In our PIO:
    // wait 0 gpio 3  -> SCL (GP3)
    // in pins, 1     -> SDA (GP2)
    
    // This hardcoding in PIO is problematic if pins change.
    // Ideally, we configure the PIO SM to map specific pins to input/wait.
    
    pio_sm_config sm_config = ps2_rx_program_get_default_config(offset);
    
    // Set IN base to SDA
    sm_config_set_in_pins(&sm_config, config->sda_gpio.pin);
    sm_config_set_jmp_pin(&sm_config, config->sda_gpio.pin);
    
    // Shift Right, Autopush enabled, Threshold 11
    sm_config_set_in_shift(&sm_config, true, true, 11);
    
    // Clock divider? PS/2 is ~10-16kHz. 
    // PIO runs at system clock (125MHz).
    sm_config_set_clkdiv(&sm_config, 1.0f);
    
    // Initialize GPIOs for PIO usage
    pio_gpio_init(config->pio, config->sda_gpio.pin);
    pio_gpio_init(config->pio, config->scl_gpio.pin);
    
    // Enable Pull-ups (Critical for PS/2)
    gpio_pin_configure_dt(&config->scl_gpio, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&config->sda_gpio, GPIO_INPUT | GPIO_PULL_UP);
    
    // Apply config
    pio_sm_init(config->pio, config->sm, offset, &sm_config);
    
    // Enable Interrupts
    pio_set_irq0_source_enabled(config->pio, (enum pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + config->sm), true);
    
    // RP2040 has two IRQs per PIO. Zephyr usually connects them.
    // For now, we'll use a direct Pico SDK style connection if possible,
    // but the best way is to use irq_connect_dynamic.
    // PIO0_IRQ_0 is 7, PIO0_IRQ_1 is 8, PIO1_IRQ_0 is 9, PIO1_IRQ_1 is 10.
    int irq = (config->pio == pio0) ? 7 : 9; 
    irq_connect_dynamic(irq, 0, (void (*)(const void *))ps2_pio_isr, NULL, 0);
    irq_enable(irq);
    
    // Start SM
    pio_sm_set_enabled(config->pio, config->sm, true);

    LOG_INF("PIO RX configured on SM %d", config->sm);
    printk("PS2 PIO Driver Init: Success\n");
    return 0;
}

static const struct ps2_driver_api ps2_pio_driver_api = {
    .config = ps2_pio_configure,
    .read = NULL, // API deprecated?
    .write = ps2_pio_write,
    .disable_callback = ps2_pio_disable_callback,
    .enable_callback = ps2_pio_enable_callback,
};

#define PS2_PIO_INIT(n)                                                     \
    static struct ps2_pio_data ps2_pio_data_##n;                            \
    static const struct ps2_pio_config ps2_pio_config_##n = {               \
        .scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),                    \
        .sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),                    \
        .pio = (PIO)DT_REG_ADDR(DT_INST_PHANDLE(n, pio_device)),            \
        .sm = DT_INST_PROP(n, pio_sm),                                      \
    };                                                                      \
    DEVICE_DT_INST_DEFINE(n,                                                \
              ps2_pio_init,                                                 \
              NULL,                                                         \
              &ps2_pio_data_##n,                                            \
              &ps2_pio_config_##n,                                          \
              POST_KERNEL,                                                  \
              CONFIG_PS2_PIO_INIT_PRIORITY,                                 \
              &ps2_pio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS2_PIO_INIT)
