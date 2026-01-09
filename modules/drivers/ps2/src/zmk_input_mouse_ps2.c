#define DT_DRV_COMPAT zmk_input_mouse_ps2

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>

LOG_MODULE_REGISTER(zmk_input_mouse_ps2, LOG_LEVEL_DBG);

struct zmk_input_mouse_ps2_config {
    const struct device *ps2_dev;
};

struct zmk_input_mouse_ps2_data {
    uint8_t packet[3];
    uint8_t packet_index;
    const struct device *dev;
};

// Global reference for callback (since PS2 API doesn't pass user data)
static const struct device *last_instance;

static void zmk_input_mouse_ps2_callback(const struct device *ps2_dev, uint8_t byte)
{
    // Use the last initialized instance
    const struct device *dev = last_instance;
    struct zmk_input_mouse_ps2_data *data = dev->data;
    
    // Simple state machine for 3-byte packet
    // Byte 1: Y overflow, X overflow, Y sign, X sign, 1, Middle, Right, Left
    
    // Sync check: Bit 3 of Byte 1 must be 1
    if (data->packet_index == 0 && !(byte & 0x08)) {
        LOG_WRN("PS2 Sync Error: Byte 1 bit 3 not set (%02x)", byte);
        return; 
    }
    
    data->packet[data->packet_index++] = byte;
    
    if (data->packet_index == 3) {
        // Full packet received
        uint8_t flags = data->packet[0];
        int16_t x = data->packet[1];
        int16_t y = data->packet[2];
        
        // Handle sign extension (9-bit signed values)
        if (flags & 0x10) x = (int16_t)(x | 0xFF00);
        if (flags & 0x20) y = (int16_t)(y | 0xFF00);
        
        // Log the movement
        LOG_DBG("Mouse Packet: X=%d, Y=%d, Btn=%x", x, y, flags & 0x07);
        
        // Report to Zephyr Input Subsystem
        // Note: Y is usually inverted in PS/2 vs Screen coordinates
        input_report_rel(dev, INPUT_REL_X, x, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, -y, false, K_NO_WAIT); // Invert Y
        
        // Report Buttons
        input_report_key(dev, INPUT_BTN_LEFT, flags & 0x01, false, K_NO_WAIT);
        input_report_key(dev, INPUT_BTN_RIGHT, flags & 0x02, false, K_NO_WAIT);
        input_report_key(dev, INPUT_BTN_MIDDLE, flags & 0x04, true, K_NO_WAIT); // Sync on last
        
        data->packet_index = 0;
    }
}

static int zmk_input_mouse_ps2_init(const struct device *dev)
{
    const struct zmk_input_mouse_ps2_config *config = dev->config;
    struct zmk_input_mouse_ps2_data *data = dev->data;
    
    data->dev = dev;
    last_instance = dev;

    if (!device_is_ready(config->ps2_dev)) {
        LOG_ERR("PS2 device not ready");
        return -ENODEV;
    }

    if (ps2_config(config->ps2_dev, zmk_input_mouse_ps2_callback) < 0) {
        LOG_ERR("Failed to configure PS2 callback");
        return -EIO;
    }
    
    if (ps2_enable_callback(config->ps2_dev) < 0) {
        LOG_ERR("Failed to enable PS2 callback");
        return -EIO;
    }
    
    // Attempt to send Reset (FF) and Enable Data Reporting (F4)
    if (ps2_write(config->ps2_dev, 0xF4) < 0) {
        LOG_WRN("Failed to enable data reporting (F4) - Write might not be supported");
    }
    
    return 0;
}

#define ZMK_INPUT_MOUSE_PS2_DEFINE(n)                                       \
    static const struct zmk_input_mouse_ps2_config zmk_input_mouse_ps2_config_##n = { \
        .ps2_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, ps2_device)),          \
    };                                                                      \
    static struct zmk_input_mouse_ps2_data zmk_input_mouse_ps2_data_##n;    \
                                                                            \
    DEVICE_DT_INST_DEFINE(n, zmk_input_mouse_ps2_init, NULL,                \
                          &zmk_input_mouse_ps2_data_##n,                    \
                          &zmk_input_mouse_ps2_config_##n,                  \
                          POST_KERNEL, 90,                                  \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(ZMK_INPUT_MOUSE_PS2_DEFINE)
