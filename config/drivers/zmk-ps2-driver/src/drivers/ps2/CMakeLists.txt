# /config/drivers/zmk-ps2-driver/src/drivers/ps2/CMakeLists.txt

# Create a library for this driver
zephyr_library()

# Add sources to the library if the corresponding Kconfig is enabled
zephyr_library_sources_ifdef(CONFIG_PS2_GPIO ps2_gpio.c)
zephyr_library_sources_ifdef(CONFIG_PS2_UART ps2_uart.c)