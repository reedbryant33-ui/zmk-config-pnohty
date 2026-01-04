# /config/drivers/zmk-ps2-driver/src/drivers/ps2/CMakeLists.txt

# Create a library for this driver
zephyr_library()

# Add sources to the library if the corresponding Kconfig is enabled
zephyr_library_sources_ifdef(CONFIG_PS2_GPIO ps2_gpio.c)
zephyr_library_sources_ifdef(CONFIG_PS2_UART ps2_uart.c)
zephyr_library_sources_ifdef(CONFIG_PS2_PIO ps2_pio/ps2_pio.c)

# Add PIO program for RP2040
if(CONFIG_PS2_PIO)
    # This assumes you have the Pico SDK integrated and pico_add_pio_program is available.
    # If not, you might need to manually compile the .pio file to .h and include it.
    pico_add_pio_program(ps2_pio_rx src/drivers/ps2/ps2_pio/ps2_rx.pio)
endif()