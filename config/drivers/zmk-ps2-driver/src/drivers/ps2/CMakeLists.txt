# /config/drivers/zmk-ps2-driver/src/drivers/ps2/CMakeLists.txt

# Create a library for this driver
zephyr_library()

# Add sources to the library if the corresponding Kconfig is enabled
zephyr_library_sources_ifdef(CONFIG_PS2_GPIO ps2_gpio.c)
zephyr_library_sources_ifdef(CONFIG_PS2_UART ps2_uart.c)
zephyr_library_sources_ifdef(CONFIG_PS2_PIO
    ps2_pio/ps2_pio.c
)

if(CONFIG_PS2_PIO)
    pico_generate_pio_header(
        ps2_pio/ps2_rx.pio
        OUTPUT_HEADER "ps2_rx.pio.h"
    )
endif()