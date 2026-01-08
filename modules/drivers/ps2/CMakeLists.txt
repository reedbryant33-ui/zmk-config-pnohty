zephyr_include_directories(.)
target_sources_ifdef(CONFIG_PS2_UART app PRIVATE src/ps2_uart.c)
target_sources_ifdef(CONFIG_PS2_GPIO app PRIVATE src/ps2_gpio.c)

# In src/drivers/ps2/CMakeLists.txt

# Compile PIO programs (requires pioasm tool)
add_custom_command(
    OUTPUT ps2_pio_rx.h ps2_pio_tx.h
    COMMAND pioasm -o c-sdk src/drivers/ps2/ps2_pio_rx.pio ps2_pio_rx.h
    COMMAND pioasm -o c-sdk src/drivers/ps2/ps2_pio_tx.pio ps2_pio_tx.h
    DEPENDS src/drivers/ps2/ps2_pio_rx.pio src/drivers/ps2/ps2_pio_tx.pio
    COMMENT "Generating PIO programs from assembly"
)

# Add generated headers to include path
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Compile driver
target_sources_ifdef(CONFIG_PS2_PIO app PRIVATE src/drivers/ps2/ps2_pio.c)

zephyr_sources()
