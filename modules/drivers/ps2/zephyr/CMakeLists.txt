# Compile PIO programs (requires pioasm tool)
set(PIOASM /Users/reed/dev/zmk-config-pnohty/build-tools/pioasm/pioasm)

# Generate headers from PIO assembly
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_rx.pio.h
    COMMAND ${PIOASM} -o c-sdk ${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers/ps2/ps2_pio_rx.pio ${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_rx.pio.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers/ps2/ps2_pio_rx.pio
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_tx.pio.h
    COMMAND ${PIOASM} -o c-sdk ${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers/ps2/ps2_pio_tx.pio ${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_tx.pio.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers/ps2/ps2_pio_tx.pio
)

# Add generated headers directory to include path
zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR})
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers/ps2)

# Add sources (using relative paths to root)
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_PS2_PIO ../src/drivers/ps2/ps2_pio.c)
# Trigger generation of headers if driver is enabled
if(CONFIG_PS2_PIO)
    zephyr_library_sources(${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_rx.pio.h)
    zephyr_library_sources(${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_tx.pio.h)
endif()
zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_MOUSE_PS2 ../src/zmk_input_mouse_ps2.c)
