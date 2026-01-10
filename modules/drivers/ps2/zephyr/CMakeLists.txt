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

set(PS2_PIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers/ps2)

zephyr_library_sources(${PS2_PIO_DIR}/ps2_pio.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/../src/zmk_input_mouse_ps2.c)

# Ensure PIO headers are generated before compiling the driver
if(CONFIG_PS2_PIO)
    # Using absolute paths to the pioasm tool
    set(PIOASM /Users/reed/dev/zmk-config-pnohty/build-tools/pioasm/pioasm)
    
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_rx.pio.h
        COMMAND ${PIOASM} -o c-sdk ${PS2_PIO_DIR}/ps2_pio_rx.pio ${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_rx.pio.h
        DEPENDS ${PS2_PIO_DIR}/ps2_pio_rx.pio
    )

    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_tx.pio.h
        COMMAND ${PIOASM} -o c-sdk ${PS2_PIO_DIR}/ps2_pio_tx.pio ${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_tx.pio.h
        DEPENDS ${PS2_PIO_DIR}/ps2_pio_tx.pio
    )
    
    # We must add these as generated sources so CMake knows to build them
    zephyr_library_sources(${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_rx.pio.h)
    zephyr_library_sources(${CMAKE_CURRENT_BINARY_DIR}/ps2_pio_tx.pio.h)
endif()

# Include directories
zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR})
zephyr_include_directories(${PS2_PIO_DIR})
