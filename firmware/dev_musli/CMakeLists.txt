cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(dev_musli)

add_executable(dev_musli
        dev_musli.c
        )

target_sources(dev_musli PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/dev_musli.c
        ${CMAKE_CURRENT_LIST_DIR}/pio_spi.c
        )

pico_sdk_init()
pico_generate_pio_header(dev_musli ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_link_libraries(dev_musli PRIVATE pico_stdlib hardware_resets hardware_irq hardware_spi hardware_pio)

pico_add_extra_outputs(dev_musli)
