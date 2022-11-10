cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(eis)

add_executable(eis
        eis.c
        )

set(PICO_PIO_USB_SRC "${CMAKE_CURRENT_LIST_DIR}/pico-pio-usb")

target_sources(eis PUBLIC
	${CMAKE_CURRENT_LIST_DIR}/eis.c
	${CMAKE_CURRENT_LIST_DIR}/pio_spi.c
	${PICO_PIO_USB_SRC}/pio_usb.c
	${PICO_PIO_USB_SRC}/pio_usb_device.c
	${PICO_PIO_USB_SRC}/pio_usb_host.c
	${PICO_PIO_USB_SRC}/usb_crc.c
	)

target_include_directories(eis PUBLIC
   ${CMAKE_CURRENT_LIST_DIR}
   ${PICO_PIO_USB_SRC}
   )

pico_sdk_init()

pico_generate_pio_header(eis ${CMAKE_CURRENT_LIST_DIR}/spi.pio)
pico_generate_pio_header(eis ${CMAKE_CURRENT_LIST_DIR}/spi_master_tx.pio)

target_link_libraries(eis PRIVATE pico_stdlib hardware_resets hardware_uart hardware_irq hardware_spi hardware_pio pico_multicore hardware_dma tinyusb_host)

pico_add_extra_outputs(eis)
