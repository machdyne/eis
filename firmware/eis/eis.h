/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef EIS_H_
#define EIS_H_

#define MUSLI_SPI_RX_PIN   24    // CSPI_SO
#define MUSLI_SPI_CSN_PIN  22
#define MUSLI_SPI_SCK_PIN  26
#define MUSLI_SPI_TX_PIN   27    // CSPI_SI

#define MUSLI_CMD_READY 0x00
#define MUSLI_CMD_INIT 0x01

#define MUSLI_CMD_GPIO_SET_DIR 0x10
#define MUSLI_CMD_GPIO_DISABLE_PULLS 0x11
#define MUSLI_CMD_GPIO_PULL_UP 0x12
#define MUSLI_CMD_GPIO_PULL_DOWN 0x13

#define MUSLI_CMD_GPIO_GET 0x20
#define MUSLI_CMD_GPIO_PUT 0x21

#define MUSLI_CMD_SPI_READ 0x80
#define MUSLI_CMD_SPI_WRITE 0x81
#define MUSLI_CMD_CFG_PIO_SPI 0x8f

#define MUSLI_CMD_REBOOT 0xf0

// pins

#define ICE40_CDONE 3
#define ICE40_CRESET 2

#define EIS_CLKOUT 23

#define EIS_TX 16
#define EIS_RX 17
#define EIS_INT 18
#define EIS_SS 25

#define MUSLI_SCK EIS_INT
#define MUSLI_MOSI EIS_RX

#define EIS_SD_SCK   26
#define EIS_SD_MISO  24
#define EIS_SD_MOSI  27
#define EIS_SD_SS    5

#define SPI_MODE_HW 1
#define SPI_MODE_PIO 2

// ---

#include "usb_common.h"

// Struct in which we keep the endpoint configuration
typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);
struct usb_endpoint_configuration {
    const struct usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    // Pointers to endpoint + buffer control registers
    // in the USB controller DPSRAM
    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t *data_buffer;

    // Toggle after each packet (unless replying to a SETUP)
    uint8_t next_pid;
};

// Struct in which we keep the device configuration
struct usb_device_configuration {
    const struct usb_device_descriptor *device_descriptor;
    const struct usb_interface_descriptor *interface_descriptor;
    const struct usb_configuration_descriptor *config_descriptor;
    const unsigned char *lang_descriptor;
    const unsigned char **descriptor_strings;
    // USB num endpoints is 16
    struct usb_endpoint_configuration endpoints[USB_NUM_ENDPOINTS];
};

#define EP0_IN_ADDR  (USB_DIR_IN  | 0)
#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP1_OUT_ADDR (USB_DIR_OUT | 1)
#define EP2_IN_ADDR  (USB_DIR_IN  | 2)

// EP0 IN and OUT
static const struct usb_endpoint_descriptor ep0_out = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP0_OUT_ADDR, // EP number 0, OUT from host (rx to device)
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep0_in = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP0_IN_ADDR, // EP number 0, OUT from host (rx to device)
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
};

// Descriptors
static const struct usb_device_descriptor device_descriptor = {
        .bLength         = sizeof(struct usb_device_descriptor),
        .bDescriptorType = USB_DT_DEVICE,
        .bcdUSB          = 0x0110, // USB 1.1 device
        .bDeviceClass    = 0,      // Specified in interface descriptor
        .bDeviceSubClass = 0,      // No subclass
        .bDeviceProtocol = 0,      // No protocol
        .bMaxPacketSize0 = 64,     // Max packet size for ep0
        .idVendor        = 0x2e8a, // Your vendor id
        .idProduct       = 0x1025, // Your product ID
        .bcdDevice       = 0,      // No device revision number
        .iManufacturer   = 1,      // Manufacturer string index
        .iProduct        = 2,      // Product string index
        .iSerialNumber = 0,        // No serial number
        .bNumConfigurations = 1    // One configuration
};

static const struct usb_interface_descriptor interface_descriptor = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bInterfaceNumber   = 0,
        .bAlternateSetting  = 0,
        .bNumEndpoints      = 2,    // Interface has 2 endpoints
        .bInterfaceClass    = 0xff, // Vendor specific endpoint
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface         = 0
};

static const struct usb_endpoint_descriptor ep1_out = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP1_OUT_ADDR, // EP number 1, OUT from host (rx to device)
        .bmAttributes     = USB_TRANSFER_TYPE_BULK,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep2_in = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP2_IN_ADDR, // EP number 2, IN from host (tx from device)
        .bmAttributes     = USB_TRANSFER_TYPE_BULK,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
};

static const struct usb_configuration_descriptor config_descriptor = {
        .bLength         = sizeof(struct usb_configuration_descriptor),
        .bDescriptorType = USB_DT_CONFIG,
        .wTotalLength    = (sizeof(config_descriptor) +
                            sizeof(interface_descriptor) +
                            sizeof(ep1_out) +
                            sizeof(ep2_in)),
        .bNumInterfaces  = 1,
        .bConfigurationValue = 1, // Configuration 1
        .iConfiguration = 0,      // No string
        .bmAttributes = 0xc0,     // attributes: self powered, no remote wakeup
        .bMaxPower = 0x32         // 100ma
};

static const unsigned char lang_descriptor[] = {
        4,         // bLength
        0x03,      // bDescriptorType == String Descriptor
        0x09, 0x04 // language id = us english
};

static const unsigned char *descriptor_strings[] = {
        (unsigned char *) "Raspberry Pi",    // Vendor
        (unsigned char *) "\x4d\xfc\x73\x6c\x69\x20\x55\x53\x42\x20\x50\x6d\x6f\x64" // Product
};

#endif
