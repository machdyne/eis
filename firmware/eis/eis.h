/**
 * Copyright (c) 2022 Lone Dynamics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef EIS_H_
#define EIS_H_

#define MUSLI_SPI_RX_PIN   24		// CSPI_SO
#define MUSLI_SPI_CSN_PIN  22
#define MUSLI_SPI_SCK_PIN  26
#define MUSLI_SPI_TX_PIN   27		// CSPI_SI

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

#define MUSLI_CMD_REBOOT 0xf0

#define ICE40_CDONE 3
#define ICE40_CRESET 2

#define EIS_CLKOUT 23

#define EIS_TX 16			// rpint spi master mosi
#define EIS_RX 17			// spi slave hold
#define EIS_HOLD EIS_RX

#define EIS_INT 18			// rpint master clk

#define EIS_SD_SCK	26
#define EIS_SD_MISO	24
#define EIS_SD_MOSI	27
#define EIS_SD_SS		5

#endif
