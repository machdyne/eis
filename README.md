# Eis Computer

Eis is a small FPGA computer designed by Lone Dynamics Corporation.

![Eis Computer](https://github.com/machdyne/eis/blob/88b2240afcf820420434226afd51348e94bad919/eis.png)

This repo contains schematics, pinouts, example firmware, gateware, documentation and a 3D printable case.

Find more information on the [Eis product page](https://machdyne.com/product/eis-computer/).

## Blinky 

Building the blinky example requires [Yosys](https://github.com/YosysHQ/yosys), [nextpnr-ice40](https://github.com/YosysHQ/nextpnr) and [IceStorm](https://github.com/YosysHQ/icestorm).

Assuming they are installed, you can simply type `make` to build the gateware, which will be written to output/blinky.bin. You can then connect the USB-C port to your computer and use the latest version of [ldprog](https://github.com/machdyne/ldprog) to write the gateware to the device.

## Programming

The RP2040 firmware, FPGA SRAM and flash MMOD can be programmed over the USB-C port.

Configure the FPGA SRAM:

```
$ ldprog -i -s blinky.bin
```

Program the flash MMOD:

```
$ ldprog -i -f blinky.bin
```

## Firmware

Eis ships with RP2040 [firmware](firmware) based on the [Müsli](https://github.com/machdyne/musli) firmware which allows it to communicate with [ldprog](https://github.com/machdyne/ldprog).

The firmware is responsible for initializing the system, [configuring and outputting the system clock](https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__clocks.html#details), and either configuring the FPGA or telling the FPGA to configure itself from the MMOD.

The system clock (CLK\_RP) is 48MHz by default.

The firmware can be updated by holding down the BOOTSEL button, connecting the USB-C port to your computer, and then dragging and dropping a new UF2 file to the device filesystem.

## Zucker

[Zucker](https://github.com/machdyne/zucker) is a RISC-V SOC that supports Eis.

## Pinouts

 * [MMOD](https://github.com/machdyne/mmod)
 * [DDMI](https://github.com/machdyne/ddmi)
