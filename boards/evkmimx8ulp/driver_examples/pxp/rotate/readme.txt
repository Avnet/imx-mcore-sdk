Overview
========
The PXP rotate project shows how to use the PXP rotate function. If this example
runs correctly, you will see a square with three color(red, green and blue).
The rotate degree is changing.

Toolchain supported
===================
- IAR embedded Workbench  9.10.2
- GCC ARM Embedded  10.2.1

Hardware requirements
=====================
- Micro USB cable
- MIMX8ULP-EVK board
- J-Link Debug Probe
- 5V power supply
- RK055AHD091(rm68200) panel or RK055MHD091(hx8394) panel

Board settings
==============
Connect the MIPI panel to MIMX8ULP-EVK board J18.

Prepare the Demo
================
1.  Connect 5V power supply and J-Link Debug Probe to the board, switch SW10 to power on the board.
2.  Connect a micro USB cable between the host PC and the J17 USB port on the target board.
3.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Make sure that uboot is workable(flash imx-boot-imx8ulp-evk-sd.bin-flash_singleboot to emmc with uuu).
    Note: You can get the file imx-boot-imx8ulp-evk-sd.bin-flash_singleboot from linux release.
5.  Open two serials lines provided at J17 USB port.
    (e.g. /dev/ttyUSB0~3, /dev/ttyUSB2 for A Core, /dev/ttyUSB3 for M Core)
6.  Press the reset button on your board and boot to uboot and let uboot not using diplay peripherals
    => setenv video_off yes;saveenv
    => reset
7.  Let Stop in Uboot.
8.  Build the project. Uses RK055MHD091 panel by default
    If want to use the RK055AHD091 panel, change "#define USE_MIPI_PANEL MIPI_PANEL_RK055MHD091" to "#define USE_MIPI_PANEL MIPI_PANEL_RK055AHD091" in display_support.h
9.  Running the example with IAR/gdb.(Not running it with imx-mkimage, unless it will be failed to running).

Running the demo
================
When the example runs, the screen shows what described in overview.
The log below shows the output of the demo in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
LCDIF pixel clock is: 66000000Hz

PXP Rotate example start...
LCDIF pixel clock is: 66000000Hz
MIPI DSI tx_esc_clk frequency: 19800000Hz
MIPI DSI DPHY bit clock: 444000000Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
