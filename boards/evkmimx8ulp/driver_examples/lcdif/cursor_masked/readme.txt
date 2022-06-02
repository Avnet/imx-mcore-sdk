Overview
========
This example demonstrates how to use the LCDIF cursor in masked mode.
In this exapmle, the screen is devided into two parts: red and blue. A cursor
is moving in the screen, the cursor contains 4 parts like this:

          +-----------------------+
          |        Part 1         |
          +-----------------------+
          |        Part 2         |
          +-----------------------+
          |        Part 3         |
          +-----------------------+
          |        Part 4         |
          +-----------------------+

Part 1 is background color set by LCDIF_SetCursorColor, it is black.
Part 2 is the backgroud frame buffer color.
Part 3 is foreground color set by LCDIF_SetCursorColor, it is white.
Part 4 is the invertion of backgroud frame buffer color.

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
- RK055AHD091 panel or RK055MHD091 panel

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
4.  Build the project, this project uses RK055MHD091 panel by default, to use the RK055AHD091 panel,
    change #define USE_MIPI_PANEL MIPI_PANEL_RK055MHD091 to #define USE_MIPI_PANEL MIPI_PANEL_RK055AHD091
    in lcdif_support.h.
5.  Download the program to the target board.
6.  Either press the reset button on your board or launch the debugger in your IDE to begin running the example.
7.  Open two serials lines provided at J17 USB port.
    (e.g. /dev/ttyUSB0~3, /dev/ttyUSB2 for A Core, /dev/ttyUSB3 for M Core)
8.  Boot to uboot and let uboot not using diplay
    => setenv video_off yes;saveenv
    => reset
9.  Let Stop in Uboot after the step 8.

Running the demo
================
When the example runs, the screen shows what described in overview.
