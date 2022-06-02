Overview
========
The freertos_lpspi_b2b_slave example shows how to use LPSPI driver in FreeRTOS.
In this example are required two boards, one board is used as LPSPI master on which
is runnuing freertos_lpspi_b2b_master and another board is used as LPSPI slave on which
is running freertos_lpspi_b2b_slave example.

Connection of boards is in section Board settings.

It is required to run first the slave demo.



Toolchain supported
===================
- IAR embedded Workbench  9.10.2
- GCC ARM Embedded  10.2.1

Hardware requirements
=====================
- Micro USB cable
- two MIMX8ULP-EVK boards
- J-Link Debug Probe
- 5V power supply
- Personal Computer

Board settings
==============
Remove R139, R159 and populat R140, R158 on both base boards.

SPI one board:
Transfer data from one board instance to another board's instance.
SPI1 pins are connected with SPI1 pins of another board
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
INSTANCE0(SPI1)     CONNECTS TO         INSTANCE0(SPI1)
Pin Name   Board Location     Pin Name  Board Location
SPI1_SCK    J23 pin 4            SPI1_SCK  J23 pin 4
SPI1_SIN    J23 pin 2            SPI1_SOUT J23 pin 3
SPI1_SOUT   J23 pin 3            SPI1_SIN  J23 pin 2
SPI1_PCS0   J23 pin 1            SPI1_PCS0 J23 pin 1
GND                             GND
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
4.  Download the program to the target board.
5.  Either cold boot your board or launch the debugger in your IDE to begin running the example.

Running the demo
================
When the demo runs successfully, the log would be seen on the terminal like:

This example uses two boards to connect with one as master and anohter as slave.
Master and slave are both use interrupt way.
Please make sure you make the correct line connection. Basically, the connection is:
LPSPI_master -- LPSPI_slave
    CLK      --    CLK
    PCS      --    PCS
    SOUT     --    SIN
    SIN      --    SOUT

LPSPI slave transfer completed successfully.
LPSPI transfer all data matched !
