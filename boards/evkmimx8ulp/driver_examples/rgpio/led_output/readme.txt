Overview
========
The RGPIO Example project is a demonstration program that uses the SDK software to manipulate the general-purpose
outputs.
The example is supported by the set, clear, and toggle write-only registers for each port output data register. The 
example take turns to shine the LED.


Toolchain supported
===================
- IAR embedded Workbench  9.10.2
- GCC ARM Embedded  10.2.1

Hardware requirements
=====================
- Micro USB cable
- MIMX8ULP-EVK  board
- J-Link Debug Probe
- 5V power supply
- Personal Computer

Board settings
==============
Pease make sure on-board RGB LED (U15) related GPIOE module is not initilazed by other CPU core.
That's mean, considering Single Boot mode case, since A35 BSP firmware must be executed and it will
configure xRDC to refuse M33 core access to GPIO E (which belongs to Application Domain), running
this example will cause hardfault.

So you may have to turn to LP Boot mode instead and make sure A35 BSP firmware will not run in that case.

Prepare the Demo
================
1.  Connect 5V power supply and JLink Plus to the board, switch SW10 to power on the board
2.  Connect a micro USB cable between the host PC and the J17 USB port on the target board.
3.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
When the example runs successfully, the following message is displayed in the terminal:
Since no LED connected to the RGPIO. Please use oscilloscope probe to the pin to check the output. The output will toggle periodically.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 RGPIO Driver example
 The LED is taking turns to shine.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
