Overview
========
The Secure GPIO demo application demonstrates using of TRDC and GPIO PIN control.
TRDC allows user to control GPIO peripheral into secure world or into normal world.
The Pin Control Non-Secure Register (PCNS) configures secure/non-secure access protection for each pin.
Thus for all pins, user can select whether the pin is controlled from secure or non-secure domain.

This Secure GPIO demo uses GPIO peripheral to read SW7 button from secure world and normal world.
If SW7 button is pressed (logical zero is read) the RGPIOA PIN15 is pulled high in secure world
and concurrently the RGPIOA PIN18 is low from normal world.

The second part of the demo is GPIO PIN control Non-secure access protection feature. This feature is controlled by
button SW8. If the SW8 button is released, The SW7 is only allowed by software in secure world. SW7 is read zero
(also is zero When SW7 pressed) in normal world. If the SW8 button is pressed, the SW7 is for normal world. Thus normal
world can read state of SW7 button. The RGPIO PIN18 is pulled high while SW7 is released. Meanwhile the SW7 is pressed
PIN18 is low and PIN15 is pulled high.

Toolchain supported
===================
- GCC ARM Embedded  10.2.1
- IAR embedded Workbench  9.10.2

Hardware requirements
=====================
- Micro USB cable
- MIMX8ULP-EVK board
- Personal Computer

Board settings
==============

Prepare the Demo
================
1.  Connect a micro USB cable between the PC host and the CMSIS DAP USB port (J5) on the board
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Use secure project to download the program to target board. Please refer to "TrustZone application debugging" below for details.
4.  Launch the debugger in your IDE to begin running the demo.
Note: Refering to the "Getting started with MCUXpresso SDK for MIMX8ULP-EVK" documentation for more information
      on how to build and run the TrustZone examples in various IDEs.

Running the demo by gdb on Windows
==================================
1.  Reset board.
2.  Run JLinkGDBServer into install directory.
    .\JLinkGDBServer.exe -jlinkscriptfile Devices\NXP\iMX8ULP\NXP_iMX8ULP_Connect_CortexM33.JLinkScript -device MIMX8UD7_M33 -if SWD
3.  Startup gdb.
    arm-none-eabi-gdb.exe
4.  Connect remote target(MIMX8ULP-EVK board) in gdb.
    (gdb)target remote :2331
5.  First load non-secure app then load.
    (gdb) monitor reset
    (gdb) monitor halt
    (gdb) load {app}_ns.elf
    (gdb) load {app}_s.elf
6.  Set sp value that come from interrupt lma addr of {app}_s.elf
    (gdb) load {app}_s.elf
         Loading section .interrupts, size 0x300 lma 0x14002000
         Loading section .resource_table, size 0x10 lma 0x14002300
         Loading section .text, size 0x6a48 lma 0x14002310
         Loading section .ARM, size 0x8 lma 0x14008d58
         Loading section .init_array, size 0x4 lma 0x14008d60
         Loading section .fini_array, size 0x4 lma 0x14008d64
         Loading section .data, size 0x2e4 lma 0x14008d68
         Loading section .gnu.sgstubs, size 0x20 lma 0x140ffe00
         Start address 0x140023cc, load size 28780
         Transfer rate: 573 KB/sec, 3197 bytes/write.
    (gdb) monitor reg sp=(0x14002000)
7. Then run this demo.
    (gdb) monitor go

Running the demo
================
Use S7 button to control output of GPIOA PIN15 and S8 to control GPIO mask feature.
If S7 button is pressed (logical zero is read), the GPIOA PIN15 is pulled high in secure world
If S8 button is pressed, the GPIO mask feature is disabled. In this case the GPIOA PIN18 is pulled high, 
if also S7 button is pressed, the GPIOA PIN18 is pulled low.

TrustZone Application Development in SDK
----------------------------------------
Every TrustZone based application consists of two independent parts - secure part/project and non-secure part/project.

The secure project is stored in <application_name>\<application_name>_s directory.
The non-secure project is stored in <application_name>\<application_name>_ns directory. 

The secure projects always contains TrustZone configuration and it is executed after device RESET. The secure project usually
ends by jump to non-secure application/project.
If IDE allows the two projects in single workspace, the user can also find the project with <application_name>.
This project contains both secure and non-secure projects in one workspace (Keil MDK, IAR) and it allows to user easy transition from
one to another project.

Project Structure
-----------------
The all TrustZone files related to TrustZone are located in trustzone virtual directory. The files are:

- tzm_config.c
- tzm_config.h
- veneer_table.c
- veneer_table.h

File tzm_config.c, tzm_config.h
-------------------------------
This file is used by secure project only. It contains one function BOARD_InitTrustZone(), which configures complete TrustZone
environment. It includes SAU, MPU's, AHB secure controller and some TrustZone related registers from System Control Block.
This function is called from SystemInitHook() function, it means during system initialization.

File veneer_table.c, veneer_table.h
----------------------------------
This file defines all secure functions (secure entry functions) exported to normal world. This file is located in secure
project only. While header file is used by both secure and non-secure projects. The secure entry functions usually contain
validation of all input parameters in order to avoid any data leak from secure world.

The files veneer_table.h and <application_name>_s_import_lib.o or <application_name>_s_CMSE_lib.o create the connection
between secure and non-secure projects. The library file is generated by linker during compilation of secure project and
it is linked to the non-secure project as any other library.

TrustZone application compilation
---------------------------------
Please compile secure project firstly since CMSE library is needed for compilation of non-secure project.
After successful compilation of secure project, compile non-secure project.

TrustZone application debugging
-------------------------------
- Download both output file into device memory
- Start execution of secure project since secure project is going to be executed after device RESET.
