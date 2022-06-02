Overview
========
The Hello World demo application provides a sanity check for the new SDK build environments and board bring up. This demo application also utilizes TrustZone, 
so it demonstrates following techniques for TrustZone applications development:
1. Application separation between secure and non-secure part
2. TrustZone environment configuration
3. Exporting secure function to non-secure world
4. Calling non-secure function from secure world
4. Creating veneer table
5. Configuring IAR, MDK, GCC and MCUX environments for TrustZone based projects

Toolchain supported
===================
- GCC ARM Embedded  10.2.1
- IAR embedded Workbench  9.10.2

Hardware requirements
=====================
- Micro USB cable
- MIMX8ULP-EVK board
- J-Link Debug Probe
- 5V power supply
- Personal Computer

Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect 5V power supply and J-Link Debug Probe to the board, switch SW1 to power on the board.
2.  Connect a micro USB cable between the host PC and the J6 USB port on the target board.
3.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board by IAR or GDB.
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
The log below shows the output of the hello world demo in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Hello from secure world!
Entering normal world.
Welcome in normal world!
This is a text printed from normal world!
Comparing two string as a callback to normal world
String 1: Test1
String 2: Test2
Both strings are not equal!
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


TrustZone Application Development in SDK
Every TrustZone based application consists of two independent parts - secure part/project and non-secure part/project.

The secure project is stored in <application_name>\<application_name>_s directory.
The non-secure project is stored in <application_name>\<application_name>_ns directory.

The secure projects always contains TrustZone configuration and it is executed after device RESET. The secure project usually
ends by jump to non-secure application/project.
If IDE allows the two projects in single workspace, the user can also find the project with <application_name>.
This project contains both secure and non-secure projects in one workspace (Keil MDK, IAR) and it allows to user easy transition from
one to another project.

Project Structure
The all TrustZone files related to TrustZone are located in trustzone virtual directory. The files are:

- tzm_config.c
- tzm_config.h
- veneer_table.c
- veneer_table.h

File tzm_config.c, tzm_config.h
This file is used by secure project only. It contains one function BOARD_InitTrustZone(), which configures complete TrustZone
environment. It includes SAU, MPU's, TRDC secure controller and some TrustZone related registers from System Control Block.
This function is called from SystemInitHook() function, it means during system initialization.

File veneer_table.c, veneer_table.h
This file defines all secure functions (secure entry functions) exported to normal world. This file is located in secure
project only. While header file is used by both secure and non-secure projects. The secure entry functions usually contain
validation of all input parameters in order to avoid any data leak from secure world.

The files veneer_table.h and <application_name>_s_import_lib.o or <application_name>_s_CMSE_lib.o create the connection
between secure and non-secure projects. The library file is generated by linker during compilation of secure project and
it is linked to the non-secure project as any other library.

TrustZone application compilation
As first compile secure project since CMSE library is needed for compilation of non-secure project.
After successful compilation of secure project, compile non-secure project.

TrustZone application debugging
- Download both output file into device memory
- Start execution of secure project since secure project is going to be executed after device RESET.

If IDE (IAR) allows to manage download both output files as single download, the secure project
is configured to download both secure and non-secure output files so debugging can be fully managed
from secure project.

Device header file and secure/non-secure access to the peripherals
Both secure and non-secure project uses identical device header file. The access to secure and non-secure aliases for all peripherals
is managed using compiler macro __ARM_FEATURE_CMSE.
