.. _debugging-with-gdb-on-stm32:

===========================
Debugging with GDB on STM32
===========================

This page describes how to setup GDB on Linux to debug issues on STM32. The specific commands were tested on Ubuntu 18.10.

Introduction
============

GDB (the GNU Debugger) "allows you to see what is going on \`inside'
another program while it executes or what another program was doing at
the moment it crashed." which can be useful when investigating very
low-level failures with the STM32 (it cannot be used with the
APM1/APM2)

This guide assumes that you have already successfully built the firmware
on your machine following the instructions for
:ref:`Windows <building-setup-windows>`,
:ref:`Mac <building-setup-mac>` or
:ref:`Linux <building-setup-linux>`.

A `BlackMagic probe <http://www.blacksphere.co.nz/main/index.php/blackmagic>`__ or `ST-Link V2 JST debugger <https://www.ebay.com/itm/ST-Link-V2-Stlink-Emulator-Downloader-Programming-Mini-Unit-STM8-STM32-KK/223056820813>`__ is
also required.  A BlackMagic probe can be purchased in the US from `Transition Robotics <http://transition-robotics.com/products/black-magic-probe-mini>`__,
`1 Bit Squared <http://1bitsquared.com/collections/frontpage/products/black-magic-probe>`__
or in NewZealand from
`Greenstage <http://shop.greenstage.co.nz/product/black-magic-debug-probe>`__.

A small plug (`FTSH-105-04-L-DV <https://au.element14.com/samtec/ftsh-105-04-l-dv/connector-header-1-27mm-smt-10way/dp/2308438?scope=partnumberlookahead&ost=FTSH-105-04-L-DV&searchref=searchlookahead&exaMfpn=true&ddkey=https%3Aen-AU%2FElement14_Australia%2Fw%2Fsearch>`__) may be required to solder on to the JTAG port of the flight controller.


Compiling the Firmware
======================

Build and upload the debug variant of the firmware:

```
./waf configure --board=Pixhawk1-1M --debug --enable-asserts
./waf --target=examples/UART_test --upload
```

Connecting the BlackMagic probe to the Pixhawk
==============================================

.. image:: ../images/DebuggingWithGDB_PixhawkBlackMagicProbe.jpg
    :target: ../_images/DebuggingWithGDB_PixhawkBlackMagicProbe.jpg

The BlackMagic probe should be connected to the Pixhawk's JTAG connector
using the grey 10wire cable that came with the probe. Note that most
Pixhawk come with no headers soldered onto the JTAG connector because it
interferes with the case.

Connecting the Segger J-Link EDU to CubeRed
===========================================

.. image:: ../images/CubeRedJLinkEDU.jpg
    :target: ../_images/CubeRedJLinkEDU.jpg

The J-Link EDU is connected to the small 6wire connector on the bottom of Cube Red.

Connecting the ST-Link V2 JTAG debugger to the STM32
====================================================

.. image:: ../images/stlink-pixhawk-debugger.jpeg
    :target: ../_images/stlink-pixhawk-debugger.jpeg

The ST-Link debugger should be connected to the STM32's JTAG connector
using the JTAG breakout cable. Connect the
SWDIO and SWCLK (see `Debug port pinout <https://ardupilot.org/copter/docs/common-pixracer-overview.html#debug-port-jst-sm06b-connector>`__
for reference) to the corresponding SWDIO and SWCLK pins on the ST-Link debugger.

Ensure :ref:`BRD_OPTIONS<BRD_OPTIONS>` sets the "Enable Debug Pins" bit if bit 3 is present in the vehicle software being used.
It is also advised to disable the watchdog by ensuring bit 0 is not set.

Once BRD_OPTIONS bit 3 is set, the board will not boot until `gdb` tells it to start.
If you try using mavproxy, it will log "no link"

Connecting the ST-LINK V3 JTAG debugger to the STM32
====================================================

Install drivers. For Ubuntu x86_64:
```
wget https://github.com/stlink-org/stlink/releases/download/v1.7.0/stlink_1.7.0-1_amd64.deb
sudo apt install ./stlink_1.*.deb
rm ./stlink_1.*.deb
```


Connecting the Segger J-Link EDU to the STM32 on Cube Red
=========================================================

Install drivers for Segger. https://www.segger.com/downloads/jlink/

Install drivers. For Ubuntu x86_64:
```
wget https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb
sudo apt install ./JLink_Linux_x86_64.deb
rm ./JLink_Linux_x86_64.deb
```

Next, update `RTOSPlugin_ChibiOS.so`. TODO add instructions.
Place the file in `/opt/SEGGER/JLink/GDBServer/`.


With CubeRed wired up to both the debugger, and connected over USB-C to power it, test the connection. 

For CubeRed:

```bash
JLinkGDBServerCLExe -select USB -device STM32H743ZI -endian little -if SWD -speed auto -noir -noLocalhostOnly -nologtofile -port 2331 -rtos /opt/SEGGER/JLink/GDBServer/RTOSPlugin_ChibiOS
SEGGER J-Link GDB Server V7.94 Command Line Version

JLinkARM.dll V7.94 (DLL compiled Nov 29 2023 13:40:58)

Command line: -select USB -device STM32H743ZI -endian little -if SWD -speed auto -noir -noLocalhostOnly -nologtofile -port 2331 -rtos /opt/SEGGER/JLink/GDBServer/RTOSPlugin_ChibiOS
-----GDB Server start settings-----
GDBInit file:                  none
GDB Server Listening port:     2331
SWO raw output listening port: 2332
Terminal I/O port:             2333
Accept remote connection:      yes
Generate logfile:              off
Verify download:               off
Init regs on start:            off
Silent mode:                   off
Single run mode:               off
Target connection timeout:     0 ms
------J-Link related settings------
J-Link Host interface:         USB
J-Link script:                 none
J-Link settings file:          none
------Target related settings------
Target device:                 STM32H743ZI
Target device parameters:      none
Target interface:              SWD
Target interface speed:        auto
Target endian:                 little

Connecting to J-Link...
J-Link is connected.
Firmware: J-Link EDU Mini V1 compiled Nov 22 2023 09:50:48
Hardware: V1.00
S/N: 801026166
Feature(s): FlashBP, GDB
Checking target voltage...
Target voltage: 3.30 V
Listening on TCP/IP port 2331
Connecting to target...
Halting core...
Connected to target
Waiting for GDB connection...
```

If it prompts you for a firmware update for the connected emulator, select "Yes". Once the terminal output of the JLink GDB server says "Waiting for GDB connection...", it's ready. 
Leave it running.

In another terminal, configure with debug and asserts, flash, and run the binary. 
```
./waf configure --board CubeRedPrimary --debug --enable-asserts
./waf plane --upload
```

Installing GDB
==============

If using Ubuntu, GDB is likely already installed on your machine and it
will likely work although we recommend using the version available for
download here `https://firmware.ardupilot.org/Tools/STM32-tools <https://firmware.ardupilot.org/Tools/STM32-tools>`__

The gcc-arm-none-eabi*-linux.tar.bz2 file contains both the
recommended compiler and the recommended version of gdb.

If you will be using the ST-Link debugger, please install the gdb version found
in the gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2 file.

After installation you should find you have a tool called
arm-none-eabi-gdb.

Installing OpenOCD with stlink support
==================

You will need to install OpenOCD if you are using the ST-Link debugger.

As of November 2023, release 0.12.0 does not include the necessary ChibiOS support.
Until then, compile OpenOCD from source.

```
git clone --recurse-submodules git@github.com:openocd-org/openocd.git
cd openocd
./configure --enable-stlink
make
make install
```

Check the install was successful
```
openocd --version
Open On-Chip Debugger 0.12.0+dev-g9fcf33da8 (2023-11-15-23:04)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
```

Once the next version is released, you could install openocd with apt.
``sudo apt-get install openocd``.

When you run OpenOCD, supply it with a config file.
For example, on an H7 board like CubeRed:
```bash
./waf configure --board CubeRedPrimary --debug --enable-asserts
openocd --file Tools/debug/openocd-h7.cfg
```

Type ``openocd`` in your terminal in the ``bin`` directory above.

.. image:: ../images/openocd.png
    :target: ../_images/openocd.png

Note: there are 2 versions of the ST-link debugger on eBay, so if
the command does not work, change the first line to ``source [find interface/stlink-v2-1.cfg]``.


Note: Plugging a Cube Red into your computer with a debugger, without OpenOCD running, can potentially interfere with keyboard functionality.
If that happens, unplug the Cube, start OpenOCD, and plug the cube back in.

Starting GDB and running some commands
======================================

Copy ``.gdbinit`` to ``./build/<boardname>/bin`` and
edit ``~/.gdbinit`` to have the following text: ``set auto-load safe-path /``,
if not done so previously.

In another window, type ``arm-none-eabi-gdb arducopter`` in the 
``./build/<boardname>/bin`` folder. Now you have
connected to the gdb debugging session and can use the commands from
the next section.

If using J-Link, send ``target extended-remote :2331`` to connect to JLink.

.. image:: ../images/DebuggingWithGDB-startGBD.png
    :target: ../_images/DebuggingWithGDB-startGBD.png

Some useful commands:

``layout src`` -- Show source code in parallel

``r`` -- restarts the process

``b function-name`` -- i.e. b setup -- sets a breakpoint at the start of
the "setup" function. Note a class name can be prepended such as
``b AC_AttitudeControl::init``

``Ctrl-C`` -- stops the code from executing so you can set breakpoints,
etc

``continue`` -- continues the code from wherever it was stopped

``show interrupted-thread`` -- shows address where execution has stopped
(see below)

``info line * <address>`` -- shows c++ line for a given address (i.e.
from show interrupted-thread)

``info threads`` -- show status of all threads

``disassemble <address>`` -- converts given address into assembler code

``exit`` -- exits from the GDB

.. image:: ../images/GDB_commands2.jpg
    :target: ../_images/GDB_commands2.jpg
