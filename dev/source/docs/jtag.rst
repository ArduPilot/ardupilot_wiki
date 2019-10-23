.. _jtag:

==============================
Archived: Debugging using JTAG
==============================

Overview
========

The JTAG interface on APM provides additional debugging features that
can be useful when working on certain kinds of problems. This page
describes how to configure your APM setup so that you can use JTAG to
debug it.

Note that enabling JTAG disables the ADC4, ADC5, ADC6 and ADC7 pins.
These aren't normally used by APM, but if your application requires more
than the low four ADC inputs, you won't be able to use JTAG.

Hardware Required
=================

Obviously you will need an APM board. The JTAG signals are available on
the oilpan, so we will assume you are using one as well. For other
boards (e.g. Arduino Mega) see your board's documentation for the
location of the relevant pins.

To program the JTAG and OCD fuses in the ATMega1280 an ISP programmer of
some sort is required. This article describes using the cheap and
effective !BusPirate, but another programmer (e.g. STK500, GoodFET,
etc.) could also be used.

Last but not least, you will need an AVR JTAG unit. The Atmel JTAGICE
mkii works well, but is expensive. This article was developed using the
much cheaper JTAGICE mkii-CN from

`http://www.mcuzone.com <http://www.mcuzone.com/>`__

obtained via eBay for about 1/3 the price of the Atmel unit. The JTAGICE
can be used as a programmer as well.

Note that the mkii-CN unit seems very sensitive to USB setup. If it
misbehaves when connected to a hub, try connecting it directly to a port
on your system.

Software Required
=================

The AVRDUDE and AVaRICE tools are required, as well as avr-gdb.

On Windows, all of the required tools should be installed by WinAVR.
(TBD: verify whether AVaRICE comes with WinAVR)

On Mac OS the use the MacPorts tool to install the tools. Fetch and
install the MacPorts package, then use the following commands:

::

    sudo port selfupdate
    sudo port install avrdude
    sudo port install avarice
    sudo port install avr-gdb

You can omit avr-gdb if you already have the AVR CrossPack installed.

Most Linux systems have packages available for avrdude, avarice and
avr-gdb. See your system management documentation for details on
installing these, or ask a friend.

Initial Setup
=============

Before debugging, the JTAG Enable and On Chip Debug Enable fuses must be
programmed in the ATMega1280. By default both are disabled, and despite
offering a tempting option in the boards.txt file, Arduino does not seem
to provide a way to change them.

Separate the APM and oilpan boards, and connect your ISP programmer to
the AT1280 SPI header on the APM board. In the illustration here, the
BusPirate breakout cable is being used. Note that the wire colours shown
are for the cable supplied by !SparkFun; other vendors may use different
colour-coding. Use the following table as a guide:

+------------------+-----------+---------------+
| !BusPirate Pin   | ISP Pin   | Wire Colour   |
+------------------+-----------+---------------+
| 1 (GND)          | 6         | black         |
+------------------+-----------+---------------+
| 3 (+5)           | 2         | grey          |
+------------------+-----------+---------------+
| 7 (CLK)          | 3         | yellow        |
+------------------+-----------+---------------+
| 8 (MOSI)         | 4         | orange        |
+------------------+-----------+---------------+
| 9 (CS)           | 5         | red           |
+------------------+-----------+---------------+
| 10 (MISO)        | 1         | brown         |
+------------------+-----------+---------------+

Programmer type should be BusPirate.

Here is a table of connections if you are using the JTAGICE mkII-CN
debugger as an ISP programmer:

+---------------+-----------+
| JTAGICE Pin   | ISP Pin   |
+---------------+-----------+
| TDO           | 1         |
+---------------+-----------+
| Vref          | 2         |
+---------------+-----------+
| TCK           | 3         |
+---------------+-----------+
| TDI           | 4         |
+---------------+-----------+
| SRST          | 5         |
+---------------+-----------+
| GND           | 6         |
+---------------+-----------+

Programmer type should be jtagmkii. NOTE: this configuration has not
been verified to work.

Use this AVRDUDE command to program the fuses on the ATMega1280 to
enable OCD and JTAG:

::

    avrdude -P 

    -c 

    -p m1280 -U hfuse:w:0x1a:m -v

Replace

with the port to which your programmer is connected, and

with the programmer type.

This should only take a few seconds, and you should see towards the end
the following lines output:

::

    avrdude: safemode: lfuse reads as FF
    avrdude: safemode: hfuse reads as 1A
    avrdude: safemode: efuse reads as F5
    avrdude: safemode: Fuses OK

These settings are persistent; unless you reflash the bootloader on your
APM JTAG will remain permanently enabled.

To revert to the default fuse settings, use:

::

    avrdude -P 

    -c 

    -p m1280 -U hfuse:w:0xda:m

JTAG Hardware Setup
===================

On the oilpan, add a 2x3 header to the Expansion Ports pads, and a 1x7
header to the analog pads (GND-AN5).

Connect your JTAG debugger as follows. Note that in the example here a
SparkFun !BusPirate breakout cable is being connected to a JTAGICE
mkii-CN. Your wire colours may vary; use the following table as a guide:

+------------+--------------+---------------+
| JTAG Pin   | Oilpan Pin   | Wire Colour   |
+------------+--------------+---------------+
| TCK        | ADC4         | brown         |
+------------+--------------+---------------+
| TMS        | ADC5         | green         |
+------------+--------------+---------------+
| TDO        | ADC6         | orange        |
+------------+--------------+---------------+
| TDI        | ADC7         | white         |
+------------+--------------+---------------+
| GND        | GND          | black, red    |
+------------+--------------+---------------+
| Vref       | +5           | yellow        |
+------------+--------------+---------------+
| SRST       | Reset        | blue          |
+------------+--------------+---------------+

Software Setup
==============

Once the hardware setup is done, there are a variety of ways to debug
using JTAG. The following steps cover just the basics.

The bridge between the JTAG unit and your debugger is AVaRICE. Some
setups will automatically launch the tool when the debugger starts,
others require that you launch it manually. It is critical for
successful APM debugging that the --capture argument is passed to
AVaRICE, rather than attempting to download a new program.

To start AVaRICE manually, assuming a JTAGICE mkii (or clone) connected
via USB, use:

::

    avarice --mkII --capture --jtag usb :4242

You should see output like:

::

    AVaRICE version 2.10, Dec 22 2010 21:38:18

Defaulting JTAG bitrate to 250 kHz.

JTAG config starting. Found a device: JTAGICEmkII Serial number:
00:a0:00:40:26:63 Reported JTAG device ID: 0x9703 Configured for device
ID: 0x9703 atmega1280 JTAG config complete. Waiting for connection on
port 4242.

Note that AVaRICE will exit every time the the debugger disconnects, so
be prepared to restart it regularly if your debugger doesn't manage it
already. Once the "Waiting for connection" message is displayed, you are
ready to connect with your debugger.

Debugging Code Built by Arduino
===============================

Arduino does its best to hide the dirty parts of the build process from
the user. This can make it slightly difficult to find the files that you
need for debugging. Hold down the SHIFT key while clicking the Verify
button in the Arduino IDE, and you will see the commands executed by
Arduino as it builds your sketch. Assuming that the build is successful,
the last command listed (before the "Binary sketch…" line) will look
something like this example taken from a Mac OS system:

::

    /Volumes/Data/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avr-objcopy -O ihex -R .eeprom 
    /var/folders/Bu/Burcn-0aFa4N+++0Me1I2U++1dw/-Tmp-/build159308262941509457.tmp/GPS_AUTO_test.cpp.elf
    /var/folders/Bu/Burcn-0aFa4N+++0Me1I2U++1dw/-Tmp-/build159308262941509457.tmp/GPS_AUTO_test.cpp.hex

The file that your debugger will want to load is the .elf file, in this
case

/var/folders/Bu/Burcn-0aFa4N+++0Me1I2U++1dw/-Tmp-/build159308262941509457.tmp/GPS_AUTO_test.cpp.elf

The name of the file will depend on your sketch name, and the location
will vary depending on your system, but it will remain the same for a
given sketch until you quit and restart Arduino. Note that the directory
containing the file is deleted and re-created every time you compile the
sketch.

To debug this file with avr-gdb, use Arduino to compile and upload the
sketch to APM. Start AVaRICE as described above, then start GDB:

::

    avr-gdb /var/folders/Bu/Burcn-0aFa4N+++0Me1I2U++1dw/-Tmp-/build159308262941509457.tmp/GPS_AUTO_test.cpp.elf

You should see GDB start up:

::

    GNU gdb 6.8
    Copyright (C) 2008 Free Software Foundation, Inc.
    License GPLv3+: GNU GPL version 3 or later 

    This is free software: you are free to change and redistribute it.
    There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
    and "show warranty" for details.
    This GDB was configured as "--host=i386-apple-darwin9.8.0 --target=avr"...
    (gdb)

At the (gdb) prompt, tell GDB to connect to AVaRICE:

::

    (gdb) target remote localhost:4242

This will stop the program running on the APM and tell you where it
stopped:

::

    Remote debugging using localhost:4242
    0x00002992 in AP_GPS_NMEA::read (this=0x8007aa)
    at /Volumes/Data/Users/msmith/work/Mike/ArduPilot/Sketchbook/libraries/AP_GPS/AP_GPS_NMEA.cpp:72
    72      numc = _port->
    available();
    (gdb)

Use the 'continue' command to start the program running again, and hit
control-C to stop it once more.

At this point, GDB can be used normally. Other debuggers that are
compatible with or build on GDB such as DDD or Insight can be used
instead, as can plugins for various IDEs like Eclipse.
