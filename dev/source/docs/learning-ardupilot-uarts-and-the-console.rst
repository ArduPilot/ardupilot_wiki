.. _learning-ardupilot-uarts-and-the-console:

=====================
UARTs and the Console
=====================

A lot of components in ArduPilot rely on UARTs. They are used for debug
output, telemetry, GPS modules and more. Understanding how to talk to
the UARTs via the HAL will help you understand a lot of ArduPilot code.

The 5 UARTs
===========

The ArduPilot HAL currently defines 5 UARTs. The HAL itself doesn't
define any particular roles for these UARTs, but the other parts of
ArduPilot assume they will be assigned particular functions

-  uartA - the console (usually USB, runs MAVLink telemetry)
-  uartB - the first GPS
-  uartC - primary telemetry (telem1 on Pixhawk, 2nd radio on APM2)
-  uartD - secondary telemetry (telem2 on Pixhawk)
-  uartE - 2nd GPS

If you are writing your own sketch using the ArduPilot HAL then you can
use these UARTs for any purpose you like, but if possible you should try
to use the above assignments as it will allow you to fit in more easily
to existing code.

Some UARTs have dual roles. For example there is a parameter
SERIAL2_PROTOCOL changes uartD from being used for MAVLink versus being
used for Frsky telemetry.

Go and have a look at the
`libraries/AP_HAL/examples/UART_test <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/examples/UART_test/UART_test.cpp>`__
example sketch. It prints a hello message to all 5 UARTs. Try it on your
board and see if you can get all the outputs displaying using a USB
serial adapter. Try changing the baudrate in the sketch.

Debug console
-------------

In addition to these 5 UARTs there is an additional debug console
available on some platforms. You can tell if the platform you are on has
a debug console by checking for the HAL_OS_POSIX_IO macro, like
this:

::

    #if HAL_OS_POSIX_IO
      ::printf("hello console\n");
    #endif

If you have a board that does have HAL_OS_POSIX_IO set (check that
in
`AP_HAL/AP_HAL_Boards.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/AP_HAL_Boards.h>`__)
then try adding some ::printf() and other stdio functions to the
UART_test sketch. 

If ::printf doesn't work for you, it may be that your particular file ( eg a library ) does not have "#include <stdio.h>" at the top of it, just add it. :-) 

Note that on some boards (eg. the Pixhawk) hal.console->printf() goes to
a different place to ::printf(). On the Pixhawk a hal.console->printf()
goes to the USB port whereas ::printf() goes to the dedicated debug
console (which also runs the nsh shell for NuttX access).

UART Functions
==============

Every UART has a number of basis IO functions available. The key
functions are:

-  printf - formatted print
-  printf_P - formatted print with progmem string (saves memory on AVR
   boards)
-  println - print and line feed
-  write - write a bunch of bytes
-  read - read some bytes
-  available - check if any bytes are waiting
-  txspace - check how much outgoing buffer space is available
-  get_flow_control - check if the UART has flow control capabilities

Go and have a look at the declarations of each of these in AP_HAL and
try them in UART_test.
