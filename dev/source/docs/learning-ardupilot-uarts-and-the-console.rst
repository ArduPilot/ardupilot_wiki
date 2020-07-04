.. _learning-ardupilot-uarts-and-the-console:

=====================
UARTs and the Console
=====================

A lot of components in ArduPilot rely on UARTs. They are used for debug
output, telemetry, GPS modules and more. Understanding how to talk to
the UARTs via the HAL will help you understand a lot of ArduPilot code.

The 8 UARTs
===========

The ArduPilot HAL currently defines 8 UARTs. The HAL itself doesn't
define any particular roles for these UARTs, but the other parts of
ArduPilot assume they will be assigned particular functions

-  uartA - the console (usually USB, runs MAVLink telemetry)
-  uartB - the first GPS
-  uartC - primary telemetry (telem1 on most autopilots)
-  uartD - secondary telemetry (telem2 on most autopilots)
-  uartE - 2nd GPS
-  uartF - User Configurable
-  uartG - User Configurable
-  uartH - User Configurable

See also :ref:`sitl-serial-mapping`

If you are writing your own sketch using the ArduPilot HAL then you can
use these UARTs for any purpose you like, but if possible you should try
to use the above assignments as it will allow you to fit in more easily
to existing code.

You can change the role of UART by changing its SERIALn_PROTOCOL param. Possible parameter values are 
-1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Rangefinder, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 13:Beacon, 14:Volz servo out, 15:SBus servo out, 16:ESC Telemetry, 17:Devo Telemetry, 18:OpticalFlow, 19:RobotisServo, 20:NMEA Output, 21:WindVane, 22:SLCAN
Search code for 1_PROTOCOL to get updated list of uart roles.

Go and have a look at the
`libraries/AP_HAL/examples/UART_test <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/examples/UART_test/UART_test.cpp>`__
example sketch. It prints a hello message to the 1st 5 UARTs. Try it on your
board and see if you can get all the outputs displaying using a USB
serial adapter. Try changing the baudrate in the sketch.

Debug console
-------------

Historically, In addition to the basic 5 UARTs there was an additional debug console
available on some platforms. Recently debug console is directed to USB.
On SITL debug is directed to terminal running SITL, while USB is directed to port 5760 by default.

If you have a board that does have HAL_OS_POSIX_IO set (check that
in
`AP_HAL/AP_HAL_Boards.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/AP_HAL_Boards.h>`__)
then try adding some ::printf() and other stdio functions to the
UART_test sketch.

If ::printf doesn't work for you, it may be that your particular file ( eg a library ) does not have "#include <stdio.h>" at the top of it, just add it. :-) 

You can also use hal.console->printf() to specify USB port.

UART Functions
==============

Every UART has a number of basic IO functions available. The key
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
