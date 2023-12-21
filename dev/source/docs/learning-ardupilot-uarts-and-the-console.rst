.. _learning-ardupilot-uarts-and-the-console:

=====================
UARTs and the Console
=====================

A lot of components in ArduPilot rely on UARTs. They are used for debug
output, telemetry, GPS modules, and more. Understanding how to talk to
the UARTs via the HAL will help you understand a lot of ArduPilot code.

The 8 UARTs
===========

The ArduPilot HAL currently defines 8 UARTs. The HAL itself does not define any particular roles for these UARTs, but the other parts of ArduPilot assume they will be assigned particular functions. The command-line options for using with sim_vehicle.py the serial port should be preceded by :code:`-A` to pass along to the vehicle binary. Make sure to include the :code:`uart` protocol. Specifying a baudrate is not required, but is more consistent. For example, :code:`sim_vehicle.py --console --map -A --serial5=uart:/dev/ttyS15:115200`.

+-------------+----------------------+----------+-------------------------+
| ParamPrefix | Sim_vehicle Cmd Line | Def Role | Default Connection      |
+=============+======================+==========+=========================+
| \SERIAL0_   | \- -serial0=         | Console  | tcp:localhost:5760:wait |
+-------------+----------------------+----------+-------------------------+
| \SERIAL1_   | \- -serial1=         | MAVLink  | tcp:localhost:5762      |
+-------------+----------------------+----------+-------------------------+
| \SERIAL2_   | \- -serial2=         | MAVLink  | tcp:localhost:5763      |
+-------------+----------------------+----------+-------------------------+
| \SERIAL3_   | \- -serial3=         | GPS      | Simulated GPS           |
+-------------+----------------------+----------+-------------------------+
| \SERIAL4_   | \- -serial4=         | GPS      | Simulated GPS           |
+-------------+----------------------+----------+-------------------------+
| \SERIAL5_   | \- -serial5=         |          |                         |
+-------------+----------------------+----------+-------------------------+
| \SERIAL6_   | \- -serial6=         |          |                         |
+-------------+----------------------+----------+-------------------------+
| \SERIAL7_   | \- -serial7=         |          |                         |
+-------------+----------------------+----------+-------------------------+

If you are writing your own sketch using the ArduPilot HAL then you can
use these UARTs for any purpose you like, but if possible you should try
to use the above assignments as it will allow you to fit in more easily
to existing code.

You can change the role of UART by changing its SERIALn_PROTOCOL param. Possible parameter values are listed in the description for :ref:`SERIAL1_PROTOCOL<SERIAL1_PROTOCOL>`.

Go and have a look at the `libraries/AP_HAL/examples/UART_test <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/examples/UART_test/UART_test.cpp>`__
example sketch. It prints a hello message to the 1st 5 UARTs. Try it on your
board and see if you can get all the outputs displaying using a USB
serial adapter. Try changing the baudrate in the sketch.

Debug console
-------------

Historically, In addition to the basic 5 UARTs there was an additional debug console
available on some platforms. Recently debug console is directed to USB.
On SITL, debug is directed to a terminal, while USB is directed to port 5760 by default.

If you have a board that does have HAL_OS_POSIX_IO set (check that
in
`AP_HAL/AP_HAL_Boards.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/AP_HAL_Boards.h>`__)
then try adding some :code:`::printf()` and other stdio functions to the
UART_test sketch.

If :code:`::printf` doesn't work for you, it may be that your particular file (e.g. a library) does not have :code:`#include <stdio.h>` at the top of it, just add it. :-) 

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
