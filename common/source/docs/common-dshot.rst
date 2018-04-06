.. _common-dshot:

============================
DShot and BLHeli ESC Support
============================

..  youtube:: np7xXY_e5sA
    :width: 100%

This articles describes how to setup and use three features supported
by recenet BLHeli ESC firmwares.

- DShot ESC protocol support
- BLHeli pass-thru configuration and ESC flashing
- ESC telemetry support

.. note::

   These feature are available only in the ChibiOS firmware
   builds on STM32 based flight boards. As of April 2018 please join the
   http://gitter.im/ArduPilot/ChibiOS gitter channel for up to date
   information on these builds

DShot Protocol
==============

The DShot ESC protocol is a digital protocol for communication between
a flight board and an ESC. The key advantages are:

- all values sent to the ESC are protected with a 4 bit CRC
- clock differences between the ESC and flight controller don't affect
  flight
- no need to do any ESC throttle range calibration
- very high protocol frame frames are supported

The DShot protocol can run at several difference speeds. ArduPilot
supports four speeds:

- DShot150 at 150kbaud
- DShot300 at 300kbaud
- DShot600 at 600kbaud
- DShot1200 at 1200kbaud

As ArduPilot currently runs quite slow loop rates (maximum of 1kHz
currently), we recommend using the lowest baud rate DShot150 protocol,
as it is the most reliable protocol (lower baudrates are less
susceptible to noise on cables).

DShot sends 16 bits per frame, with bits allocated as follows:

- 11 bits for the throttle level
- 1 bit for telemetry request
- 4 bits for CRC (simple XOR)

This gives a good throttle resolution, with support for asking the ESC
to provide telemetry feedback. See below for more information on ESC
telemetry.

To enable DShot support in ArduPilot you should first check that your
ESC supports it. Do not enable DShot on ESCs that don't support it or
you will get unpredictable results.

Then you need to set the "Motor PWM Type" to one of the 4 DShot
protocol varients. On multi-copters you should set the parameter
MOT_PWM_TYPE to a value from 4 to 7. The value 4 corresponds to
DShot150.

On a QuadPlane you should set the Q_M_PWM_TYPE to the motor output
type, with a value of 4 for DShot150.

We do not currently support DShot output on other vehicle types.

.. note::

   DShot output is currently only supported on the "FMU" outputs of
   your flight controller. If you have a board with an IO
   microcontroller, with separate "main" and "auxillary" outputs, such
   as a Pixhawk1 or Pixhawk2 board then you can only use DShot on the
   "auxillary" outputs. You will need to use the SERVOn_FUNCTION
   parameters to remap your motors to the auxillary outputs.
   

BLHeli Pass-Through Support
===========================

BLHeli pass-through support is a feature that allows you to configure
and upgrade the firmware on your ESCs without having to disconnect
them from your vehicle. You can plug a USB cable into your flight
controller and run the BLHeliSuite software for Windows to configure
your ESCs.

Note that you do not have to be using DShot to take advantage of
BLHeli pass-through support, although it is recommended that you do.

To enable BLHeli pass-through support you need to set one of two
variables:

- SERVO_BLH_AUTO=1 to enable automatic mapping of motors to
  BLHeliSuite ESC numbers
- SERVO_BLH_MASK if you want to instead specify a specific set of
  servo outputs to enable

For most users setting SERVO_BLH_AUTO=1 will do the right thing. The
alternative SERVO_BLH_MASK is for more complex setups where you want
to choose exactly which servo outputs you want to configure.

Once you have enabled BLHeli support with one of the above two
parameters you should reboot your flight board.

Now connect a USB cable to your flight board and use BLHeliSuite on
Windows to connect. You will need to use BLHeliSuite32 for BLHeli_32
ESCs, and BLHeliSuite16 for older BLHeli_S ESCs.

ESC Telemetry
=============

You can also enable ESC telemetry feedback, allowing you to log the
following variables from each ESC in flight:

- RPM
- Voltage
- current
- temperature
- total-current

To use ESC telemetry you need to connect a separate telemetry pin on
all your ESCs back to a single UART RX pin on your flight board. ESC
telemetry is only available on BLHeli_32 ESCs, and a wire for the
telemetry is only pre-soldered for some ESCs. If the wire isn't
pre-soldered you will need to solder it yourself.

The wires from all ESCs should all come back to a single UART RX
line. The way it works is that the flight board requests telemetry
from only one ESC at a time, cycling between them.

You can use any of the UARTs on your flight board for telemetry
feedback. You need to enable it using the SERIALn_PROTOCOL option for
the UART you are using. For example, on a PH2.1 if you wanted to use
the Serial5 UART you would set SERIAL5_PROTOCOL=16 (where 16 is the
value for "ESC Telemetry").

You also need to set the telemetry rate in the SERVO_BLH_TRATE
parameter. This rate is the rate in Hz per ESC. So if you set it to 10
then you will get 10Hz data for all ESCs.

The data is logged in the ESCn log messages in your dataflash
log. This can be viewed in any ArduPilot dataflash log viewer.
