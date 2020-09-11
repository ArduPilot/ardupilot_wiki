.. _common-msp-overview:

==============================
Multiwii Serial Protocol (MSP)
==============================

ArduPilot supports the MSP protocol for telemetry and sensors via any of its serial ports. This allows ArduPilot to send its telemetry data to MSP compatible devices, such as DJI goggles, for On Screen Display (OSD).

Protocol overview
-----------------

MSP is the main communication protocol used by all Betaflight derived flight stacks.
It's a binary message based protocol used for control, telemetry and sensors.
ArduPilot's MSP protocol module is ported from Betaflight and iNav, and supports both MSPV1 and MSPV2. It also supports iNav's MSPV2 sensor messages for:

- lidar
- optical flow
- gps
- barometer (future)
- magnetometer (future)
- airspeed (future)

At the moment, the ArduPilot implementation supports only telemetry and sensor messages. So currently ArduPilot works with DJI FPV goggles, but can't be controlled by the Betaflight configurator.


Configuration
-------------

MSP requires a free serial port, and its speed defaults to 115200 baud.

There are 2 MSP backends selected by serial protocol:
 - SERIAL_PROTOCOL = 33 is for DJI FPV or RE Goggles
 - SERIAL_PROTOCOL = 32 is for sensors or generic MSP telemetry usage

.. note:: There are 2 different backends to try to maintain compatibility with DJI's evolving hardware.

When SERIAL_PROTOCOL = 33 is selected the protocol decoder can work in polling mode (default) or in "telemetry push" mode.
When working in polling mode, both TX and RX must be connected to the MSP telemetry transceiver. While in push mode only the TX line is used. To enable push mode, simply set MSP_OPTIONS bit 0 to "1"; to disable set it to "0" (default).

MSP sensors such as the Matek 3901-L0X are supported by both backends.


