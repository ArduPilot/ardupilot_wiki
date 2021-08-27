.. _common-msp-overview:

==============================
Multiwii Serial Protocol (MSP)
==============================

ArduPilot supports the MSP protocol for telemetry, osd and sensors via any of its serial ports. This allows ArduPilot to send its telemetry data to MSP compatible devices, such as DJI goggles, for On Screen Display (OSD) (See :ref:`common-msp-osd-overview`).

Protocol overview
-----------------

MSP is the main communication protocol used by all Betaflight derived flight stacks.
It's a binary message based protocol used for control, telemetry and sensors.
ArduPilot's MSP protocol module is ported from Betaflight and iNav, and supports both MSPV1 and MSPV2.

At the moment, the ArduPilot implementation supports:
 - MSP telemetry
 - MSP OSDs such as DJI FPV Goggles and FatShark's ByteFrost and SharkByte
 - MSP Sensors such as lidar, optical flow, gps, barometer, magnetometer and airspeed
 - MSP Displayport OSDs such as FatShark's Shark Byte and MWOSD

ArduPilot configuration via MSP protocol is not supported, so for instance the Betaflight configurator won't work.

Configuration
-------------

MSP requires a free serial port, and its speed defaults to 115200 baud.

There are 3 MSP backends selected by serial protocol:
 - SERIAL_PROTOCOL = 33 is for DJI FPV or RE Goggles
 - SERIAL_PROTOCOL = 32 is for sensors or generic MSP telemetry usage
 - SERIAL_PROTOCOL = 42 is for DisplayPort OSDs (aka CANVAS MODE) such as FatShark's SharkByte

.. note:: There's a dedicated backend for DJI to try to maintain compatibility with DJI's evolving hardware.

When SERIAL_PROTOCOL = 33 is selected the protocol decoder can work in polling mode (default) or in "telemetry push" mode.
When working in polling mode, both TX and RX must be connected to the MSP telemetry transceiver. While in push mode only the TX line is used. To enable push mode, simply set MSP_OPTIONS bit 0 to "1"; to disable set it to "0" (default).

MSP sensors such as the Matek 3901-L0X are supported by both backends.


