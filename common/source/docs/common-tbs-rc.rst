.. _common-tbs-rc:

===========================
Team Black Sheep RC Systems
===========================

Any Crossfire compatible receiver can be used with ArduPilot. 

If you do not wish to use telemetry then a **TBS Crossfire** receiver can be connected to the **RCIN** port using :ref:`SBUS <common-rc-systems>`

CRSF Receivers 
==============

If you wish to use telemetry then a TBS receiver can be connected to a UART utilizing the `CRSF <https://www.team-blacksheep.com/products/prod:crossfire_tx>`__ protocol.

CRSF is a full-duplex protocol that supports integrated telemetry and a number of other features. Connect the RX pin of the UART to the TX pin of the CRSF device and vice versa.
Currently a full-duplex UART connection is required.

In the configuration of the serial port select the RCIN protocol. So for example for serial port 4:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 23
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3

With the receiver connected and configured correctly proceed with RC calibration as normal.

Ardupilot supports two types of crossfire telemetry:

- CRSF standard telemetry (default)
- CRSF passthrough telemetry

CRSF standard telemetry
======================

Crossfire standard telemetry requires the CRSF protocol and exposes to OpenTX a wide range of telemetry data as discoverable sensors, please refer to the `TBS manual <https://www.team-blacksheep.com/tbs-crossfire-manual.pdf>`__ for more details.
Crossfire standard telemetry is the default telemetry type for the CRSF protocol and is enabled by the following parameters:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 23
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3

CRSF passthrough telemetry
=========================

Crossfire passthrough telemetry requires the use of the CRSF protocol and has to be manually enabled by setting the following parameters:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 23
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3
- Set :ref:`RC_OPTIONS <RC_OPTIONS>` += 256 (RC_OPTIONS is a bitmask, to enable set bit 8 "enable passthrough telemetry" to 1)

This configuration will send ArduPilot specific telemetry over the crossfire link using the :ref:`passthrough protocol <common-frsky-passthrough>` while retaining standard telemetry at a much lower refresh rate.
Telemetry sensors will be discoverable under OpenTX in the standard way but in order to take full advantage of ArduPilot's extended telemetry data a script such as the `Yaapu Telemetry Script or Widget <https://github.com/yaapu/FrskyTelemetryScript>`__ is required.
Passthrough telemetry over crossfire is functionally equivalent to passthrough telemetry over FRSky.

CRSF Video Transmitters
=======================

TBS Video Transmitters can also be connected to ArduPilot using the CRSF protocol. This allows the VTX to be configured via parameters and, for VTXs such as the `TBS Unify Evo <https://www.team-blacksheep.com/products/prod:tbs_unify_evo>`__ , receive telemetry data that can be displayed using the built-in OSD.

If you are using CRSF for RC input as well then nothing more needs to be done in order to enable VTX control.

If you only with to use CRSF for VTX control then connect the VTX to the UART in the normal way - TX to RX and RX to TX and configure the UART connection as follows: 

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 29


