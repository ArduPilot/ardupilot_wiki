.. _common-tbs-rc:

===========================
Team Black Sheep RC Systems
===========================

Any Crossfire compatible receiver can be used with ArduPilot. 

If you do not wish to use telemetry then a **TBS Crossfire** receiver can be connected to the **RCIN** port using :ref:`SBUS <common-rc-systems>`

CRSF Receivers 
==============

If you wish to use telemetry then a TBS receiver can be connected to a UART utilizing the `CRSF <https://www.team-blacksheep.com/products/prod:crossfire_tx>`__ protocol.

CRSF is a single-wire, half-duplex protocol that supports integrated telemetry and a number of other features. 

In the configuration of the serial port select the RCIN protocol and set the serial options to half-duplex operation. So for example for serial port 4:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 23
- Set :ref:`SERIAL4_OPTIONS <SERIAL4_OPTIONS>` = 4
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3

Many single board flight controllers support RC input on serial port 6 only exposing the RX connection. For F7 flight controllers, such as the Kakute F7, it is necessary to swap the RX and TX pins in order to support CRSF. It is also necessary to support serial access on the UART by using the board's alternate configuration:

- Set :ref:`SERIAL6_PROTOCOL <SERIAL6_PROTOCOL>` = 23
- Set :ref:`SERIAL6_OPTIONS <SERIAL6_OPTIONS>` = 12
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3
- Set :ref:`BRD_ALT_CONFIG <BRD_ALT_CONFIG>` = 1

With the receiver connected and configured correctly proceed with RC calibration as normal.

CRSF Video Transmitters
=======================

TBS Video Transmitters can also be connected to ArduPilot using the CRSF protocol. This allows the VTX to be configured via parameters and, for VTXs such as the `TBS Unify Evo <https://www.team-blacksheep.com/products/prod:tbs_unify_evo>`__ , receive telemetry data that can be displayed using the built-in OSD.

If you are using CRSF for RC input as well then nothing more needs to be done in order to enable VTX control.

If you only with to use CRSF for VTX control then connect the VTX to the UART in the normal way - TX to RX and RX to TX and configure the UART connection as follows: 

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 29


