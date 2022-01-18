.. _common-tbs-rc:

===========================
Team Black Sheep RC Systems
===========================

Any Crossfire compatible receiver can be used with ArduPilot. 

If you do not wish to use telemetry then a **TBS Crossfire** receiver can be connected to the **RCIN** port using :ref:`SBUS <common-rc-systems>`. You must configure the Receiver to output SBUS, of course.

CRSF Receivers 
==============

If you wish to use telemetry then a TBS receiver can be connected to a UART utilizing the `CRSF <https://www.team-blacksheep.com/products/prod:crossfire_tx>`__ protocol.

CRSF is a full-duplex protocol that supports integrated telemetry and a number of other features. Connect the RX pin of the UART to the CRSF TX pin of the CRSF device and vice versa.
Currently a full-duplex UART connection is required. For best performance a UART with DMA capability on its RX port is desirable, but not required. A message will be displayed once on the GCS console, if connected to a UART without this capability.

In the configuration of the serial port select the RCIN protocol. So for example for serial port 4:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 23
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3

With the receiver connected and configured correctly proceed with RC calibration as normal.

See :ref:`common-crsf-telemetry` for information about telemetry data sent, display scripts for OpenTX transmitters, and adjustment of ArduPilot parameters via CRSF.

.. warning:: If the autopilot is rebooted via MAVLink, it will lose communication with the CRSF receiver until the receiver is power cycled. Also, the CRSF TX must be transmitting BEFORE the receiver is powered up.

CRSF Video Transmitters
=======================

TBS Video Transmitters can also be connected to ArduPilot using the CRSF protocol. This allows the VTX to be configured via parameters and, for VTXs such as the `TBS Unify Evo <https://www.team-blacksheep.com/products/prod:tbs_unify_evo>`__ , receive telemetry data that can be displayed using the built-in OSD.

If you are using CRSF for RC input as well then nothing more needs to be done in order to enable VTX control.

If you only with to use CRSF for VTX control then connect the VTX to the UART in the normal way - TX to RX and RX to TX and configure the UART connection as follows: 

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 29
