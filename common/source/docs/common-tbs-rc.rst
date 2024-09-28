.. _common-tbs-rc:
[copywiki destination="plane,copter,rover,blimp"]
=============================
Crossfire and ELRS RC Systems
=============================

Any Crossfire/ELRS compatible receiver can be used with ArduPilot. 

.. note::  ELRS (ExpressLRS) RC systems use the CRSF protocol and are connected in a similar manner as Crossfire receivers to an autopilot UART. ELRS can also be configured to be MAVLink protocol with embedded RC control (See below). On  F4/F7 based autopilots the UART MUST be DMA enabled for reliable operation in CRSF protocol mode. Consult :ref:`common-autopilots`.

If you do not wish to use telemetry then a TBS Crossfire or ELRS receiver can be connected to the **RCIN** port using :ref:`SBUS <common-rc-systems>`. You must configure the receiver to output SBUS, of course.


CRSF Receivers 
==============

If you wish to use telemetry then a receiver can be connected to a UART utilizing the `CRSF <https://www.team-blacksheep.com/products/prod:crossfire_tx>`__ protocol.

CRSF is a full-duplex protocol that supports integrated telemetry and a number of other features. Connect the RX pin of the UART to the CRSF TX pin of the CRSF device and vice versa.
Currently a full-duplex UART connection is required. For best performance a UART with DMA capability on its RX port is desirable, but not required. A message will be displayed once on the GCS console, if connected to a UART without this capability on an F4/F7 based autopilot.

In the configuration of the serial port select the RCIN protocol. So for example for serial port 4:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 23
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3

.. note:: The serial port baudrate is automatically set and controlled by the firmware when any serial RC protocol, such as CRSF, is detected.

With the receiver connected and configured correctly proceed with RC calibration as normal.

See :ref:`common-crsf-telemetry` for information about telemetry data sent, display scripts for OpenTX transmitters, and adjustment of ArduPilot parameters via CRSF.

.. warning:: If the autopilot is rebooted via MAVLink, it will lose communication with the CRSF receiver until the receiver is power cycled. Also, the CRSF TX must be transmitting BEFORE the receiver is powered up.

ELRS Receivers
==============

ELRS can be setup in the same manner as CRSF above , however, bit 13 of :ref:`RC_OPTIONS<RC_OPTIONS>` should be set to alter the baudrate from 416KBaud that CRSF uses, to 420KBaud that ELRS uses. As mentioned above, the UART selected must have DMA capability. ELRS can also be configured to output MAVLink, enabling full bidirectional Mavlink telemetry and RC control over the same link. See :ref:`ELRS <common-rc-systems>` for more information.

ELRS MAVLink Configuration
--------------------------

Instead of CRSF protocol, MAVLink protocol can be used. In this case, using SERIAL 4 for example:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 2
- Set :ref:SERIAL4_BBAUD <SERIAL4_BAUDL>` = 460
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 5

If the ELRS transmitter module has WIFI capability, then the telemetry data can be forwarded wirelessly to a PC or phone based GCS close to the transmitter.

MAVLink Option
--------------

In addition to SBUS and CRSF protocols, ELRS can be configured to use MAVLink protocol for telemetry and embedded RC control. To utilize this attach to SERIAL port 4(as an example) and configure:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 2
- Set :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` = 460
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` =  5

If the ELRS transmitter module has WIFI, the MAVLink telemetry can be wirelessly forwarded to a phone or PC GCS.

ELRS bootloader "lockup"
------------------------

Some ELRS receivers can appear to be locked up in "bootloader" mode when the autopilot system is powered on. This is usually due to the UART RX pin to which they are attached being held low during power application, which ELRS interprets as being forced into bootloader mode. 

The most common cause for this is when the autopilot board also has a DJI HD VTX attached and the SBUS pin of that interface is directly tied to the UART RX pin being used for RC input from the ELRS. If the VTX is not powered up at the same time as the ELRS, the SBUS output will act as a current sink for the ELRS TX pin and force it into bootloader. This occurs if the autopilot is only being powered from USB or the autopilot bootloader forces the HD VTX power supply off on boot(so that the pilot must toggle a transmitter switch to actively turn it on, for power/heat saving). The simple solution is to remove the SBUS wire going from the HD VTX to the DJI connector.

CRSF Video Transmitters
=======================

TBS Video Transmitters can also be connected to ArduPilot using the CRSF protocol. This allows the VTX to be configured via parameters and, for VTXs such as the `TBS Unify Evo <https://www.team-blacksheep.com/products/prod:tbs_unify_evo>`__ , receive telemetry data that can be displayed using the built-in OSD.

If you are using CRSF for RC input as well then nothing more needs to be done in order to enable VTX control.

If you only wish to use CRSF for VTX control then connect the VTX to the UART in the normal way - TX to RX and RX to TX and configure the UART connection as follows: 

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 29
