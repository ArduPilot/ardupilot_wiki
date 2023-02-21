.. _common-spektrum-rc:
[copywiki destination="plane,copter,rover,blimp"]
===================
Spektrum RC Systems
===================

Any DSM/DSM2 compatible receiver can be used with ArduPilot. Spektrum also makes receivers with SBus, PPM, SRXL and SXRL2 protocols which are also compatible.

For a **Spektrum DSM**, **DSM2**, or **DSM-X Satellite** receiver,
connect to the **SPKT/DSM** port (some boards, such as the CUBE mini carrier board, require you to modify solder bridges. See the autopilot board's documentation)

.. image:: ../../../images/pixhawk_spektrum_connection.jpg
    :target: ../_images/pixhawk_spektrum_connection.jpg
    
Spektrum Satellite Receivers 
============================

`Spektrum Satellite Receivers <http://www.spektrumrc.com/Products/Default.aspx?ProdID=SPM9645>`__
work as a DSM receiver with Pixhawk. Spektrum satellite receivers do not have buttons to bind, so
there are two ways to bind them to a transmitter. They are connected as if they were PPM-SUM or SBus output devices.
ArduPilot automatically discovers the serial protocol being used.

-  Pre-bind the Spektrum Satellite to your transmitter using a
   conventional Spektrum receiver with satellite attached, then
   disconnect the satellite from the conventional receiver, and
   connect it to the Spektrum port on autopilot
-  Bind the satellite receiver using Mission Planner to initiate
   the bind. This functionality is located in Radio Calibration 
   screen on Initial Setup.

.. image:: ../../../images/dsm_bind.png
    :target: ../_images/dsm_bind.png

-  Being a satellite, range may be limited and the
   preceding receivers and methods may provide greater range.

.. image:: ../../../images/spm9645.jpg
    :target: ../_images/spm9645.jpg

.. image:: ../../../images/PX4SpektrumSatellite1.jpg
    :target: ../_images/PX4SpektrumSatellite1.jpg

.. _common-spektrum-srxl2-rc:

Spektrum SRXL2 Receivers 
========================

All Spektrum receivers released since August 2019 only support the `SRXL2 <https://github.com/SpektrumRC/SRXL2>`__ protocol. SRXL2 is a single-wire, half-duplex protocol that supports integrated telemetry and a number of other features. 

An `SRXL2 cable <https://www.spektrumrc.com/Products/Default.aspx?ProdID=SPM4650>`__ has four wires, one of which is not connected. This new format is to distinguish them from older receivers supporting DSMX since DSMX and SRXL2 are incompatible.

SRXL2 receivers must be connected to a UART. SRXL2 receivers support a wide range of voltages, including the 5v available on a UART so connect GND to GND, VCC  to VCC and the signal wire to the TX pin of the UART.

In the configuration of the serial port select the RCIN protocol and set the serial options to half-duplex operation. So for example for serial port 4:

- Set :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 23
- Set :ref:`SERIAL4_OPTIONS <SERIAL4_OPTIONS>` = 4
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3

Many single board autopilots support RC input on serial port 6 only exposing the RX connection. For F7 autopilots, such as the Kakute F7, it is necessary to swap the RX and TX pins in order to support SRXL2. It is also necessary to support serial access on the UART by using the board's alternate configuration:

- Set :ref:`SERIAL6_PROTOCOL <SERIAL6_PROTOCOL>` = 23
- Set :ref:`SERIAL6_OPTIONS <SERIAL6_OPTIONS>` = 12
- Set :ref:`RSSI_TYPE <RSSI_TYPE>` = 3
- Set :ref:`BRD_ALT_CONFIG <BRD_ALT_CONFIG>` = 1

With the receiver connected and configured correctly proceed with RC calibration as normal.


