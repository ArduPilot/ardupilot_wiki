.. _common-uavcan-adapter-node:

===================
UAVCAN Adapter Node
===================

These allow existing ArduPilot supported peripheral to be adapted to the CAN bus as UAVCAN devices.

.. image:: ../../../images/uavcan-node.jpg

They utilize the `AP_Periph <https://github.com/ArduPilot/ardupilot/tree/master/Tools/AP_Periph>`__ library to remotely locate existing ArduPilot drivers onto an STMF103 or STMF303 based device, translating UART,SPI, I2C, or GPIO-based peripheral devices supported by ArduPilot into UAVCAN devices on the CAN bus.

GPS adapted to UAVCAN

.. image:: ../../../images/uavcan-node-gps.jpg
   :width: 450px

This provides an easy method to develop integrated UAVCAN peripherals which can be used with ArduPilot or other UAVCAN systems.

Firmware
========

`Firmware <https://firmware.ardupilot.org/AP_Periph/>`__ is provided in the AP_Periph folder for several UAVCAN devices based on this concept including versions for a typical UAVCAN Adapter Node itself, manufactured by `mRobotics <https://store.mrobotics.io/product-p/mro10042.htm>`__.

.. image:: ../../../images/mRo-can-node.jpg


The f103-GPS firmware enables its UART interface for GPS and provides I2C compass interface detection. The f103-ADSB firmware enables its UART for ADS-B Receiver attachment instead of GPS. The f103-RangeFinder firmware enables its UART for connection to a Rangefinder.

`Schematic <https://github.com/ArduPilot/Schematics/blob/master/mRobotics/mRo_CANnode_V1_R1.pdf>`__


Features
=========

The AP_Periph firmware can be configured to enable a wide range of
UAVCAN sensor types. Support is included for:

 - GPS modules (including RTK GPS)
 - Magnetometers (SPI or I2C)
 - Barometers (SPI or I2C)
 - Airspeed sensors (I2C)
 - Rangefinders (UART or I2C)
 - ADSB (Ping ADSB receiver on UART)
 - LEDs (GPIO, I2C or WS2812 serial)
 - Safety LED and Safety Switch
 - Buzzer (tonealarm or simple GPIO)

An AP_Periph UAVCAN firmware supports these UAVCAN features:

 - dynamic or static CAN node allocation
 - firmware upload
 - automatically generated bootloader
 - parameter storage in flash
 - easy bootloader update
 - high resiliance features using watchdog, CRC and board checks
 - firmware update via MissionPlanner or uavcan-gui-tool


Setup
======

AP_Periph f303-universal Firmware
---------------------------------

 The f303-Universal Firmware has the ability to be used for several devices but only one can be
 enabled to use the single uart.
 Once uploaded the default device is set to GPS, to use another device such as Rangefinder the
 GPS has to be turned off and Rangefinder or other device enabled. To achieve this either use
 UAVCAN GUI or MissionPlanner SLCAN. (link here). Once connected set params i.e. for TFmini_S/Plus rangefinder
 you would set GPS_TYPE=0 and RNGFND1_TYPE=20, this allows the single serial port to connect to the rangefinder.

  Options for serial devices are:

 - GPS_TYPE=0
 - COMPASS_ENABLE=0
 - BARO_ENABLE=0
 - RNGFND1_TYPE=0
 - ADSB_BAUDRATE=0

 The above settings would disable all of the devices, then you should enable just the ones you want,
 knowing that you can’t have two serial devices as there is just one uart.

  The Firmware can also be used for I2C peripherals.

 - COMPASS
 - BARO
 - AIRSPEED SENSOR
 - NCP5623 LED

 Rangefinder
 ------------

 To use rangefinders, follow the instructions at  :ref:`UAVCAN Setup Advanced<common-uavcan-setup-advanced>` to set up the Ardupilot parameters. Using MissionPlanner or UAVCAN Gui, set the parameters on the adaptor node following the instructions for the relevant rangefinder.

 .. note::

 	The orientation of the rangefinder (RNGFND1_ORIENT) must be set to 0 on the adaptor node.


 .. note::

 	The RNGFNDx_ADDR Ardupilot parameter must be set above 0 and be equal to the number set on the UAVCAN adapter node.
