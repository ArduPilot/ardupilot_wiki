.. _common-uavcan-adapter-node:

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

`Firmware <http://firmware.ardupilot.org/AP_Periph/>`__ is provided in the AP_Periph folder for several UAVCAN devices based on this concept including versions for a typical UAVCAN Adapter Node itself, manufactured by `mRobotics <https://store.mrobotics.io/product-p/mro10042.htm>`__. 

.. image:: ../../../images/mRo-can-node.jpg


The f103-GPS firmware enables its UART interface for GPS and provides I2C compass interface detection. The f103-ADSB firmware enables its UART for ADS-B Receiver attachment instead of GPS. The f103-RangeFinder firmware enables its UART for connection to a Rangefinder.

`Schematic <https://github.com/ArduPilot/Schematics/blob/master/mRobotics/mRo_CANnode_V1_R1.pdf>`__

