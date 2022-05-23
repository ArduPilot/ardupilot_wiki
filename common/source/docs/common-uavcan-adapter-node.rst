.. _common-uavcan-adapter-node:

=====================
DroneCAN Adapter Node
=====================

These allow existing ArduPilot supported peripherals to be adapted to the CAN bus as DroneCAN devices.

.. image:: ../../../images/uavcan-node.jpg

They utilize the `AP_Periph <https://github.com/ArduPilot/ardupilot/tree/master/Tools/AP_Periph>`__ library to remotely locate existing ArduPilot drivers onto an STMF103 or STMF303 based device, translating UART,SPI, I2C, or GPIO-based peripheral devices supported by ArduPilot into DroneCAN devices on the CAN bus.

GPS adapted to DroneCAN:

.. image:: ../../../images/uavcan-node-gps.jpg
   :width: 450px


This provides an easy method to develop integrated DroneCAN peripherals which can be used with ArduPilot or other DroneCAN systems.

The first adapter was manufactured by `mRobotics <https://store.mrobotics.io/product-p/mro10042.htm>`__ , is shown below,and provides socketed outputs for a UART+I2C and another I2C connection, and board pads for a second UART, SPI bus, GPIOs, and ADC inputs.

.. image:: ../../../images/mRo-can-node.jpg

The first generation was based on the f103 processor, while current generation uses an f303 for more memory, allowing more peripheral options to be accommodated simultaneously in the firmware.


`Schematic <https://github.com/ArduPilot/Schematics/blob/master/mRobotics/mRo_CANnode_V1_R1.pdf>`__

Features
========

The AP_Periph DroneCAN firmware can be configured to enable a wide range of
DroneCAN sensor types. Support is included for:

 - GPS modules (including RTK GPS)
 - Magnetometers (SPI or I2C)
 - Barometers (SPI or I2C)
 - Airspeed sensors (I2C)
 - Rangefinders (UART or I2C)
 - ADSB (Ping ADSB receiver on UART)
 - LEDs (GPIO, I2C or WS2812 serial)
 - Safety LED and Safety Switch
 - Buzzer (tonealarm or simple GPIO)

And AP_Periph DroneCAN firmware supports these DroneCAN features:

 - dynamic or static CAN node allocation
 - firmware upload
 - automatically generated bootloader
 - parameter storage in flash
 - easy bootloader update
 - high resiliance features using watchdog, CRC and board checks
 - firmware update via MissionPlanner or uavcan-gui-tool

Firmware
========

`Firmware <https://firmware.ardupilot.org/AP_Periph/>`__ is provided in the AP_Periph folder for several DroneCAN devices based on this concept. Currently, the following firmware is pre-built, but the code allows for easy customization for any given peripheral or adapter based on the F103/303 processors. Firmware can be installed using either :ref:`DroneCAN GUI<common-uavcan-gui>` or :ref:`MissionPlanner SLCAN.<common-mp-slcan>` when the device is attaced to a DroneCAN port on an autopilot and the autopilot has enabled that port, see :ref:`common-uavcan-setup-advanced`.


F103 Based
----------

- f103-GPS         :Serial GPS, I2C Compass, I2C RGB LED
- f103-ADSB        :Serial ADS_B, I2C Compass, I2C Airspeed
- f103-Rangefinder :Serial Rangefinder, I2C Airspeed

F303 Based
----------

- f303-GPS         :Serial GPS, SPI RM3100 Compass, I2C Compass, I2C RGB LED
- f303-M10025      :Serial GPS, SPI RM3100 Compass, SPI DPS310 Baro, I2C RGB LED, I2C Airspeed, Safety Switch
- f303-Universal   :Serial GPS/Rangefinder/ADS-B, I2C Compass, I2C Baro, I2C RGB LED, I2C Airspeed

L431 Based
----------

- MatekL431-Periph      :Serial GPS, I2C QMC5883L Compass, I2C SPL06 Baro, I2C RGB LED, I2C Airspeed (MS4525 default), Passive Buzzer, Battery Monitor, MSP, 5 PWM outputs(not recommended for use, use MatekL431-DShot for those applications)
- MatekL431-Airspeed    :I2C Airspeed, DLVR 10" default type
- MatekL431-DShot       :5 Bi-Directional DShot(default)/PWM outputs starting at SERVO5 by default, ESC telem on UART1 RX ( see `setup instructions here <https://discuss.ardupilot.org/t/using-matekl431-adapters-for-pwm-and-dshot>`__ )
- MatekL431-Rangefinder :Serial Rangefinders


ArduPilot Firmware DroneCAN Setup
=================================

.. note:: Be sure to enable the autopilot's CAN port and set it up for DroneCAN protocol. See : :ref:`common-uavcan-setup-advanced`

DroneCAN Adapters can support various devices and configurations. Often, its configuration parameters will need to be altered. To achieve this, either use :ref:`DroneCAN GUI<common-uavcan-gui>` or :ref:`MissionPlanner SLCAN.<common-mp-slcan>` to change the device's parameters.

For example, when using the MatekL431-Airspeed, you may need to change the ARSPD_TYPE parameter in the device to match the actual I2C airpseed sensor you are using.

f303-Universal Setup
--------------------

The f303-Universal firmware has the ability to be used for several serial devices but only one can be enabled to use the single UART. Once Firmware is uploaded, the default device connected to the UART port is set to GPS, to use another device such as Rangefinder, the GPS has to be turned off and Rangefinder or other device enabled.

Options for serial devices are:

 - GPS_TYPE=0
 - RNGFND1_TYPE=0
 - ADSB_BAUDRATE=0

 The above settings would disable all of the devices, then you should enable just the one you want, knowing that you can’t have two serial devices as there is just one UART.

The firmware can also be used for I2C peripherals.

 - COMPASS
 - BARO
 - AIRSPEED SENSOR
 - NCP5623 LED

Rangefinder Setup
=================

 To use rangefinders, follow the instructions at  :ref:`DroneCAN Setup Advanced<common-uavcan-setup-advanced>` to set up the ArduPilot parameters. Using MissionPlanner or DroneCAN Gui, set the parameters on the adaptor node following the instructions for the relevant rangefinder.

 .. note::

 	The orientation of the rangefinder (RNGFND1_ORIENT) must be set to 0 on the adaptor node.


 .. note::

 	The RNGFNDx_ADDR ArduPilot parameter must be set above 0 and be equal to the number set on the DroneCAN adapter node.

DroneCAN Adapter Nodes
======================

:ref:`mRo DroneCAN Adapter Node <common-mro-uavcan-adapter-node>`
`MatekL431 DroneCAN Adapter Node <http://www.mateksys.com/?portfolio=can-l431>`__
