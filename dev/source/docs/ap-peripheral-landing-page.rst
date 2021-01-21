.. _ap-peripheral-landing-page:

.. image:: ../images/ap-periph-logo.png

=========
AP_Periph
=========

.. image:: ../images/ap-periph-collage.png


AP_Periph is an abbreviation for ArduPilot Peripheral, ie. an ArduPilot peripheral device based on the existing ArduPilot autopilot code. It takes peripheral device driver libraries of ArduPilot and adapts them to run on stand-alone peripheral devices, which communicate to the main autopilot via CAN, MSP, or other peripheral bus protocols.

The Peripheral device usually uses an STMF103 or STMF303 processor, but it is also being ported to use normal autopilot boards to be configured instead as a peripheral. Both sensors (distance sensor, GNSS, IMU, Barometer, etc.) and output ports (I2C, SPI, PWM, UART, ESC, LED, etc.) can be used to build new peripherals, as well as providing bus expansion for CAN, MSP, I2C,SPI, etc. 

The software uses the same build system as ArduPilot for autopilot boards. All the firmware build configuration for an AP_Periph board done using a single configuration file (hwdef.dat) defines the inputs / outputs of the device and what device drivers will be included, in the same manner as an autopilot board. This makes it possible, for example, to define a AP_Periph device for UAVCAN with only a micro-controller of the STM32F103 type and 128 KB of flash memory, although processors with larger memory will be required depending on the number of drivers.


.. images/ap-periph-block-diagram.png

Capabilities
============

- Bootloader update via serial or CAN port
- Firmware update
- Dynamic or static CAN node allocation
- Parameter storage in flash memory
- Self-diagnostic and security: watchdog, CRC, autotest, etc.
- Updates with MissionPlanner or UAVCAN tools like :ref:`common-uavcan-gui`

Existing Products
=================

Some (but not all) product examples using AP-Periph:

- :ref:`Mateksys M8Q<common-matek-m8q>`
- Hitec GNSS (`Septentrio Mosaic <https://hitecnology.com/drone-peripherals/hcs-positionpro-gnss-receiver>`__ )
- :ref:`mRo UAVCAN Adapter node<common-mro-uavcan-adapter-node>`
- :ref:`Orange Cube<common-thecubeorange-overview>` ( it's a flight controller,but it can also be used in AP_Periph..many others to follow)

Firmware
========

Firmware for existing device definitions are posted `here <https://firmware.ardupilot.org/AP_Periph/>`__. Their hardware definition files are in the ArudPilot Github repository, `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef>`__

To create firmware for a new design, follow the same instructions as for porting to a new autopilot board, as explained in the Wiki section :ref:`porting` .

Additional information is available in the AP_Periph readme `here <https://github.com/ArduPilot/ardupilot/blob/master/Tools/AP_Periph/README.md>`__
