.. _common-furiousfpv-f35:

========================================
Furious FPV F-35 Lightning and Wing FC10
========================================

.. image:: ../../../images/furiousfpv-f35.jpg
    :target: ../_images/furiousfpv-f35.jpg

*above image and some content courtesy of the* `Furious FPV website <https://furiousfpv.com/product_info.php?cPath=25&products_id=641>`__ and `banggood.com <https://www.banggood.com/Wing-FC-10-DOF-Flight-Controller-INAV-OSD-Accelerometer-Barometer-Gyro-Compass-For-RC-Airplane-Drone-p-1318626.html>`__

.. note::

   Support for the FuriousFPV and WingFC10 were released with Copter-3.6.1.  These boards use the same firmware.

Specifications
==============

-  **Processor and Sensors**

   -  STM32F4 ARM microcontroller
   -  InvenSense IMU (accel, gyro, compass)

-  **Interfaces**

   -  6x PWM outputs
   -  1x RC input (PWM/PPM, SBUS)
   -  6x serial port inputs
   -  battery voltage and current monitor
   -  Onboard OSD
   -  USB port
   -  2S to 6S input power

Where to Buy
============

- Furious FPV F-35 Lightening is available from `FuriousFPV <https://furiousfpv.com/product_info.php?cPath=25&products_id=641>`__ (`Full options model here <https://furiousfpv.com/product_info.php?cPath=25&products_id=657>`__)
- Wing FC-10 is available from multiple retailers including `banggood.com <https://www.banggood.com/Wing-FC-10-DOF-Flight-Controller-INAV-OSD-Accelerometer-Barometer-Gyro-Compass-For-RC-Airplane-Drone-p-1318626.html>`__

Peripheral Connections
======================

.. image:: ../../../images/furiousfpv-f35-wiring.jpg
    :target: ../_images/furiousfpv-f35-wiring.jpg
    
Default UART order
==================

- SERIAL0 = console = USB
- SERIAL1 = Telemetry1 = USART1
- SERIAL2 = Telemetry2 = UART5
- SERIAL3 = GPS1 = USART2
- SERIAL4 = not used
- SERIAL5 = USER = UART4 (only TX4 pinned out)
- SERIAL6 = USER = USART6 (only TX6 pinned out as "SPO" with hardware inverter)

Serial protocols can be adjusted to personal preferences.

Notes
=====
The F-35 has ability to have the middle rail for M1/M2 to be independent of that of the S1-S6 slots, which are supplied by the internal 5V regulator, or to have it tied to them, via board jumper pads. The WingFC10 is isolated with no pad option. The internal regulators can only supply two or three standard servos (3A).
It is generally recommended that servos be powered independent of on-board regulators to avoid noise injection issues. For both boards this means either cutting traces or externally powering the servos.

Both boards have an onboard compass, however, since there is an on-board current sensor, if used, the compass cannot be accurately used and should be disabled in Ardupilot. For plane use, not having a compass is a non-issue, but compass is required for Copter/Quadplane. Since these boards have no external I2C pinouts, external compass is not an option. So, currently, this board is only applicable to Plane applications. (Using Compass_Motor_Compensation for copter/quadplane may be viable,but it has not been shown yet)

Either rssi or analog airspeed can be input on the AIR pin. Either must be first enabled in the params, params refreshed, and then pin "13" set for the input, and type set.

Videos
======
arduplane auto mission using the Wing FC10: https://vimeo.com/301399536
