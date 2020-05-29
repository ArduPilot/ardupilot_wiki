.. _common-thecubepurple-overview:

=================
The Cube Purple Overview
=================

.. image:: ../../../images/Cube_purple.jpg
    :target: ../_images/Cube_purple.jpg
    :width: 360px

System Features
===============

The Cube Purple autopilot is designed for ground based applications such as boat, car or rover. Based on the Cube Black main module the purple is a much smaller platform for use cases where sensor redundancy is not required.

-  Single IMU sensor
-  Single Baro Sensor
-  The entire flight management unit(FMU) and inertial management unit(IMU) are housed in a reatively small form factor (a cube). All inputs and outputs go through a 80-pin DF17 connector, allowing a plug-in solution for manufacturers of commercial systems. Manufacturers can design their own carrier boards to suite their specific needs.

Specifications
==============

-  **Processor**

   -  32-bit ARM Cortex M4 core with FPU
   -  168 Mhz/256 KB RAM/2 MB Flash
   -  32-bit failsafe co-processor

-  **Sensors**

   -  1 x IMU (accels, gyros and compass)
   -  1 x InvenSense MPU9250 (accel and gyro)
   -  1 x MS5611 barometers

-  **Power**

   -  Redundant power supply with automatic failover
   -  Servo rail high-power (7 V) and high-current ready
   -  All peripheral outputs over-current protected, all inputs ESD
      protected

-  **Interfaces**

   -  14x PWM servo outputs (8 from IO, 6 from FMU)
   -  S.Bus servo output
   -  R/C inputs for CPPM, Spektrum / DSM and S.Bus
   -  Analogue / PWM RSSI input
   -  5x general purpose serial ports, 2 with full flow control
   -  2x I2C ports
   -  SPI port (un-buffered, for short cables only not recommended for use)
   -  2x CAN Bus interface
   -  3x Analogue inputs (3.3V and 6.6V)
   -  High-powered piezo buzzer driver (on expansion board)
   -  High-power RGB LED (I2C driver compatible connected externally only)
   -  Safety switch / LED
   -  Optional carrier board for Intel Edison



The Cube connector pin assignments
==================================

All other specification and external connections remain identical to the original board s listed on the Cube Black page.

Cubepilot Ecosystem
===================

.. image:: ../../../images/Cubepilot_ecosystem.jpg
    :target: ../_images/Cubepilot_ecosystem.jpg


More Information
================

For more detials and instructions on the Cubepilot system see  `Cubepilot Docs  <https://docs.cubepilot.org/user-guides/>`__

For help and support see  `Cubepilot Forum  <https://discuss.cubepilot.org/>`__

More info on Cubpilot see  `www.proficnc.com  <http://www.proficnc.com>`__


Carrier Board Design
====================

The reference design files of the standard carrier board are available in `github  <https://github.com/proficnc/The-Cube>`__, or `here <https://github.com/ArduPilot/Schematics/tree/master/ProfiCNC>`__ ,this serve as a starting point for designers to design their own system based on The Cube autopilot.

Where to Buy
============

Official retailers are listed `here  <http://www.proficnc.com/stores>`__.
