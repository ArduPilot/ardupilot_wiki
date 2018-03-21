.. _common-pixhawk2-overview:

============================
Pixhawk 2 (TheCube) Overview
============================

.. image:: ../../../images/thecube/connect-to-com-port.jpg
    :target: ../images/thecube/connect-to-com-port.jpg
    :width: 360px

Specifications
==============

-  **Processor**

   -  32-bit ARM Cortex M4 core with FPU
   -  168 Mhz/256 KB RAM/2 MB Flash
   -  32-bit failsafe co-processor

-  **Sensors**

   -  Three redundant IMUs (accels, gyros and compass)
   -  InvenSense MPU9250, ICM20948 and/or ICM20648 as first and third IMU (accel and gyro)
   -  ST Micro L3GD20+LSM303D or InvenSense ICM2076xx as backup IMU (accel and gyro)
   -  Two redundant MS5611 barometers

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

Pixhawk 2.1 system features
=======================
Pixhawk2.1 'The Cube' flight controller is a further evloution of the Pixhawk flight controller. It is designed for commercial systems and manufacturers who wish to integrate flight controller into their system. On top of the existing features of Pixhawk, it has the following enhancements:

-  3 sets of IMU sensors for extra redundancy
-  2 sets of IMU are vibration-isolated by built-in pieces of foam, reducing the effect of frame vibration to state estimation
-  IMUs are temperature-controlled by onboard heating resistors, allowing optimum working temperature of IMUs
-  The entire flight management unit(FMU) and inertial management unit(IMU) are housed in a reatively small foam factor (a cube). All power and signal inputs and outputs go through a 80-pin DF17 connector, allowing a plug-in solution for manufacturers of commercial systems. Manufacturers can design their own carrier boards to suite their specific needs of connectivity.



Where to Buy
============

Official retailers are listed `here  <http://www.proficnc.com/stores>`__.

Quick Start
===========

Use the :ref:`Pixhawk Wiring QuickStart <common-pixhawk-wiring-and-quick-start>` as a guide. PH2 update coming soon

More Information
================

see  `www.proficnc.com  <http://www.proficnc.com>`__
The reference design files of the standard carrier board are available in <https://github.com/proficnc/pixhawk2.1>, this serve as a starting point for designers to design their own system based on Pixhawk2.1 flight controller.

More Images
===========

.. image:: ../../../images/thecube/pixhawk2-overhead.jpg
    :target: ../images/thecube/pixhawk2-overhead.jpg
    :width: 360px
