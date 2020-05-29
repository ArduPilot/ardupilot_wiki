.. _common-thecubeorange-overview:

=================
The Cube Orange With ADSB-In Overview
=================

.. image:: ../../../images/Cube_orange_adsb.jpg
    :target: ../_images/Cube_orange_adsb.jpg
    :width: 360px

System Features
===============

The Cube Orange autopilot is the latest and most powerful model in the Cubepilot ecosystem. Designed for hobby users, commercial system integrators and UAS manufacturers the Cube Orange autopilot is part on a wide ecosystem of autopilot modules and carrier boards. Because all Cube models are compatible with all carriers this allows users to choose an off the shelf carrier board that best suits their needs. System designers are able to integrate the Cube directly into their designs via published carrier board specifications. 


The Cube Orange is available as a standalone module or as a package with a new updated version of the original carrier board that now includes an integrated ADS-B In module from uAvionics. 

The new carrier boards overall footprint is identical to the standard versions and main changes compared to original carrier are as follows:

 - Integration of uAvonix ADS-B IN Receiver on Serial 5
 - Built-In ADS-B Antenna 
 - Removal of Intel Edison Bay and Debug USB Ports

All other specification and external connections remain identical to the original board s listed on the Cube Black page.

Cube Orange Features
==============

-  3 sets of IMU sensors for extra redundancy
-  2 sets of IMU are vibration-isolated mechanically, reducing the effect of frame vibration to state estimation
-  IMUs are temperature-controlled by onboard heating resistors, allowing optimum working temperature of IMUs
-  The entire flight management unit(FMU) and inertial management unit(IMU) are housed in a reatively small form factor (a cube). All inputs and outputs go through a 80-pin DF17 connector, allowing a plug-in solution for manufacturers of commercial systems. Manufacturers can design their own carrier boards to suite their specific needs.

Specifications
==============

-  **Processor**

   -  STM32 H7 Cortex M4 core with FPU
   -  400 Mhz/1 MB RAM/2 MB Flash
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



The Cube connector pin assignments
==================================

All other specification and external connections remain identical to the original board s listed on the Cube Black page.

Where to Buy
============

Official retailers are listed `here  <http://www.proficnc.com/stores>`__.

Quick Start
===========

Use the :ref:`Pixhawk Wiring QuickStart <common-pixhawk-wiring-and-quick-start>` as a guide. The Cube update coming soon

More Information
================

see  `www.proficnc.com  <http://www.proficnc.com>`__

The reference design files of the standard carrier board are available in `github  <https://github.com/proficnc/The-Cube>`__, or `here <https://github.com/ArduPilot/Schematics/tree/master/ProfiCNC>`__ ,this serve as a starting point for designers to design their own system based on The Cube autopilot.

More Images
===========

.. image:: ../../../images/thecube/pixhawk2-overhead.jpg
    :target: ../_images/pixhawk2-overhead.jpg
    :width: 360px
