.. _parrot-bebop-autopilot:

======================
Parrot Bebop Autopilot
======================

The Parrot `Bebop Drone <http://www.parrot.com/usa/products/bebop-drone/>`__ is a Wifi
controlled quadrotor UAV that uses `this Linux autopilot <https://us.store.parrot.com/en/accessoires/247-main-board-3520410021619.html>`__
and which can run Copter firmware.  Instructions for converting a Bebop
to run ardupilot are
:ref:`here <dev:building-for-bebop-on-linux>`.

.. note::

   Copter support was added in Copter-3.3.

.. image:: ../../../images/bebop-drone.jpg
    :target: ../_images/bebop-drone.jpg

Specifications
==============

-  **Processor**

   -  Parrot P7 dual-core CPU Cortex 9 with quad core GPU
   -  8GB flash

-  **Sensors**

   -  MPU6050 for accelerometers and gyroscope (I2C)
   -  AKM 8963 compass
   -  MS5607 barometer
   -  `Furuno GN-87F GPS <http://www.furuno.com/en/products/gnss-module/GN-87>`__
   -  Sonar
   -  Optical-flow
   -  HD camera

-  **Interfaces**

   -  1x UART serial ports
   -  USB
   -  Built-in Wifi

-  **Dimensions**

   -  33x38x3.6cm (with hull)
   -  400g (with hull)

-  **OS**

   -  Linux (Busybox)

.. note::

   Some of this information was taken from the `Paparazzi UAV wiki page on the Bebop <http://wiki.paparazziuav.org/wiki/Bebop>`__.

Video of Bebop flying Copter-3.3
================================

..  youtube:: IUzM7Ln_MZE
    :width: 100%
