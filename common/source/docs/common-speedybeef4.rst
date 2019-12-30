.. _common-speedybeef4:

=============
Speedy Bee F4
=============

.. warning:: This board's operation has not been verified. It requires an external I2C barometer in order to be ArduPilot compatible. Currently, this has not been re-verified as functional.

.. image:: ../../../images/speedybeef4.jpg
    :target: ../_images/speedybeef4.jpg

*above image and some content courtesy of the* `speedybee.com <https://www.speedybee.com/speedy-bee-f4-aio-flight-controller/>`__

.. note::

   Support for the SpeedyBee F4 was released with Copter-3.6.1

Specifications
==============

-  **Processor and Sensors**

   -  STM32F405 ARM microcontroller
   -  InvenSense MPU6000 IMU (accel, gyro, compass)

-  **Interfaces**

   -  4x PWM outputs
   -  1x RC input (PWM/PPM, SBUS)
   -  4x serial port inputs (including RC input listed above)
   -  1x I2C for external compass
   -  battery voltage and current monitor
   -  Onboard Bluetooth
   -  USB port
   -  3S to 6S input power

.. note:: This controller does not integrate a barometer which ArduPilot requires for proper operation, so an external barometer must be attached.

Dshot capability
================

All motor/servo outputs are Dshot and PWM capable. However, mixing Dshot and normal PWM operation for outputs is restricted into groups, ie. enabling Dshot for an output in a group requires that ALL outputs in that group be configured and used as Dshot, rather than PWM outputs. The output groups that must be the same (PWM rate or Dshot, when configured as a normal servo/motor output) are: 1/2/3/4,5, and 6.

Logging
=======

Logging to on-board data flash is supported on this controller.

Where to Buy
============

- Available from various retailers and directly from the manufacturer `SpeedyBee <https://www.speedybee.com/speedy-bee-f4-aio-flight-controller/>`__
