.. _common-omnibusf7:

==========
Omnibus F7
==========

.. figure:: ../../../images/omnibusf7.png
    :target: ../_images/omnibusf7.png
    :width: 450px

    Above image and some content courtesy of `myairbot.com <https://store.myairbot.com/flight-controller/omnibus-f7/omnibusf7v2.html>`__

.. note::

   Support for this board is available with Copter-3.6.0 (and higher)

Specifications
==============

-  **Processor**

   -  STM32F745VG ARM
   -  1MB of Flash memory


-  **Sensors**

   -  InvenSense MPU6000 IMU (accel, gyro) with vibration isolation
   -  InvenSense ICM20608 IMU (accel, gyros, compass) with vibration isolation
   -  BMP280 barometer
   

-  **Interfaces**

   -  UARTS
   -  PWM outputs
   -  RC input PWM/PPM, SBUS
   -  I2C port for external compass
   -  USB port
   -  Built-in OSD
   -  Voltage and Current sensing inputs (Needs external current sensor)


-  **Size and Dimensions**

   - 36mm x 36mm

Board Connections
=================
GPS is attached to UART6

Telem is available at UART 1

The shared UART3/I2C pins are enabled only for I2C operation to allow external compass or digital airspeed sensor attachment.If at least one device attached externally, does not have pull-up resistors, then 2K ohm pull-up resistors will need to be added externally.


Configuration
=============
Enable Battery monitor with these settings :

BATT_MONITOR=4

Then reboot.

:ref:`BATT_VOLT_PIN<BATT_VOLT_PIN>` 13

:ref:`BATT_AMP_OFFSET<BATT_AMP_OFFSET>` 0.008

:ref:`BATT_CURR_PIN<BATT_CURR_PIN>` 12

:ref:`BATT_VOLT_MULT<BATT_VOLT_MULT>` 10.925

:ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` 58.0 (note, this value is valid if using Matek Systems FCHUB A5 current sensor)...will need to be calibrated to match actual current if using any other make of PDB board)

Dshot capability
================

All motor/servo outputs are Dshot and PWM capable. However, mixing Dshot and normal PWM operation for outputs is restricted into groups, ie. enabling Dshot for an output in a group requires that ALL outputs in that group be configured and used as Dshot, rather than PWM outputs. The output groups that must be the same (PWM rate or Dshot, when configured as a normal servo/motor output) are: 1/2, and 3/4.

Where to Buy
============

- available from multiple retailers including `myairbot.com <https://store.myairbot.com/flight-controller/omnibus-f7/omninxtf7.html>`__
