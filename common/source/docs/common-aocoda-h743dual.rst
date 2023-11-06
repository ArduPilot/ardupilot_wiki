.. _common-aocoda-h743dual:

======================================
Aocoda-RC H743Dual MPU6000/BMI270 Dual
======================================

.. image:: ../../../images/aocoda-h743dual/aocoda_h743dual.jpg
     :target: ../_images/aocoda-h743dual/aocoda_h743dual.jpg
    

the above image and some content courtesy of `aocoda-rc.com <https://www.aocoda-rc.com/>`__

.. note:: All versions use the same firmware, but have varying configurations for pinouts and resources available. See Aocoda RC's site for exact details for every variant. 

Specifications
==============

-  **Processor**

   -  STM32H743VIH6 (480MHz)


-  **Sensors**

   -  InvenSense MPU6000 / BOSCH BMI270 Dual IMU (accel, gyro) 
   -  DPS310/BMP280/MS56XX barometer
   -  Voltage & 130A current sensor


-  **Power**

   -  3-6S Lipo input power
   -  5V 2.5A BEC for peripherals
   -  9V 3A BEC for video


-  **Interfaces**

   -  8x UARTS
   -  12x PWM outputs
   -  1x RC input PWM/SBUS
   -  2x I2C ports for external compass, airspeed sensor, etc.
   -  4x SPI port
   -  USB port (type-C)
   -  4x ADC
   -  Buzzer and LED stripe
   -  Built-in OSD
   -  Blackbox Flash


-  **Size and Dimensions**

   - 37 mm x 37 mm
   - 8.8 g

-  **Mounting Hole**

   - 30.5 mm x 30.5 mm


Wiring Diagram
==================

.. image:: ../../../images/aocoda-h743dual/aocoda_h743dual_wiring_diagram.jpg
     :target: ../_images/aocoda-h743dual/aocoda_h743dual_wiring_diagram.jpg
  
Default UART order
==================

- SERIAL0 = USB (primary mavlink, usually USB)
- SERIAL1 = UART1 (Default RC input, CRSF when :ref:`BRD_ALT_CONFIG<BRD_ALT_CONFIG>` = 1)
- SERIAL2 = UART2 (GPS) 
- SERIAL3 = UART3 (VTX or DJI HD Air Unit)
- SERIAL4 = UART4 (USER)
- SERIAL5 = not available
- SERIAL6 = UART6 (ESC Telemetry)
- SERIAL7 = UART7 (USER)
- SERIAL8 = UART8 (USER)

.. note:: Serial port protocols (Telem, GPS, etc.) can be adjusted to personal preferences.

RC Input
========

RC input is configured on SERIAL1 (USART1), which is available on the Rx1, Tx1. PPM receivers are *not* supported as this input does not have a timer resource available. 

*Note* It is recommended to use CRSF/ELRS.

With recommended option:

- Set :ref:`SERIAL1_PROTOCOL<SERIAL1_PROTOCOL>` must be set to "23"
- Set :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` to "0".

Any UART can be used for RC system connections in ArduPilot also. See :ref:`common-rc-systems` for details.

RSSI/Analog Airspeed Input
==========================

Analog inputs are provided.

- Airspeed reference pin number is "4"
- RSSI reference pin number is "8"

OSD Support
===========

The Aocoda-RC H743Dual has an on-board OSD using :ref:`OSD_TYPE<OSD_TYPE>` =  1 (MAX7456 driver). The CAM and VTX pins provide connections for using the internal OSD.

DJI Video and OSD
=================

A "SH1.0 6P" connector supports a standard DJI HD VTX connection and SERIAL3 is already setup for this by default.  Pin 1 of the connector is 9V so be careful not to connect this to any peripheral requiring 5V.


Dshot capability
================

All motor/servo outputs are Dshot and PWM capable. However, mixing Dshot and normal PWM operation for outputs is restricted into groups, ie. enabling Dshot for an output in a group requires that ALL outputs in that group be configured and used as Dshot, rather than PWM outputs. The output groups that must be the same (PWM rate or Dshot, when configured as a normal servo/motor output) are: 1/2, 3/4, 5/6, 7/8, 9/10, 11/12, and 13 (LED).

GPIOs
=====

The Aocoda-RC H743Dual outputs can be used as GPIOs (relays, buttons, RPM etc). To use them you need to set the output's ``SERVOx_FUNCTION`` to -1. See :ref:`common-gpios` page for more information.

The numbering of the GPIOs for PIN variables in ArduPilot is:

 - PWM1 50
 - PWM2 51
 - PWM3 52
 - PWM4 53
 - PWM5 54
 - PWM6 55
 - PWM7 56
 - PWM8 57
 - PWM9 58
 - PWM10 59
 - PWM11 60
 - PWM12 61
 - PINIO1 81

.. warning:: PINIO1 is for 9V DC-DC control (HIGH:on; LOW:off). Default 9V DC is ON. Please install an antenna on VTX when battery powered.

Connecting a GPS/Compass module
===============================

This board does not include a GPS or compass so an :ref:`external GPS/compass <common-positioning-landing-page>` should be connected in order for autonomous modes to function.

.. note:: If the GPS is attached to UART2 TX/RX and powered from the adjacent 5V pins, a battery must be plugged in for power to be provided.

Battery Monitor Settings
========================

These should already be set by default. However, if lost or changed:

Enable Battery monitor with these parameter settings :

:ref:`BATT_MONITOR<BATT_MONITOR>` = 4

Then reboot.

First group of battery monitor pins & options:

 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN>` = 10
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN>` = 11
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT>` = 11 
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` = 17.0 (note: Please calibrate before use.)

Second group of battery monitor pins & options:

 - :ref:`BATT2_VOLT_PIN<BATT2_VOLT_PIN>` = 18
 - :ref:`BATT2_CURR_PIN<BATT2_CURR_PIN>` = 7
 - :ref:`BATT2_VOLT_MULT<BATT2_VOLT_MULT>` = 11 
 - :ref:`BATT2_AMP_PERVLT<BATT2_AMP_PERVLT>` = 17.0 (note: Please calibrate before use.)

.. note:: this autopilot uses a high precision current sensor which is sensitive to ESC switching noise. Please check carefully before use :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>`/:ref:`BATT2_AMP_PERVLT<BATT2_AMP_PERVLT>`, as voltage divider circuit for data collection is at ESC/BEC side.

Where to Buy
============

see this list of:

- `aocoda-rc.com <https://www.aocoda-rc.com/>`__
- `aocoda-rc aliexpress <https://www.aliexpress.com/item/1005005610849417.html>`__


Firmware
========

Firmware for these boards can be found `here <https://firmware.ardupilot.org>`_ in  sub-folders labeled
"Aocoda-RC-H743Dual".

.. warning:: Only outputs 1-8 are bi-directional Dshot capable by default. 

.. note:: If you experience issues with the device ceasing to initialize after power up, see :ref:`common-when-problems-arise` section for H7 based autopilots for a possible solution.

[copywiki destination="plane,copter,rover,blimp"]
