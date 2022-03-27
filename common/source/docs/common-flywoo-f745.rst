.. _common-flywoo-f745:

=====================
FlyWoo F745 AIO BL_32
=====================

.. image:: ../../../images/flywooF745.png
   :target: ../_images/flywooF745.png
   
.. note:: This article also applies to other Flywoo GOKU F745 boards. Check `Flywoo website <https://flywoo.net>`__ for detailed pinouts. Use the FlywooF745 firmware on the `firmware server <https://firmware.ardupilot.org>`__ . For 16.5mm x 16.5mm F745 boards use the FlywooF745Nano firmware.

Specifications
==============

-  **Processor**

   -  STM32F745VG  ARM (216MHz), 1MB Flash
   -  Integrated 4 output, BLHeli-32 40A ESC


-  **Sensors**

   -  InvenSense MPU6000 IMU (accel, gyro) 
   -  BMP280 barometer
   -  Voltage & 100A Current sensor


-  **Power**

   -  7.4V ~ 25V DC input power
   -  5V 2A BEC for peripherals
   -  9V 1.5A BEC for video


-  **Interfaces**

   -  7x UARTS
   -  10x PWM outputs, first 4 are internally connected to 4in1 40A BLHeli32 ESC.
   -  I2C port for external compass, airspeed sensor, etc.
   -  USB port
   -  Camera input/ VTX output
   -  Built-in OSD


-  **Size and Dimensions**

   - 33.5mm x 333.5mm (25.6 x 25.6mm mount pattern)
   - 8.5g

Where to Buy
============

https://flywoo.net/products/goku-gn745-40a-aio-bl_32-mpu6000-25-5-x-25-5


Default UART order
==================

- SERIAL0 = console = USB
- SERIAL1 = Telemetry1 = USART1 
- SERIAL2 = Telemetry2 = USART2
- SERIAL3 = RC Input = USART3 
- SERIAL4 = USER = USART4
- SERIAL5 = USER = UART5
- SERIAL6 = GPS = USART6
- SERIAL7 = ESC Telem = UART7 (RX tied to ESC telemetry) See :ref:`blheli32-esc-telemetry`

UART3 supports RX and TX DMA. UART1, UART2, UART4, and UART6 supports TX DMA. UART5 and UART7 do not support DMA. Serial port protocols (Telem, GPS, etc.) can be adjusted to personal preferences.

RC Input
========

RC input is configured by default via the USART3 RX input. It supports all RC protocols except PPM. FPort and full duplex protocols will use both RX6 and TX6. The 4.5V output pin is powered when the USB is connected to allow setup of receiver without connecting to the main battery.

.. note:: If the receiver is SBUS, then :ref:`SERIAL3_OPTIONS<SERIAL3_OPTIONS>` must be set to "1" to provide inversion for detection. If the receiver is FPort, then the receiver must be tied to the USART3 TX pin and :ref:`SERIAL3_OPTIONS<SERIAL3_OPTIONS>` = 7 (invert TX/RX, half duplex) and :ref:`RSSI_TYPE<RSSI_TYPE>` =3.


Dshot Capability
================

All motor 1-4 outputs are bi-directional Dshot and PWM capable. However, mixing Dshot and normal PWM operation for outputs is restricted into groups, ie. enabling Dshot for an output in a group requires that ALL outputs in that group be configured and used as Dshot, rather than PWM outputs. The output groups that must be the same (PWM rate or Dshot, when configured as a normal servo/motor output) are: 1/2, 3/4/10, 5, 6, 7/8, 9.

Neopixel Output
===============

The LED pin is PWM output 9 and is default setup for use with a NeoPixel 4 led string for notifications.

Battery Monitor
===============

The board has a built-in voltage and current sensors.

The correct battery monitor parameters are:

-    :ref:`BATT_MONITOR<BATT_MONITOR>` =  4
-    :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN>` = 12
-    :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT>` ~ 10.9
-    :ref:`BATT_CURR_PIN<BATT_CURR_PIN>` = 13
-    :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` ~ 28.5

These are set by default in the firmware and shouldn't need to be adjusted.

Flashing Firmware
=================

Usually these boards are sold pre-flashed with Betaflight firmware and require both firmware and bootloader to be updated if you want to use ArduPilot. See :ref:`common-loading-firmware-onto-chibios-only-boards`.

[copywiki destination="plane,copter,rover,blimp"]
