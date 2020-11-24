.. _common-matekh743-wing:

==================
Mateksys H743-Wing
==================

.. image:: ../../../images/matekh743-wing.png
     :target: ../_images/matekh743-wing.png
    

the above image and some content courtesy of `mateksys.com <http://www.mateksys.com/>`__

.. note::

   Support for this board is available with ArduPilot Plane 4.0.6, Copter 4.0.4, and higher

Specifications
==============

-  **Processor**

   -  STM32H743VIT6  ARM (480MHz)


-  **Sensors**

   -  InvenSense MPU6000 IMU (accel, gyro) & ICM20602
   -  DPS310 barometer
   -  Voltage & 132A current sensor


-  **Power**

   -  9V ~ 36V DC input power
   -  5V 2A BEC for peripherals
   -  9/12V 2A BEC for video
   -  5/6/7.2V 8A BEC for servos


-  **Interfaces**

   -  7x UARTS
   -  13x PWM outputs
   -  1x RC input PWM/PPM, SBUS
   -  2x I2C ports for external compass, airspeed sensor, etc.
   -  SPI4 port
   -  USB port (with remote cabling)
   -  CAN port
   -  6 ADC
   -  Buzzer and Safety Switch
   -  Dual Switchable Camera inputs
   -  Built-in OSD
   -  microSD card
   -  Second battery monitor input pins


-  **Size and Dimensions**

   - tbd mm x tbd mm x tbd mm
   - tbd g


Camera and Supply Switch
========================

Switching between the two camera inputs, C1 (default on) or C2, and between on (default) and off of Vsw (5/9V selectable supply), can be implemented using the Relay function of ArduPilot and assigning the relays to an ``RCx_OPTION`` switch on the transmitter.

Set the ``RELAYx_PIN`` to "81" for on/off of Vsw, and to "82" to control the camera switching.
Then select an RC channel for control (Chx) and set its ``RCx_OPTION`` to the appropriate Relay (1-4) that you had set its pin parameter above.

For example, use Channel 10 to control the camera switch using Relay 2:

- :ref:`RELAY_PIN2<RELAY_PIN2>` = "82"
- :ref:`RC10_OPTION<RC10_OPTION>` = "34" (Relay2 Control)
   
Default UART order
==================

- SERIAL0 = console = USB
- SERIAL1 = Telemetry1 = UART7 (support CTS and RTS signaling)
- SERIAL2 = Telemetry2 = USART1
- SERIAL3 = GPS1 = USART2
- SERIAL4 = GPS2 = USART3
- SERIAL5 = USER = UART8
- SERIAL6 = USER = UART4
- SERIAL7 = USER = UART6 (TX only unless :ref:`BRD_ALT_CONFIG<BRD_ALT_CONFIG>` = 1, then RX available also)

RC input is configured on the R6 (UART6_RX) pin. It supports all RC protocols, however for FPort the receiver should be connected to T6 and SERIAL7 configured as described in :ref:`FPort<common-FPort-receivers>` section.

Serial port protocols (Telem, GPS, etc.) can be adjusted to personal preferences.

Dshot capability
================

All motor/servo outputs are Dshot and PWM capable. However, mixing Dshot and normal PWM operation for outputs is restricted into groups, ie. enabling Dshot for an output in a group requires that ALL outputs in that group be configured and used as Dshot, rather than PWM outputs. The output groups that must be the same (PWM rate or Dshot, when configured as a normal servo/motor output) are: 1/2, 3/4/5/6, 7/8/9/10, 11/12, and 13 (LED).

Where to Buy
============

- see this list of `Mateksys Distributors <http://www.mateksys.com/?page_id=1212>`__

Connecting a GPS/Compass module
===============================

This board does not include a GPS or compass so an :ref:`external GPS/compass <common-positioning-landing-page>` should be connected in order for autonomous modes to function.

If the GPS is attached to UART2 TX/RX and powered from the adjacent 4.5V pins, it will be powered when connected via USB, as would the RX if powered from the adjacent 4.5V pins to UART6.

A battery must be plugged in for power to be provided to the pins marked 5V on the board.

Battery Monitor Settings
========================

These should already be set by default. However, if lost or changed:

Enable Battery monitor with these parameter settings :

:ref:`BATT_MONITOR<BATT_MONITOR>` =4

Then reboot.

:ref:`BATT_VOLT_PIN<BATT_VOLT_PIN>` 10

:ref:`BATT_CURR_PIN<BATT_CURR_PIN>` 11

:ref:`BATT_VOLT_MULT<BATT_VOLT_MULT>` 11.0

:ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` 40.0

:ref:`BATT2_VOLT_PIN<BATT2_VOLT_PIN>` 18

:ref:`BATT2_CURR_PIN<BATT2_CURR_PIN>` 7

:ref:`BATT2_VOLT_MULT<BATT2_VOLT_MULT>` 11.0

