.. _common-CUAV-7-Nano:

=============================
CUAV-7-Nano Flight Controller
=============================

The CUAV-7-Nano flight controller produced by `CUAV <https://www.cuav.net>`_.

Features
========

* STM32H753 microcontroller
* 2 IMUs: IIM42652 and BMI088
* builtin IST8310 magnetometer
* 2 barometers: BMP581 and ICP20100
* microSD card slot
* USB-TypeC port
* 1 ETH network interface
* 5 UARTs plus USB
* 14 PWM outputs
* 3 I2C ports
* 3 CAN ports (two of which share a CAN bus and one is an independent CAN bus)
* Analog RSSI input
* 3.3V/5V configurable PWM ouput voltage

Pinout
======

.. image:: ../../../images/CUAV-7-Nano-pinout.png
   :target: ../_images/CUAV-7-Nano-pinout.png

UART Mapping
============

* SERIAL0 -> USB
* SERIAL1 -> UART7 (TELEM1)
* SERIAL2 -> UART5 (TELEM2)
* SERIAL3 -> USART1 (GPS&SAFETY)
* SERIAL4 -> UART8 (GPS2)
* SERIAL5 -> USART3 (FMU DEBUG)

The TELEM1 and TELEM2 ports have RTS/CTS pins, the other UARTs do not have RTS/CTS. All have full DMA capability.

RC Input
========

RC input is configured on the RCIN pin, at one end of the servo rail, marked RCIN in the above diagram. All ArduPilot supported unidirectional RC protocols can be input here including PPM. For bi-directional or half-duplex protocols, such as CRSF/ELRS a full UART will have to be used. For example if SERIAL2 (UART5) is used for bi-directional RC, then:

- :ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>` must be set to "23", and:

- PPM is not supported.

- SBUS/DSM/SRXL connects to the R6 pin.

- FPort requires connection to Tx and :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` be set to "7".

- CRSF/ELRS also requires a Tx connection, in addition to Rx, and automatically provides telemetry. Set :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` to "0".

- SRXL2 requires a connection to Tx and automatically provides telemetry.  Set :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` to "4".

Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. See :ref:`common-rc-systems` for details.

PWM Output
==========

The CUAV-7-Nano flight controller supports up to 14 PWM outputs.

The 14 PWM outputs are in 6 groups:


* PWM 1-4 in group1 (TIM5)
* PWM 5 and 6 in group2 (TIM4)
* PWM 7 and 8 in group3 (TIM1)
* PWM 9, 10 and 11 in group4 (TIM8)
* PWM 12 in group5 (TIM15)
* PWM 13 and 14 in group6 (TIM12)

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot. Outputs 1-4 support BDShot.

First first 8 PWM outputs of CUAV-7-Nano flight controller support switching between 3.3V voltage and 5V voltage output. It can be switched to 5V by setting GPIO 80 high by setting up a Relay (see :ref:`common-relay`) to control it.

Battery Monitoring
==================

The board has a dedicated power monitor ports on 6 pin connectors(POWER A). The correct battery setting parameters are dependent on the type of power brick which is connected.
Parameters for the supplied powe monitor are already set by default. However, if lost or changed:

Enable Battery monitor with these parameter settings :

- :ref:`BATT_MONITOR<BATT_MONITOR>` = 4

Then reboot.

- :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN>` 9
- :ref:`BATT_CURR_PIN<BATT_CURR_PIN>` 8
- :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT>` 10.1
- :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` 17


Compass
=======

The CUAV-7-Nano has an IST8310 builtin compass, but due to interference the board is usually used with an external I2C compass as part of a GPS/Compass combination.

Analog inputs
=============

The CUAV-7-Nano has 6 analog inputs.


* ADC Pin9 -> Battery Voltage
* ADC Pin8 -> Battery Current Sensor
* ADC Pin5 -> Vdd 5V supply sense
* ADC Pin13 -> ADC 3.3V Sense
* ADC Pin12 -> ADC 6.6V Sense
* ADC Pin10 -> RSSI voltage monitoring

Loading Firmware
================

Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled "CUAV-7-Nano".

The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of \*.apj firmware files with any ArduPilot compatible ground station.

[copywiki destination="plane,copter,rover,blimp"]
