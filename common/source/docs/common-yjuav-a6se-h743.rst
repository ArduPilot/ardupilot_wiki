.. _common-yjuav-a6se-h743:

=======================================================
YJUAV A6SE_H743 High Performance Mini Flight Controller
=======================================================

Overview
========
The A6SE_H743 is a high-performance and cost-effective flight control product, with a small size, light weight, and easy installation.

The A6SE_H743 flight controller is manufactured and sold by `YJUAV <http://www.yjuav.net>`__.

.. image:: ../../../images/yjuav/autopilot/a6se_h743/a6se_h743_1.jpg
    :target: ../_images/yjuav/autopilot/a6se_h743/a6se_h743_1.jpg
    :width: 100%

Where to Buy
============

Order `here <https://yjuav.taobao.com/>`__.


Specifications
==============

-  **Processor**

   -  STM32H743 32-bit processor
   -  480 Mhz/ 1 MB RAM
   -  2MB Flash
   -  32KB F-RAM nonvolatile memory

-  **Sensors**

   -  InvenSense ICM42688 accelerometer / gyroscope
   -  InvenSense ICM42688 accelerometer / gyroscope
   -  DPS310 barometer
   -  IST8310 magnetometer

-  **Power**

   -  Power supply: 4.5~5.5V

-  **Interfaces**

   -  11x PWM servo outputs
   -  5x ADC pins
   -  5x Uart ports
   -  3x I2C ports
   -  2x CAN ports
   -  1x SPI port
   -  1x microSD port
   -  1x TypeC USB port
   -  1x JST_GH1.25 USB port
   -  1x Analog battery monitor port
   -  1x RC input (supports SBUS, PPM and DSM)
   -  1x Safety switch and Buzzer port
   -  1x S.Bus servo output

-  **Other**

   -  Weight 40g
   -  Size 58mm x 38mm x 16mm
   -  Operating temperature -40 ~ 85°c

Pinout
======

.. image:: ../../../images/yjuav/autopilot/a6se_h743/a6se_h743_2.jpg
    :target: ../_images/yjuav/autopilot/a6se_h743/a6se_h743_2.jpg

POWER ADC
---------
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC_IN               +5V
2                 VCC_IN               +5V
3                 BAT_CRRENT_ADC       +3.3V
4                 BAT_VOLTAGE_ADC      +3.3V
5                 GND                  GND
6                 GND                  GND
=============     ================     =============

TELEM1
------
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 UART_TX2             +3.3V
3                 UART_RX2             +3.3V
4                 CTS                  +3.3V
5                 RTS                  +3.3V
6                 GND                  GND
=============     ================     =============

TELEM2
------
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 UART_TX6             +3.3V
3                 UART_RX6             +3.3V
4                 CTS                  +3.3V
5                 RTS                  +3.3V
6                 GND                  GND
=============     ================     =============

ADC
---
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 ADC_3V3              +3.3V
3                 ADC_6V6              +6.6V
4                 GND                  GND
=============     ================     =============

SPI
---
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 SPI_SCK              +3.3V
3                 SPI_MISO             +3.3V
4                 SPI_MOSI             +3.3V
5                 SPI_CS               +3.3V
6                 GND                  GND
=============     ================     =============

I2C
---
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 I2C4_SCL              +3.3V
3                 I2C4_SDA              +3.3V
4                 GND                  GND
=============     ================     =============

CAN1
---------
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 CAN1_P                +3.3V
3                 CAN1_N                +3.3V
4                 GND                  GND
=============     ================     =============

CAN2
---------
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 CAN2_P                +3.3V
3                 CAN2_N                +3.3V
4                 GND                  GND
=============     ================     =============

GPS1
----
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 UART_TX3             +3.3V
3                 UART_RX3             +3.3V
4                 I2C2_SCL              +3.3V
5                 I2C2_SDA              +3.3V
6                 GND                  GND
=============     ================     =============

GPS2&SAFETY
-----------
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 UART_TX1             +3.3V
3                 UART_RX1             +3.3V
4                 I2C3_SCL              +3.3V
5                 I2C3_SDA              +3.3V
6                 SAFETY_SW            +3.3V
7                 SAFETY_SW_LED        +3.3V
8                 3V3_OUT              +3.3V
9                 BUZZER               +3.3V
10                GND                  GND
=============     ================     =============

DEBUG
-----
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC                  +5V
2                 UART_TX7             +3.3V
3                 UART_RX7             +3.3V
4                 SWDIO                +3.3V
5                 SWCLK                +3.3V
6                 GND                  GND
=============     ================     =============

SAFETY
------
=============     ==================    =============
Pin               Signal                Volt
=============     ==================    =============
1                 3V3_OUT               +3.3V
2                 SAFETY_SW             +3.3V
3                 SAFETY_SW_LED         +3.3V
4                 UART8_TX(SBUS_OUT)    +3.3V
5                 RSSI                  +3.3V
6                 GND                   GND
=============     ==================    =============

USB
---
=============     ================     =============
Pin               Signal               Volt
=============     ================     =============
1                 VCC_IN               +5V
2                 DM                   +3.3V
3                 DP                   +3.3V
4                 GND                  GND
=============     ================     =============

UART Mapping
============

- SERIAL0 -> USB (OTG1)
- SERIAL1 -> USART2 (Telem1)
- SERIAL2 -> USART6 (Telem2)
- SERIAL3 -> USART3 (GPS1), NODMA
- SERIAL4 -> USART1 (GPS2), NODMA
- SERIAL5 -> UART8 (USER) TX only, normally used for SBUS_OUT with protocol change
- SERIAL6 -> UART7 (USER/Debug), NODMA
- SERIAL7 -> USB2 (OTG2)

.. note:: UART8 TX is output on the SAFETY connector. Its commonly used for SBUS_OUT function by setting :ref:`SERIAL5_PROTOCOL<SERIAL5_PROTOCOL>` = "15". The UART8 RX input is not accessible for use.

RC Input
========

The RCIN pin is mapped to a timer input instead of the UART, and can be used for all ArduPilot supported receiver protocols, except CRSF/ELRS and SRXL2 which require a true UART connection. However, FPort, when connected in this manner, can provide RC without telemetry.

To allow CRSF and embedded telemetry available in Fport, CRSF, and SRXL2 receivers, a full UART must be used. For example, UART1 can have its protocol changed from the default GPS protocol for GPS2 to RX input protocol:

With this option, :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` must be set to “23”, and:

    PPM is not supported.

    SBUS/DSM/SRXL connects to the RX1 pin.

    FPort requires connection to TX1 and RX1. See :ref:`FPort Receivers<common-FPort-receivers>`.

    CRSF also requires a TX1 connection, in addition to RX1, and automatically provides telemetry.

    SRXL2 requires a connection to TX1 and automatically provides telemetry. Set :ref:`SERIAL4_OPTIONS<SERIAL4_OPTIONS>` to “4”.

Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. See :ref:`Radio Control Systems<common-rc-systems>` for details.

PWM Output
==========

The A6SE_H743 supports up to 11 PWM outputs,support all PWM protocols as well as DShot. All 11 PWM outputs have GND on the bottom row, 5V on the middle row and signal on the top row.

The 11 PWM outputs are in 3 groups:

- PWM 1, 2, 3 and 4 in group1
- PWM 5, 6, 7 and 8 in group2
- PWM 9, 10, 11 in group3

Channels 1-8 support bi-directional Dshot.
Channels within the same group need to use the same output rate. If any channel in a group uses DShot, then all channels in that group need to use DShot.

GPIOs
=====
All 11 PWM channels can be used for GPIO functions (relays, buttons, RPM etc).

The pin numbers for these PWM channels in ArduPilot are shown below:

=============     ======     =============     ======
PWM Channels      Pin        PWM Channels      Pin
=============     ======     =============     ======
PWM1              50         PWM8              57
PWM2              51         PWM9              58
PWM3              52         PWM10             59
PWM4              53         PWM11             60
PWM5              54
PWM6              55
PWM7              56
=============     ======     =============     ======

Analog inputs
=============

The A6SE_H743 flight controller has 5 analog inputs

- ADC Pin4   -> Battery Current 
- ADC Pin2   -> Battery Voltage 
- ADC Pin8   -> ADC 3V3 Sense
- ADC Pin10  -> ADC 6V6 Sense
- ADC Pin11  -> RSSI voltage monitoring

Battery Monitor Configuration
=============================
The board has voltage and current sensor inputs on the POWER_ADC connector.

The correct battery setting parameters are:

Enable Battery monitor.

:ref:`BATT_MONITOR<BATT_MONITOR>` = 4

Then reboot.

:ref:`BATT_VOLT_PIN<BATT_VOLT_PIN>` = 2

:ref:`BATT_CURR_PIN<BATT_CURR_PIN>` = 4

:ref:`BATT_VOLT_MULT<BATT_VOLT_MULT>` = 21.0 (may need adjustment if supplied monitor is not used)

:ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` = 34.0 (may need adjustment if supplied monitor is not used)

Compass
=======

There is an on-board compass. Calibration is required (see :ref:`common-compass-calibration-in-mission-planner`).

.. note:: on-board compasses are often close to battery lines and therefore subject to interference and sometimes disabled, with external compasses being located remotely to avoid this issue.

Loading Firmware
================

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of xxxxxx.apj firmware files with any ArduPilot
compatible ground station.

Firmware for these boards can be found `here <https://firmware.ardupilot.org>`_ in  sub-folders labeled "YJUAV_A6SE_H743".

[copywiki destination="plane,copter,rover,blimp"]
