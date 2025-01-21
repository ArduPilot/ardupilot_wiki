.. _common-SkySakuraH743:

================================
SkySakura H743 Flight Controller
================================

The SkySakura H743 is a flight controller produced by [SkySakuraRC]

Features
========
* MCU: STM32H743VIT6, 480MHz
* Gyro1: ICM42688
* Gyro2: IIM42652
* SD Card support
* BEC output: 5V 5A & 12V 5A (MAX 60W total) (switchable 12V)
* Barometer1: DPS310
* Barometer2: ICP20100
* Magnometer: IST8310
* CAN bus support
* 7 UARTS: (USART1, USART2, USART3, UART4, USART6, UART7 with flow control, UART8)
* 2 I2C, I2C1 is used internally.
* 13 PWM outputs (12 motor outputs, 1 led)
* 4-12s wide voltage support

Pinout
======
.. image:: ../../../images/SkySakuraH743_top.png
   :target: ../_images/SkySakuraH743_top.png

.. image:: ../../../images/SkySakuraH743-bottom.png
   :target: ../_images/SkySakuraH743-bottom.png

.. image:: ../../../images/SkySakuraH743-wiring-a.png
   :target: ../_images/SkySakuraH743-wiring-a.png

.. image:: ../../../images/SkySakuraH743-wiring-b.png
   :target: ../_images/SkySakuraH743-wiring-b.png

UART Mapping
============

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

* SERIAL0 -> USB (MAVLink2)
* SERIAL1 -> UART7 (MAVLink2, flow-control-capable)|
* SERIAL2 -> UART1 (MAVLink2, DMA-enabled)|
* SERIAL3 -> UART2 (USER)|
* SERIAL4 -> UART3 (GPS1, DMA-enabled)|
* SERIAL5 -> UART4 (RCIN, DMA-enabled)|
* SERIAL6 -> UART6 (DisplayPort, DMA-enabled)|
* SERIAL7 -> UART8 (ESC-Telemetry, RX8 on ESC connectors, TX8 can be used if protocol is change from ESC telem)|
* SERIAL8 -> USB (SLCAN)

Safety Button
=============
SkySakura H743 supports safety button, with connection via sh1.0 6 pin connector, with buzzer and safety led on the same connector.

Safety button is defaulted to be disabled but can be enabled by setting the following parameter:

* :ref:`BRD_SAFETY_DEFLT<BRD_SAFETY_DEFLT>` = 1

RC Input
========
RC input is configured on UART4 with an sh1.0 connector. It supports all RC protocols except PPM. The SBUS pin on the HD VTX connector is tied directly the UART4 RX. If RC is used on UART4, then the SBUS lead from a DJI VTX must not be connected to an SBUS to prevent the RC from no working.

OSD Support
===========
SkySakura H743 supports HD VTX DisplayPort OSD through UART6 by default. Set :ref:`OSD_TYPE<OSD_TYPE>` = 5.

VTX Power Control
=================
The 12VSW output voltage on the HD VTX connector is controlled by GPIO 85, via RELAY1 by default. Relay 1 Low activates the voltage.

PWM Output
==========
The SkySakura H743 has 13 PWM outputs. M1-M8 are linked to sh1.0 8 pin connectors. The first 8 outputs support bi-directional DShot and DShot. Output 9-13 only support non-DShot protocols and 13 is configured as NEOPIXEL LED by default.

The PWM are in in two groups:

* PWM 1-2 in group1
* PWM 3-4 in group2
* PWM 5-6 in group3
* PWM 7-8 in group4
* PWM 9-12 in group5
* PWM 13 in group6

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

Battery Monitoring
==================
The board has a builtin voltage sensor and external current monitor inputs. The voltage sensor can handle up to 12S LiPo batteries. The current sensor scale's default range is 120A, but will need to be adjusted according to which sensor is used. These inputs are present on the first ESC connector.

A second battery monitor can be also used but must be enabled. Its voltage sensor is capable of reading up to 6.6V maximum and is available on the A3 solder pad. Its current monitor input is on the A4 solder pad.

The default battery setting parameters are:

* :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
* :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
* :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 34
* :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11
* :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 40

A second battery monitor can be enabled:

* set :ref:`BATT2_MONITOR<BATT2_MONITOR>` = 4
* :ref:`BATT2_VOLT_PIN<BATT2_VOLT_PIN__AP_BattMonitor_Analog>` = 12
* :ref:`BATT2_CURR_PIN<BATT2_CURR_PIN__AP_BattMonitor_Analog>` = 13
* :ref:`BATT2_VOLT_MULT<BATT2_VOLT_MULT__AP_BattMonitor_Analog>` = 10
* :ref:`BATT2_AMP_PERVLT<BATT2_AMP_PERVLT__AP_BattMonitor_Analog>` set  to appropriate value for second current sensor

Compass
=======
The SkySakura H743 has a built-in IST8310 compass. Due to motor interference, users often disable this compass and use an external compass attached via the external SDA/SCL pins.

NeoPixel LED
============
PWM13 provides external NeoPixel LED support by default but can be re-configured as a normal motor/servo output.

Firmware
========
Firmware can be found on the `firmware server <https://firmware.ardupilot.org>`__  in the "SkySakuraH743" folders.

Loading Firmware
================
Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
\*.apj firmware files.
