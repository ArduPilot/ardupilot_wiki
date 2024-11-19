.. _common-MFT-SEMA100:

=============================
MFT-SEMA100 Flight Controller
=============================

The MFT-SEMA100 is a flight controller designed and produced by `MFT  Savunma ve Havacılık LTD. ŞTİ. <http://www.mftsavunma.com.tr/>`_

Features
========
* STM32H743 microcontroller
* BMI088 IMU
* BMP390 barometer
* LIB3MDL magnetometer
* MicroSD Card Slot
* 5 UARTs
* 12 PWM outputs
* 2 CANs
* 3 I2Cs


Physical
========
.. image:: ../../../images/MFT-SEMA100_TopView.png
   :target: ../_images/MFT-SEMA100_TopView.png
.. image:: ../../../images/MFT-SEMA100_BottomView.png
   :target: ../_images/MFT-SEMA100_BottomView.png


UART Mapping
============

* SERIAL0 -> USB
* SERIAL1 -> UART1 (MAVLink2, DMA-enabled)
* SERIAL2 -> UART2 (MAVLink2, DMA-enabled)
* SERIAL3 -> UART3 (GPS, DMA-enabled)
* SERIAL4 -> UART5 (GPS2, DMA-enabled)
* SERIAL5 -> UART7 (DMA-enabled)
* SERIAL6 -> UART8 (RX only)

Connectors
==========
All pins are 2.54 mm Pin Headers 
XT30-PW 5V Input for powering the board

RC Input
========

The default RC input is configured on the UART8 RCIN pin. All ArduPilot supported unidirectional RC protocols can be input here except PPM. For bi-directional or half-duplex protocols, such as CRSF/ELRS a full UART will have to be used. For example if SERIAL2 (UART5) is used for bi-directional RC, then:

- :ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>` must be set to "23", and:
- PPM is not supported.
- SBUS/DSM/SRXL connects to the R6 pin.
- FPort requires connection to Tx and :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` be set to "7".
- CRSF/ELRS also requires a Tx connection, in addition to Rx, and automatically provides telemetry. Set :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` to "0".
- SRXL2 requires a connection to Tx and automatically provides telemetry.  Set :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` to "4".

Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. See :ref:`common-rc-systems` for details.

PWM Output
==========

The MFT-SEMA100 supports up to 12 PWM outputs which also support DShot.

PWM outputs are grouped and every group must use the same output protocol:

* 1, 2 are Group 1;
* 3, 4 are Group 2;
* 5, 6, 7, 8 are Group 3;
* 9, 10 are Group 4;
* 11, 12 are Group 5;

Channels within the same group need to use the same output rate. 

GPIOs
=====
The numbering of the GPIOs for PIN variables in ArduPilot is:

* PWM 1 50
* PWM 2 51
* PWM 3 52
* PWM 4 53
* PWM 5 54
* PWM 6 55
* PWM 7 56
* PWM 8 57
* PWM 9 58
* PWM 10 59
* PWM 11 60
* PWM 12 61

Battery Monitoring
==================

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

* BATT_MONITOR 4
* BATT_VOLT_PIN 19
* BATT_CURR_PIN 8
* BATT_VOLT_MULT 10
* BATT_CURR_SCALE 10


Compass
=======

The MFT-SEMA100 has a built-in compass sensor (LIB3MDL), and you can also attach an external compass using I2C on the SDA and SCL connector.

Analog inputs
=============

The IMU heater in the MFT-SEMA100 can be controlled with the BRD_HEAT_TARG parameter, which is in degrees C.


Mechanical
==========

* Mounting: 55 x 56 mm, Φ4 mm
* Dimensions: 64 x 65 x 10 mm
* Weight: 15g

Loading Firmware
================
Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled "MFT-SEMA100".

The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of \*.apj firmware files with any ArduPilot compatible ground station.

[copywiki destination="plane,copter,rover,blimp"]
