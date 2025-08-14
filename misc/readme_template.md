
===
XYZ
===
The XYZ autopilot is manufactured by `XYZ Corp <https://xyz.com>`__

![XYZ](XYZ_board_image.png)

Where to Buy
============

`XYZ Co <https://XYZ.shop.com>`__

Specifications
==============
- Processor

  - STM32H743 32-bit processor, 480Mhz
  - STM32F103 IOMCU
  - 2MB Flash
  - 1MB RAM
  - MAX7456 OSD
- Sensors

  - Invensense IIM-42653 Industrial IMU with heater resistor
  - Bosch BMP390 Barometer
  - ST IIS2MDC Magnetometer

- Interfaces

  - Micro SD card
  - USB-C
  - 14 PWM (14th output defaulted for NeoPixel LED)
  - 6 UARTS, two with flow control
  - Dual CAN
  - ESC Connector with current sense and telemetry inputs
  - Analog Video In/Out with switchable camera inputs
  - HD VTX connector
  - Debug port

- Power

  - Integrated voltage/current power monitor5.5V - 54V (2S - 12S) input
  - 12V GPIO controlled Video power BEC, 2A output
  - 5V, 2A output for baord and peripheral

- Dimensions

  - Size: 3.6 × 3.6 × 0.8 cm
  - Weight: 7.5g with MicroSD card

Pinout
======

![XYZ](XYZ_image_showing_all_pads-connectors.png)

<!--In addition pin tables and/or connector images can be inserted here..see UART table below for format all user accessible connection points should be clear --> 

UART Mapping
============
The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn


|Port  | UART    |Protocol        |TX DMA |RX DMA |
|------|---------|----------------|-------|-------|
|0     |  USB    |  MAVLink2      |  ✘    |   ✘   |
|1     |  USART7 |  MAVLink2      |  ✔    |   ✔   |
|2     |  UART5  |  MAVLink2      |  ✔    |   ✔   |
|3     |  USART1 |  GPS           |  ✔    |   ✔   |
|4     |  USART2 |  GPS           |  ✘    |   ✘   |
|5     |  UART4  |  DisplayPort   |  ✘    |   ✘   |
|6     |  USART3 |  RCin          |  ✔    |   ✔   |
|7     |   USB   |  SLCAN         |  ✘    |   ✘   |

USART7 and USART 2 have flow control pins.

RC input
========
<!--This is the most difficult section and varies widely.-->

The SBUS pin, can be used for all ArduPilot supported receiver protocols, except CRSF/ELRS and SRXL2 which require a true UART connection. However, FPort, if connected to the SBUS pin, will only provide RC without telemetry since FPort includes SBUS signaling multiplexed with telemetry which will not be decoded (see below). 

To allow CRSF/ELRS and embedded telemetry available in Fport, CRSF, and SRXL2 receivers, a full UART, such as SERIAL6 (UART4) would need to be used for receiver connections. Below are setups using SERIAL6.

- :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>` should be set to "23".
- FPort would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "15".
- CRSF/ELRS would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "0".
- SRXL2 would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "4" and connects only the TX pin.
Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. See :ref:`common-rc-systems` for details.

PWM Outputs
===========
The XYZ controller supports up to 14 PWM outputs.

First 8 outputs (labelled 1 to 8) are controlled by a dedicated STM32F103 IO controller. The remaining 6 outputs (labelled 9 to 14) are the "auxiliary" outputs. These are directly attached to the STM32H753 FMU controller .
All 14 outputs support normal PWM output formats. All outputs support DShot, except 13 which only supports PWM. Outputs 1-4 support Bi-Directional DShot.

The 8 IO PWM outputs are in 4 groups:

- Outputs 1 and 2 in group1
- Outputs 3 and 4 in group2
- Outputs 5, 6, 7 and 8 in group3
  
The 6 FMU PWM outputs are in 4 groups:

- Outputs 9 -13 in group1
- Outputs 14 in group2 and defaults to NEOPIXEL Serial LED

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

OSD Support
===========
Onboard OSD using MAX7456 driver) is supported by default. Simultaneously, DisplayPort OSD is also available on the HD VTX connector.

HD VTX Support
==============
The SH1.0-6P connector supports a DJI Air Unit / HD VTX connection. Protocol defaults to DisplayPort. Pin 1 of the connector is 12v so be careful not to connect this to a peripheral requiring 5v. DisplayPort OSD is enabled by default on SERIAL5.

VTX power control
=================
GPIO 81 controls the VTX BEC output to pins marked "12V" and is included on the HD VTX connector. Setting this GPIO low removes voltage supply to this pin/pad. By default RELAY2 is configured to control this pin and sets the GPIO high at boot.

Camera Switch
=============
GPIO 82 controls which camera input, CAM1 or CAM2 is used. By default RELAY3 is configured to control this.

Compass
=======
An on-board IIS2MDC compass is provided. However, often will disabled this compass and use an external one remotely loocated to avoid power circuitry interference using the SDA and SCL I2C lines provided.

RSSI
====
If the RSSI pin is used for analog RSSI input. Set :ref:`RSSI_ANA_PIN<RSSI_ANA_PIN>` to 91. Set :ref:`RSSI_TYPE<RSSI_TYPE>` to "3" if the RC protocol provides rssi data.

Analog Airspeed
===============
If the ARSPD pin is used for analog airspeed  input. Set :ref:`ARSPD_PIN<ARSPD_PIN>` to 92. Set :ref:`ARSPD_TYPE<ARSPD_TYPE>` to "2".

GPIOs
=====


|Pin           |GPIO Number |
|--------------|------------|
|PWM(9)        | 50         |
|PWM(10)       | 51         |
|PWM(11)       | 52         |
|PWM(12)       | 53         |
|PWM(13)       | 54         |
|PWM(14)       | 55         |
|FMU_CAP1      | 58         |
|NFC_GPIO      | 60         |
|ALARM         | 77         |
|HEATER_EN     | 80         |
|VTX PWR       | 81         |
|CAM SW        | 82         |
|MAIN(1)       | 101        |
|MAIN(2)       | 102        |
|MAIN(3)       | 103        |
|MAIN(4)       | 104        |
|MAIN(5)       | 105        |
|MAIN(6)       | 106        |
|MAIN(7)       | 107        |
|MAIN(8)       | 108        |

Battery Monitor
===============
The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11 (CURR pin)
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.0
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 40

Firmware
========
Firmware for the XYZ is available from [ArduPilot Firmware Server](https://firmware.ardupilot.org) under the `XYZ` target.

Loading Firmware
================
To flash firmware initially, connect USB while holding the bootloader button and use DFU to load the `with_bl.hex` file. Subsequent updates can be applied using `\.apj` files through a ground station.
