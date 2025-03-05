.. _common-NarinFC-H7:

============================
NarinFC-H7 VOLOLAND CO., LTD
============================

Introduction
============

The NarinFC-H7 is a flight controller produced by 'VOLOLAND Co., Ltd. (https://vololand.com)'

NarinFC-H7 is an advanced autopilot family designed in-house by VOLOLAND CO., LTD.

It uses a higher-performance STM32H7 processor and integrates industrial-grade sensors.

Compared with previous autopilots, it has better performance and higher reliability.

.. image:: ../../../images/NarinFC/NarinFC_Header.jpg
  :target: ../../../images/NarinFC/NarinFC_Header.jpg



Features/Specifications
=======================

-  **Processor**
    - STM32H743

-  **Sensors**
    - Accelerometer/Gyroscope: ADIS16470
    - Accelerometer/Gyroscope: ICM-20649
	- Accelerometer/Gyroscope: BMI088
	- Magnetometer: RM3100
	- Barometer: MS5611*2

-  **Interfaces**
    - 14 PWM Output
	- Support multiple RC inputs (SBus / CPPM / DSM)
	- 2 GPS ports (GPS and UART4 ports)
	- 4 ⅹ I2C
	- 2 ⅹ CAN bus ports
	- 2 ⅹ Power ports
	- 2 ⅹ ADC ports
	- 1 ⅹ USB ports

-  **Power**
    - Power 4.3V ~ 5.4V
    - USB Input 4.75V ~ 5.25V

-  **Size and Dimensions**
    - 93.4mm x 46.4mm x 34.1mm
    - 106g



Where to Buy
============
 - 'NarinFC-H7 VOLOLAND CO., LTD. (https://vololand.com/)'



Outline Dimensions
==================
.. image:: ../../../images/NarinFC/2.Outline_Dimensions.png
  :target: ../../../images/NarinFC/2.Outline_Dimensions.png
  :width: 750px



Wire Diagram
============
.. image:: ../../../images/NarinFC/3.Wire_Diagram.png
  :target: ../../../images/NarinFC/3.Wire_Diagram.png
  :width: 750px



UART Mapping (Port Diagram & Pinouts)
=====================================
- SERIAL0 = console = USB (MAVLink2)
- SERIAL1 = Telemetry1 (MAVlink2 default)= USART2 DMA-enabled
- SERIAL2 = Telemetry2 (MAVlink2 default)= USART6 DMA-enabled
- SERIAL3 = GPS1 = USART1
- SERIAL4 = GPS2 = UART4
- SERIAL5 = USER = UART8 (not available except on custom carrier boards) DMA-enabled
- SERIAL6 = USER = UART7
- SERIAL7 = USB2 (Default protocol is MAVLink2)

Serial protocols can be adjusted to personal preferences.

.. image:: ../../../images/NarinFC/4.Port_Diagram_Pin_outs_Diagram-A.png
  :target: ../../../images/NarinFC/4.Port_Diagram_Pin_outs_Diagram-A.png
  :width: 375px

.. image:: ../../../images/NarinFC/4.Port_Diagram_Pin_outs_Diagram-B.png
  :target: ../../../images/NarinFC/4.Port_Diagram_Pin_outs_Diagram-B.png
  :width: 410px

-  **1. TELEM1, TELEM2 Port (JST GH 6P Connector)**
    .. image:: ../../../images/NarinFC/4.1.TELEM1,TELEM2_Port_JST_GH_6P_Connector.png
      :target: ../../../images/NarinFC/4.1.TELEM1,TELEM2_Port_JST_GH_6P_Connector.png

    - JST GH 6P connector
    - TELEMETRY Port

-  **2. CAN1, CAN2 Port (JST GH 4P Connector)**
    .. image:: ../../../images/NarinFC/4.2.CAN1,CAN2_Port_JST_HG_4P_Connector.png
      :target: ../../../images/NarinFC/4.2.CAN1,CAN2_Port_JST_HG_4P_Connector.png

    - JST GH 4P connector
    - Communication Protocol: UAVCAN v0 (default), UAVCAN v1 (limited support)
    - Power Supply: Typically provides 5V or 12V output
    - Pin Configuration: Usually includes CAN_H, CAN_L, VCC, and GND

-  **3. I2C, I2C2, I2C3, I2C4 Port (JST GH 4P Connector)**
    .. image:: ../../../images/NarinFC/4.3.I2C1,I2C2,I2C3,I2C4_Port_JST_GH_4P_Connector.png
      :target: ../../../images/NarinFC/4.3.I2C1,I2C2,I2C3,I2C4_Port_JST_GH_4P_Connector.png

    - JST GH 4P connector
    
-  **4. UART4 Port (JST GH 6P Connector)**
    .. image:: ../../../images/NarinFC/4.4.UART4_Port_JST_GH_6P_Connector.png
      :target: ../../../images/NarinFC/4.4.UART4_Port_JST_GH_6P_Connector.png

    - JST GH 6P connector

-  **5. GPS & Safety Port (JST GH 10P Connector)**
    .. image:: ../../../images/NarinFC/4.5.GPS_Safety_Port_JST_GH_10P_Connector.png
      :target: ../../../images/NarinFC/4.5.GPS_Safety_Port_JST_GH_10P_Connector.png

    - JST GH 10P connector
    - GPS NODMA

-  **6. PWM Out (M1-M14)**
    The NarinFC-H7 supports up to 14 PWM outputs. 
    All outputs support DShot and BiDirDshot. 
    Outputs are grouped and all outputs within their group must be the same protocol.

    .. image:: ../../../images/NarinFC/4.6.PWM_Out_M1-M14.png
      :target: ../../../images/NarinFC/4.6.PWM_Out_M1-M14.png

    - 2.54mm pitch Dupont connector
    - RC_IN : Remote control receiver
    - M1-M14 : PWM OUT

-  **7. Power Input**
    .. image:: ../../../images/NarinFC/4.7.Power_Input.png
      :target: ../../../images/NarinFC/4.7.Power_Input.png

    - 2mm pitch Dupont connector

-  **8. DEBUG Port(JST GH 6P Connector)**
    .. image:: ../../../images/NarinFC/4.8.DEBUG_Port_JST_HG_6P_Connector.png
      :target: ../../../images/NarinFC/4.8.DEBUG_Port_JST_HG_6P_Connector.png

    - JST GH 6P connector
    - DEBUG NODMA

-  **9. USB Port(USB C Type)**

    - USB C Type

-  **10. SPI Port (JST GH 7P Connector)**
    .. image:: ../../../images/NarinFC/4.10.SPI_Port_JST_GH_7P_Connector.png
      :target: ../../../images/NarinFC/4.10.SPI_Port_JST_GH_7P_Connector.png

    - JST GH 7P connector
    - SPI Port

-  **11. SD CARD**

    - SD CARD



PWM Output
==========

The NarinFC-H7 supports up to 14 PWM outputs. All 14 outputs support all normal PWM output formats.All outputs, except 13 and 14, also support DShot.

The 14 PWM outputs are in 4 groups:
  - Outputs 1, 2, 3 and 4 in group1 (these also support Bi-dir DShot if the Bi-Dir firmware version is used)
  - Outputs 5, 6, 7 and 8 in group2
  - Outputs 9, 10, 11 and 12 in group3
  - Outputs 13 and 14 in group4（NO DMA)

ALL outputs within the same group need to use the same output rate and protocol. If any output in a group uses DShot then all channels in that group must use DShot.



Battery Monitor
===============

If you are using an analog battery monitor instead, connect to the Power A connector and set the following parameters 
(if used as second monitor use the BATT2 parameters instead):

- :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
- :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 17
- :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 16
- Set the :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` and :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` as required for the analog PMU used.



RC Input
========

The RCIN pin, which by default is mapped to a timer input, can be used for all ArduPilot supported receiver protocols, except CRSF/ELRS and SRXL2 which require a true UART connection. However, FPort, when connected in this manner, will only provide RC without telemetry. 

To allow CRSF and embedded telemetry available in Fport, CRSF, and SRXL2 receivers, a full UART, such as SERIAL6 (UART7) would need to be used for receiver connections. Below are setups using Serial6.

- :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>` should be set to "23". 
- FPort would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "15". 
- CRSF would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "0". 
- SRXL2 would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "4" and connects only the TX pin. 

Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. 
See :ref:`common-rc-systems` for details.  The power rail associated with this connector position is powered via USB or PMU. 



Loading Firmware
================

This board comes with ArduPilot firmware pre-installed and other vehicle/revision Ardupilot firmware can be loaded using most Ground Control Stations. 

Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled “NarinFC-H7”. 

The board comes pre-installed with an NarinFC-H7 bootloader, allowing the loading of *.apj firmware files with any ArduPilot compatible ground station. 

you can update firmware with Mission Planner.
