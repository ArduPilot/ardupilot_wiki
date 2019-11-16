.. _common-can-pmu:

==============================
CAN PMU Power detection module
==============================

.. image:: ../../../images/can-pmu/can-pmu.jpg
    :target: ../_images/can-pmu/can-pmu.jpg

Overview
========

CAN PMU is a drone power management module with built-in STM32F4 processor; running CUAV ITT compensation algorithm, can accurately measure the voltage and current of the drone; support 6~62V voltage input, and the POWER port can output 5V/ 8A. It uses advanced CAN bus communication and supports the standard [UAVCAN](https://new.uavcan.org/) protocol; each PMU uses factory-level calibration to ensure good consistency and high accuracy.

Quick Summary
=============

* **processor** 
  * STM32F412
* **Voltage input **
  * 6~62V\(2-15S\)
* **Max current** 
  * 110A
* **Voltage accuracy ** 
  * ±0.05V
* **Current accuracy **
  * ±0.1A
* **Resolution**
  * 0.01A/V
* **Max output power **
  * 6000W/90S
* **Max stable power **
  * 5000W
* **Power port output**
  * 5.4V/5A
* **protocol**
  * UAVCAN
* **Operating temp**
  * -20~+100℃
* **Firmware upgrade**
  * Support
* **calibration**
  * no need
* **Interface Type**
  * IN/OUT:XT90\(Cable）/Amass 8.0\(Module）
  * Power:5025850670
  * CAN: GHR-04V-S
* **Appearance:**
  * Size:46.5mm \* 38.5mm \* 22.5mm
  * Weight:76g

Buy
===

[CUAV store](https://store.cuav.net/index.php)
[Aliexpress](https://www.aliexpress.com/item/4000369700535.html)

Pinouts
=======

.. image:: ../../../images/can-pmu/can-pmu-pinouts-en.png
    :target: ../_images/can-pmu/can-pmu-pinouts-en.png

.. image:: ../../../images/can-pmu/can-pmu-pinouts-en2.png
    :target: ../_images/can-pmu/can-pmu-pinouts-en2.png
    
Connection
==========

.. image:: ../../../images/can-pmu/can-pmu-connection-en.png
    :target: ../_images/can-pmu/can-pmu-connection-en.png
    
**Connection method:**

* Connect the flight control CAN1/2 and the module CAN interface.
* Connect the V5 series power cable to the V5 Flight Control Power2 (if other flight controllers are connect to the Power interface) and the module Power  interface.

Enable CAN PMU
==============

Set the following parameters in the Mission planner's full parameter list and restart after writing:
* CAN\_P1\_DRIVER=1 
* CAN\_P2\_DRIVER=1 
* BATT\_MONITOR=8（If using Battery monitor 1）

.. image:: ../../../images/can-pmu/mp-set.png
    :target: ../_images/can-pmu/mp-set.png

>**Note** Please use the firmware of AC3.6/AP3.9 (included) or higher.

More information
================

[CAN PMU Manual](http://manual.cuav.net/power-module/CAN-PMU.pdf)

[CUAV docs](Doc.cuav.net/power-module/can-pmu)

[UAV CAN](https://new.uavcan.org/)