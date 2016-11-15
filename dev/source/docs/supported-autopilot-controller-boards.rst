.. _supported-autopilot-controller-boards:

=====================================
Supported AutoPilot Controller Boards
=====================================

Currently ArduPilot supports the following autopilot boards.

Pixhawk
=======

Next-generation PX4, with more memory, improved sensors and a much
easier-to-use design. See the `product overview <https://store.3drobotics.com/products/3dr-pixhawk#product-description>`__
(store) and :ref:`Pixhawk Overview <copter:common-pixhawk-overview>` (wiki)
for more information.

+--------------------------------------+----------------------------------------------------------------------------------------------------+
| **Purchase**                         | `store.3drobotics.com <https://store.3drobotics.com/products/3dr-pixhawk>`__                       |
+--------------------------------------+----------------------------------------------------------------------------------------------------+
| **Specifications**                   | `Product Description <https://store.3drobotics.com/products/3dr-pixhawk#product-description>`__    |
|                                      |                                                                                                    |
|                                      | **OS:**                                                                                            |
|                                      |                                                                                                    |
|                                      | NuttX                                                                                              |
|                                      |                                                                                                    |
|                                      | **CPU:**                                                                                           |
|                                      |                                                                                                    |
|                                      | 32-bit STM32F427 Cortex M4 core with FPU                                                           |
|                                      |                                                                                                    |
|                                      | 32 bit STM32F103 failsafe co-processor                                                             |
|                                      |                                                                                                    |
|                                      | **Memory:**                                                                                        |
|                                      |                                                                                                    |
|                                      | 168 MHz/256 KB RAM/2 MB Flash                                                                      |
|                                      |                                                                                                    |
|                                      | **Sensors:**                                                                                       |
|                                      |                                                                                                    |
|                                      | ST Micro L3GD20 3-axis 16-bit gyroscope                                                            |
|                                      |                                                                                                    |
|                                      | ST Micro LSM303D 3-axis 14-bit accelerometer / compass (magnetometer)                              |
|                                      |                                                                                                    |
|                                      | Invensense MPU 6000 3-axis                                                                         |
|                                      | accelerometer/gyroscope                                                                            |
|                                      |                                                                                                    |
|                                      | MEAS MS5611 barometer                                                                              |
|                                      |                                                                                                    |
|                                      | **Interfaces:**                                                                                    |
|                                      |                                                                                                    |
|                                      | 5x UART (serial ports), one high-power capable, 2x with HW flow control                            |
|                                      |                                                                                                    |
|                                      | 2x CAN                                                                                             |
|                                      |                                                                                                    |
|                                      | Spektrum DSM / DSM2 / DSM-X®                                                                       |
|                                      | Satellite compatible input up to DX8 (DX9 and above not supported)                                 |
|                                      |                                                                                                    |
|                                      | Futaba S.BUS® compatible input and output                                                          |
|                                      |                                                                                                    |
|                                      | PPM sum signal                                                                                     |
|                                      |                                                                                                    |
|                                      | RSSI (PWM or voltage) input                                                                        |
|                                      |                                                                                                    |
|                                      | I2C®                                                                                               |
|                                      |                                                                                                    |
|                                      | SPI                                                                                                |
|                                      |                                                                                                    |
|                                      | 3.3 and 6.6V ADC inputs                                                                            |
|                                      |                                                                                                    |
|                                      | External microUSB port                                                                             |
|                                      |                                                                                                    |
|                                      | **Power System:**                                                                                  |
|                                      |                                                                                                    |
|                                      | Ideal diode controller with                                                                        |
|                                      | automatic failover                                                                                 |
|                                      |                                                                                                    |
|                                      | Servo rail high-power (7 V) and high-current ready                                                 |
|                                      |                                                                                                    |
|                                      | All peripheral outputs over-current                                                                |
|                                      | protected, all inputs ESD protected                                                                |
|                                      |                                                                                                    |
|                                      | **Weight and Dimensions:**                                                                         |
|                                      |                                                                                                    |    
|                                      | Weight: 38g (1.31oz)                                                                               |
|                                      |                                                                                                    |
|                                      | Width: 50mm (1.96")                                                                                |
|                                      |                                                                                                    |
|                                      | Thickness: 15.5mm (.613")                                                                          |
|                                      |                                                                                                    |
|                                      | Length: 81.5mm (3.21")                                                                             |
+--------------------------------------+----------------------------------------------------------------------------------------------------+
| **Setup**                            | `Pixhawk Overview <http://copter.ardupilot.com/common-pixhawk-overview/#specifications>`__,        |
|                                      | `Powering <http://copter.ardupilot.com/wiki/common-powering-the-pixhawk/>`__                       |
+--------------------------------------+----------------------------------------------------------------------------------------------------+
| **Design files**                     | `Schematic <http://firmware.ardupilot.org/downloads/wiki/pdf_guides/px4fmuv2.4.3_schematic.pdf>`__ |
|                                      |                                                                                                    |
|                                      | `Layout <http://firmware.ardupilot.org/downloads/wiki/pdf_guides/Pixhawk-Open-Hardware.zip>`__     |
+--------------------------------------+----------------------------------------------------------------------------------------------------+

APM2.x
======

`APM2 <http://store.jdrones.com/ArduPilot_MEGA_2_5_p/fcapm25side.htm>`__
is a popular AVR2560 8-bit autopilot.

.. note::

   APM 2.6 is compatible with Copter 3.2 and earlier only (to use
   APM:Copter version 3.3 and later, please purchase a Pixhawk). Plane and
   Rover have full support for APM 2.6 in all existing releases. This board
   is not recommended for any new users.

+--------------------------------------+--------------------------------------------+
| **Purchase**                         | |jdrones_amp2.x|                           |
+--------------------------------------+--------------------------------------------+
| **Specifications**                   | **OS:**                                    |
|                                      |                                            |
|                                      | None                                       |
|                                      |                                            |
|                                      | **CPU:**                                   |
|                                      |                                            |
|                                      | AtMega 2560                                |
|                                      |                                            |
|                                      | **Memory:Sensors:**                        |
|                                      |                                            |
|                                      | 3-axis gyro, accelerometer                 |
|                                      |                                            |
|                                      | High-performance Barometric pressure       |
|                                      | sensor                                     |
|                                      |                                            |
|                                      | MS5611-01BA03                              |
|                                      |                                            |
|                                      | **Interfaces:**                            |
|                                      |                                            |
|                                      | **Power System:**                          |
|                                      |                                            |
|                                      | :ref:`copter:common-apm25-and-26-overview` |
|                                      |                                            |
|                                      | **Weight and Dimensions:**                 |
|                                      |                                            |
|                                      | Weight:                                    |
|                                      |                                            |
|                                      | Width: 40.7 mm                             |
|                                      |                                            |
|                                      | Thickness:                                 |
|                                      |                                            |
|                                      | Length: 66.5mm                             |
+--------------------------------------+--------------------------------------------+
| **Setup**                            | :ref:`copter:common-apm25-and-26-overview` |
+--------------------------------------+--------------------------------------------+
| **Design files**                     | |APM_v25_schematic.pdf|                    |
|                                      |                                            |
|                                      | |APM_v252_RELEASE.zip|                     |
|                                      |                                            |
+--------------------------------------+--------------------------------------------+

.. |APM_v252_RELEASE.zip| replace:: `APM board layout <http://firmware.ardupilot.org/downloads/wiki/pdf_guides/APM_v252_RELEASE.zip>`__
.. |APM_v25_schematic.pdf| replace:: `APM schematic diagram <http://firmware.ardupilot.org/downloads/wiki/pdf_guides/APM_v25_schematic.pdf>`__
.. |jdrones_amp2.x| replace:: `store.jdrones.com <http://store.jdrones.com/ArduPilot_MEGA_2_5_p/fcapm25side.htm>`__  

PX4
===

A 32 bit ARM based autopilot with many advanced features, using the
`NuttX <http://nuttx.org/>`__ real-time operating system. See the :ref:`PX4 Overview <copter:common-px4fmu-overview>`
(wiki) for more information.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | Not available                        |
+--------------------------------------+--------------------------------------+
| Specifications                       | **OS:**                              |
|                                      |                                      |
|                                      |                                      |
|                                      | NuttX                                |
|                                      |                                      |
|                                      | **CPU:**                             |
|                                      |                                      |
|                                      | ARM Cortex-M4F microcontroller       |
|                                      | running at 168MHz with DSP and       |
|                                      | floating-point hardware              |
|                                      | acceleration.                        |
|                                      |                                      |
|                                      | **Memory:**                          |
|                                      |                                      |
|                                      | 1024KB of flash memory, 192KB of RAM |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | MEMS accelerometer and gyro, compass |
|                                      | and barometric pressure sensor.      |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | ?                                    |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | ?                                    |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight: 8.10 g                       |
|                                      |                                      |
|                                      | Width:                               |
|                                      |                                      |
|                                      | Thickness:                           |
|                                      |                                      |
|                                      | Length:                              |
+--------------------------------------+--------------------------------------+
| **Setup**                            | :ref:`copter:common-px4fmu-overview` |
+--------------------------------------+--------------------------------------+
| **Design files**                     | |PX4_home_page|                      |
|                                      |                                      |
|                                      | |PX4_manual|                         |
|                                      |                                      |
|                                      | |PX4_schematic|                      |
|                                      |                                      |
|                                      | |PX4_eagle_1_6|                      |
|                                      |                                      |
|                                      | |PX4_eagle_1_7|                      |
+--------------------------------------+--------------------------------------+


.. |PX4_home_page| replace:: `Module homepage <https://pixhawk.ethz.ch/px4/modules/px4fmu>`__
.. |PX4_manual| replace:: `Manual <https://pixhawk.ethz.ch/px4/_media/modules/px4fmu-manual-v1.6.pdf>`__
.. |PX4_schematic| replace:: `Schematics download <https://pixhawk.ethz.ch/px4/_media/modules/px4fmu-schematic-v1.6.pdf>`__
.. |PX4_eagle_1_6| replace:: `Eagle files for version 1.6 download <http://stuff.storediydrones.com/PX4FMUv1.6.zip>`__
.. |PX4_eagle_1_7| replace:: `Eagle files for version 1.7 download <http://stuff.storediydrones.com/PX4FMUv1.7.zip>`__    



Arsov AUAV-X2
=============

`Arsov AUAV-X2 <http://www.auav.co/product-p/auavx2.htm>`__ is a high
quality, compact, light weight and cost effective alternative to the PX4
V2 or PixHawk autopilots. It is 100% compatible with the PX4 firmware.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `www.auav.co <http://www.auav.co/pro |
|                                      | duct-p/auavx2.htm>`__                |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:**                              |
|                                      |                                      |
|                                      |                                      |
|                                      | NuttX                                |
|                                      |                                      |
|                                      | **CPU:**                             |
|                                      |                                      |
|                                      | STM32F427VI ARM microcontroller      |
|                                      |                                      |
|                                      | STM32F100C8T6 ARM microcontroller    |
|                                      |                                      |
|                                      | **Memory:**                          |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | Gyroscope: ST Micro L3GD20           |
|                                      |                                      |
|                                      | Accelerometer: ST Micro LSM303D      |
|                                      |                                      |
|                                      | Gyro: Invensense MPU 6000            |
|                                      |                                      |
|                                      | MEAS MS5611 barometer                |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | 3 x UART                             |
|                                      |                                      |
|                                      | 1 x CAN                              |
|                                      |                                      |
|                                      | 1 x I2C                              |
|                                      |                                      |
|                                      | 1 x SPI                              |
|                                      |                                      |
|                                      | 2 x ADC                              |
|                                      |                                      |
|                                      | 8 x PWM Receiver Inputs              |
|                                      |                                      |
|                                      | 8 Spare IO Pins                      |
|                                      |                                      |
|                                      | 2 x JTAG connection specifically for |
|                                      | the TC2030-CTX-NL 6-Pin cable        |
|                                      |                                      |
|                                      | micro SD card holder                 |
|                                      |                                      |
|                                      | micro USB connector                  |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | New power supply based on TPS63061   |
|                                      | DC-DC Buck-Boost                     |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight:                              |
|                                      |                                      |
|                                      | Width:                               |
|                                      |                                      |
|                                      | Thickness:                           |
|                                      |                                      |
|                                      | Length:                              |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Manual <http://www.auav.co/v/Public |
|                                      | Downloads/AUAV_X2_Manual.pdf>`__     |
+--------------------------------------+--------------------------------------+
| **Design files**                     | `License <https://github.com/PX4/Har |
|                                      | dware/blob/master/README.md>`__      |
|                                      |                                      |
|                                      | `Main Board Design                   |
|                                      | Files <http://www.auav.co/v/PublicDo |
|                                      | wnloads/AUAV_X2_R01.zip>`__          |
|                                      |                                      |
|                                      | `mIMU Board Design                   |
|                                      | Files <http://www.auav.co/v/PublicDo |
|                                      | wnloads/AUAV_Micro_IMU_V2.zip>`__    |
+--------------------------------------+--------------------------------------+


.. _supported-autopilot-controller-boards_erle-brain2_autopilot:

Erle-Brain 2 autopilot
======================

:ref:`Erle-Brain 2 <copter:common-erle-brain-linux-autopilot>` An
autopilot for making drones and robots powered by Debian/Ubuntu and with
official support for the Robot Operating System (ROS). It has access to
the first app store for drones and robots.

.. note::

   `Erle-Brain 2 <https://erlerobotics.com/blog/product/erle-brain-v2/>`__ is a
   commercial artificial robotic brain that runs APM autopilot. It combines
   a Raspberry Pi 2, a sensor cape and other components in order to achieve
   a complete embedded Linux board.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | |erlebrain_2_purchase|               |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:**                              |
|                                      |                                      |
|                                      |                                      |
|                                      | Linux Debian or Ubuntu               |
|                                      |                                      |
|                                      | **CPU:**                             |
|                                      |                                      |
|                                      | 900MHz quad-core ARM Cortex-A7 CPU   |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | Gravity sensor, gyroscope, digital   |
|                                      | compass, Pressure sensor and         |
|                                      | temperature sensor, ADC for battery  |
|                                      | sensing                              |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | 12x PWM, 1x RC IN, 1x Power Module   |
|                                      | Connector, 1x I2C connector, 1x UART |
|                                      | connector, 4 USB ports, Full HDMI    |
|                                      | port, 10/100 Ethernet, Combined      |
|                                      | 3.5mm audio jack and composite.      |
|                                      |                                      |
|                                      | **Camera (optional):** 5MP Fixed     |
|                                      | focus lens, 2592 x 1944 pixel static |
|                                      | images, supports 1080p30, 720p60 and |
|                                      | 640x480p60/90 video record           |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | Traditional Power modules            |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight: 100 grams                    |
|                                      |                                      |
|                                      | 70x96x20mm (without camera)          |
|                                      |                                      |
|                                      | 70x96x58.3mm (with camera),          |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Documentation <http://erlerobotics. |
|                                      | com/docs/>`__                        |
+--------------------------------------+--------------------------------------+


.. |erlebrain_2_purchase| replace:: `Erle-Brain2 (store) <https://erlerobotics.com/blog/product/erle-brain-v2/>`__



.. _supported-autopilot-controller-boards_erle-brain_autopilot:

Erle-Brain 1 autopilot (discontinued)
=====================================

:ref:`Erle-Brain <copter:common-erle-brain-linux-autopilot>` An
autopilot for making drones powered by Ubuntu and with official support
for the Robot Operating System (ROS). It has access to the first app
store for drones and robots.

.. note::

   Erle-Brain is a commercial autopilot. It combines a BeagleBone
   Black, the :ref:`PixHawk Fire Cape <supported-autopilot-controller-boards_pixhawk_fire_cape_pxf>` (above) and
   other components.

+--------------------------------------+-----------------------------------------------------------------------------+
| **Purchase**                         | `erle-brain <http://erlerobotics.com/blog/product/erle-brain/>`__           |
+--------------------------------------+-----------------------------------------------------------------------------+
| **Specifications**                   | **OS:**                                                                     |
|                                      |                                                                             |
|                                      |                                                                             |
|                                      | Linux Ubuntu                                                                |
|                                      |                                                                             |
|                                      | **CPU:**                                                                    |
|                                      |                                                                             |
|                                      | Cortex-A8 @ 1 GHz,                                                          |
|                                      |                                                                             |
|                                      | **Memory:**                                                                 |
|                                      |                                                                             |
|                                      | 512 MB DDR3 with 4GB of flash memory                                        |
|                                      | (8bit Embedded MMC)                                                         |
|                                      |                                                                             |
|                                      | **Sensors:**                                                                |
|                                      |                                                                             |
|                                      | MPU6000, MPU9250, LSM9DS0,                                                  |
|                                      | MS5611-01BA                                                                 |
|                                      |                                                                             |
|                                      | **Interfaces:**                                                             |
|                                      |                                                                             |
|                                      | SPI, 3xI2C, 2xUART, CAN, Buzzer,                                            |
|                                      | Safety, 8 PWM channels, PPM, S.Bus,                                         |
|                                      | ADC, Specktrum, 2xUSB, Ethernet                                             |
|                                      |                                                                             |
|                                      | **Power System:**                                                           |
|                                      |                                                                             |
|                                      | Traditional Power modules                                                   |
|                                      |                                                                             |
|                                      | **Weight and Dimensions:**                                                  |
|                                      |                                                                             |
|                                      | Weight: 110 grams                                                           |
|                                      |                                                                             |
|                                      | Width: 75 cm                                                                |
|                                      |                                                                             |
|                                      | Thickness PCB: 1.6 mm                                                       |
|                                      |                                                                             |
|                                      | Length: 92 cm                                                               |
+--------------------------------------+-----------------------------------------------------------------------------+
| **Setup**                            | `Updating software <http://erlerobotics.com/blog/updating-the-software/>`__ |
|                                      |                                                                             |
|                                      |                                                                             |
|                                      | :ref:`BeaglePilot                                                           |
|                                      | Project <beaglepilot>` (wiki)                                               |
|                                      |                                                                             |
|                                      |                                                                             |
|                                      | `Building ArduPilot for BeagleBone Black on                                 |
|                                      | Linux <building-for-beaglebone-black-on-linux/>`__  (wiki)                  |
|                                      |                                                                             |
+--------------------------------------+-----------------------------------------------------------------------------+
| **Design files**                     | `Design files <https://github.com/ArduPilot/PXF>`__                         |
|                                      |                                                                             |
|                                      |                                                                             |
|                                      | :ref:`copter:common-erle-brain-linux-autopilot`                             |
+--------------------------------------+-----------------------------------------------------------------------------+


.. _supported-autopilot-controller-boards_pixhawk_fire_cape_pxf:

PixHawk Fire Cape (PXF)
=======================

The PixHawk Fire Cape (PXF) is a daughter board for the :ref:`BeagleBone Black <beaglepilot>`
(BBB) development board that allows to create a fully functional Linux
autopilot for drones. The combination of BBB and PXF allows to a Linux
computer is a fully functional autopilot (one example is the :ref:`Erle-Brain autopilot <supported-autopilot-controller-boards_erle-brain_autopilot>`).

+--------------------------------------+--------------------------------------+
| **Purchase**                         | http://erlerobotics.com/blog/product |
|                                      | /pixhawk-fire-cape/                  |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:**                              |
|                                      |                                      |
|                                      |                                      |
|                                      | Linux Debian, Linux Ubuntu           |
|                                      |                                      |
|                                      | **CPU:**                             |
|                                      |                                      |
|                                      | **Memory:**                          |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | MPU6000, MPU9250, LSM9DS0,           |
|                                      | MS5611-01BA                          |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | SPI, 3xI2C, 2xUART, CAN, Buzzer,     |
|                                      | Safety, 8 PWM channels, PPM, S.Bus,  |
|                                      | ADC, Specktrum                       |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | Traditional Power modules            |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight: 16 grams                     |
|                                      |                                      |
|                                      | Width: 55 cm                         |
|                                      |                                      |
|                                      | Thickness: 1.6 mm                    |
|                                      |                                      |
|                                      |                                      |
|                                      | Length: 85 cm                        |
+--------------------------------------+--------------------------------------+
| **Setup**                            | |erlebrain_updating|                 |
|                                      |                                      |
|                                      |                                      |
|                                      | :ref:`BeaglePilot                    |
|                                      | Project <beaglepilot>` (wiki)        |
|                                      |                                      |
|                                      | `Building ArduPilot for BeagleBone   |
|                                      | Black on                             |
|                                      | Linux <building-for-beaglebone-black |
|                                      | -on-linux/>`__                       |
|                                      | (wiki)                               |
+--------------------------------------+--------------------------------------+
| **Design files**                     | https://github.com/ArduPilot/PXF     |
+--------------------------------------+--------------------------------------+

.. |erlebrain_updating| replace:: `Updating the software <http://erlerobotics.com/blog/updating-the-software/>`__

PixHawk Fire Mini Cape (PXFmini)
================================

The PixHawk Fire Mini Cape (PXFmini) is a daughter board designed for
the low cost `Raspberry Pi zero <https://www.raspberrypi.org/blog/raspberry-pi-zero/>`__ that
allows to create a fully functional Linux autopilot for drones. Inspired
in the PXF cape, provides a minimalist approach which allows having a
reduced size/lightweight and low-cost.

+--------------------------------------+----------------------------------------------------------------------------------------------------------------+
| **Purchase**                         | `pxfmini <https://erlerobotics.com/blog/product/pxfmini/>`__                                                   |
+--------------------------------------+----------------------------------------------------------------------------------------------------------------+
| **Specifications**                   | **OS:**                                                                                                        |
|                                      |                                                                                                                |
|                                      |                                                                                                                |
|                                      | Linux Debian, Linux Ubuntu                                                                                     |
|                                      |                                                                                                                |
|                                      | **CPU:**                                                                                                       |
|                                      |                                                                                                                |
|                                      | **Memory:**                                                                                                    |
|                                      |                                                                                                                |
|                                      | **Sensors:**                                                                                                   |
|                                      |                                                                                                                |
|                                      | MPU9250, MS5611-01BA, ADS1115                                                                                  |
|                                      |                                                                                                                |
|                                      | **Interfaces:**                                                                                                |
|                                      |                                                                                                                |
|                                      | 2xI2C, 1xUART, 1xPPM-SUM, JST-GH                                                                               |
|                                      | type connectors                                                                                                |
|                                      |                                                                                                                |
|                                      | 8xPWM channels                                                                                                 |
|                                      |                                                                                                                |
|                                      | **Power System:**                                                                                              |
|                                      |                                                                                                                |
|                                      | Traditional Power modules                                                                                      |
|                                      |                                                                                                                |
|                                      | **Weight and Dimensions:**                                                                                     |
|                                      |                                                                                                                |
|                                      | Weight: 15 grams                                                                                               |
|                                      |                                                                                                                |
|                                      | Dimension: 31x73mm                                                                                             |
+--------------------------------------+----------------------------------------------------------------------------------------------------------------+
| **Setup**                            | `Setup <http://erlerobotics.com/docs/Artificial_Brains_and_Autopilots/Autopilot_shields/PXFmini/Intro.html>`__ |
+--------------------------------------+----------------------------------------------------------------------------------------------------------------+
| **Design files**                     | To be delivered in February 2016                                                                               |
+--------------------------------------+----------------------------------------------------------------------------------------------------------------+

BBBMINI Cape
============

Low budget DIY Autopilot Cape for BeagleBone Black running ArduPilot on
Linux.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | DIY                                  |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:** Debian Linux                 |
|                                      |                                      |
|                                      |                                      |
|                                      | **CPU:** Cortex-A8 @ 1 GHz           |
|                                      |                                      |
|                                      | **Memory:** 512 MB DDR3 and 4GB of   |
|                                      | flash memory                         |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | MPU9250, MS5611, HC-SR04             |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | 2 x SPI, I2C, 2 x UART, CAN, 12 x    |
|                                      | PWM channels + 3 x PWM for X-Quad    |
|                                      | configuration, PPM, S.Bus, Spektrum  |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | Power module / UBEC                  |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight: 36 grams                     |
|                                      |                                      |
|                                      | Width: 55 mm                         |
|                                      |                                      |
|                                      | Thickness: 1.6 mm                    |
|                                      |                                      |
|                                      | Length: 86 mm                        |
+--------------------------------------+--------------------------------------+
| **Setup**                            | https://github.com/mirkix/BBBMINI    |
+--------------------------------------+--------------------------------------+
| **Design files**                     | https://github.com/mirkix/BBBMINI    |
+--------------------------------------+--------------------------------------+

APM1 (discontinued)
===================

An AVR2560 based autopilot with separate sensor board (aka "oilpan"). As
with APM2 this is no longer supported by Copter. Not recommended for any
new users.

Closed boards
=============

The following boards are known to be closed (they do not publish their
design files).

Parrot Bebop Drone
------------------

The `Bebop Drone <http://www.parrot.com/usa/products/bebop-drone/>`__ is
a Wifi controlled quadrotor UAV that uses `this Linux autopilot <https://us.store.parrot.com/en/accessoires/247-main-board-3520410021619.html>`__
and which can run Copter firmware.

From Copter 3.3 the Bebop can run ArduPilot. Instructions for converting
a Bebop to run ardupilot are
:ref:`here <building-for-bebop-on-linux>`.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `Parrot                              |
|                                      | Store <https://us.store.parrot.com/e |
|                                      | n/bebop-drone/297-bebop-drone-352041 |
|                                      | 0023996.html#/color-red>`__          |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:**                              |
|                                      |                                      |
|                                      | Linux (Busybox)                      |
|                                      |                                      |
|                                      | **CPU:**                             |
|                                      |                                      |
|                                      | Parrot P7 dual-core CPU Cortex 9     |
|                                      | with quad core GPU                   |
|                                      |                                      |
|                                      | **Memory:**                          |
|                                      |                                      |
|                                      | 8GB flash                            |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | MPU6050 for accelerometers and       |
|                                      | gyroscope (I2C),                     |
|                                      |                                      |
|                                      | AKM 8963 compass,                    |
|                                      |                                      |
|                                      | MS5607 barometer,                    |
|                                      |                                      |
|                                      | `Furuno GN-87F                       |
|                                      | GPS <http://www.furuno.com/en/produc |
|                                      | ts/gnss-module/GN-87>`__,            |
|                                      |                                      |
|                                      | Sonar,                               |
|                                      |                                      |
|                                      | Optical-flow,                        |
|                                      |                                      |
|                                      | HD camera                            |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | 1x UART serial ports, USB, Built-in  |
|                                      | Wifi                                 |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | **Weight and Dimensions (with        |
|                                      | hull):**                             |
|                                      |                                      |
|                                      | Weight: 400 grams                    |
|                                      |                                      |
|                                      | Width: 33 cm                         |
|                                      |                                      |
|                                      | Thickness: 38 cm                     |
|                                      |                                      |
|                                      | Length: 36 cm                        |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Building for Bebop on               |
|                                      | Linux <building-for-bebop-on-linux>` |
|                                      | __                                   |
|                                      | (wiki)                               |
+--------------------------------------+--------------------------------------+
| **Design files**                     | NA                                   |
+--------------------------------------+--------------------------------------+

.. note::

   Some of this information was taken from the `Paparazzi UAV wiki
   page on the Bebop <http://wiki.paparazziuav.org/wiki/Bebop>`__.

NAVIO+
------

`NAVIO+ <http://www.emlid.com/>`__ is a sensor cape for the RaspberryPi2
from Emlid. Under rapid development.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | |navio_plus_shop|                    |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:**                              |
|                                      |                                      |
|                                      | Linux Debian                         |
|                                      |                                      |
|                                      | **CPU:**                             |
|                                      |                                      |
|                                      | ?                                    |
|                                      |                                      |
|                                      | **Memory:**                          |
|                                      |                                      |
|                                      | ?                                    |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | MPU9250 9DOF IMU                     |
|                                      |                                      |
|                                      | MS5611 Barometer                     |
|                                      |                                      |
|                                      | U-blox M8N Glonass/GPS/Beidou        |
|                                      |                                      |
|                                      | ADS1115 ADC for power monitoring     |
|                                      |                                      |
|                                      | MB85RC FRAM storage                  |
|                                      |                                      |
|                                      | HAT EEPROM                           |
|                                      |                                      |
|                                      | PCA9685 PWM generator                |
|                                      |                                      |
|                                      | RGB LED                              |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | 13 PWM servo outputs                 |
|                                      |                                      |
|                                      | PPM input                            |
|                                      |                                      |
|                                      | UART, SPI, I2C for extensions        |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | Triple redundant power supply        |
|                                      |                                      |
|                                      | Power module connector               |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight: 24g                          |
|                                      |                                      |
|                                      | Width: 55mm                          |
|                                      |                                      |
|                                      | Thickness: ?                         |
|                                      |                                      |
|                                      | Length: 65mm                         |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Emlid Documentation                 |
|                                      | site <http://docs.emlid.com/Navio-AP |
|                                      | M/hardware-setup-navio-plus/>`__     |
+--------------------------------------+--------------------------------------+
| **Design files**                     | ?                                    |
+--------------------------------------+--------------------------------------+

.. |navio_plus_shop| replace:: `www.emlid.com/shop/navio-plus <http://www.emlid.com/shop/navio-plus/>`__  
 


NAVIO2
-------

`NAVIO2 <http://www.emlid.com/>`__ is a new sensor cape for the RaspberryPi 3
from Emlid.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | |navio2_shop|                        |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:**                              |
|                                      |                                      |
|                                      | Linux Debian                         |
|                                      |                                      |
|                                      | **CPU:**                             |
|                                      |                                      |
|                                      | ?                                    |
|                                      |                                      |
|                                      | **Memory:**                          |
|                                      |                                      |
|                                      | ?                                    |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | MPU9250 9DOF IMU                     |
|                                      |                                      |
|                                      | LSM9DS1 9DOF IMU                     |
|                                      |                                      |
|                                      | MS5611 Barometer                     |
|                                      |                                      |
|                                      | U-blox M8N Glonass/GPS/Beidou        |
|                                      |                                      |
|                                      | RC I/O co-processor                  |
|                                      |                                      |
|                                      | HAT EEPROM                           |
|                                      |                                      |
|                                      | RGB LED                              |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | 14 PWM servo outputs                 |
|                                      |                                      |
|                                      | PPM/S.Bus input                      |
|                                      |                                      |
|                                      | UART, I2C, ADC for extensions        |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | Triple redundant power supply        |
|                                      |                                      |
|                                      | Power module connector               |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight: 23g                          |
|                                      |                                      |
|                                      | Width: 55mm                          |
|                                      |                                      |
|                                      | Thickness: ?                         |
|                                      |                                      |
|                                      | Length: 65mm                         |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Emlid Documentation                 |
|                                      | site <http://docs.emlid.com/navio2/  |
|                                      | Navio-APM/hardware-setup/>`__        |
+--------------------------------------+--------------------------------------+
| **Design files**                     | ?                                    |
+--------------------------------------+--------------------------------------+

.. |navio2_shop| replace:: `www.emlid.com/shop/navio2 <http://www.emlid.com/shop/navio2/>`__  
 


VRBrain
-------

`VRBrain <http://vrbrain.wordpress.com/>`__ is a multipurpose controller
board that comes loaded with a 32 bit version of Copter firmware. At
time of writing the latest version is `VR Brain 5 <http://www.virtualrobotix.it/index.php/en/shop/autopilot/vrbrain5-detail>`__.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `vrbrain.wordpress.com/store/ <https |
|                                      | ://vrbrain.wordpress.com/store/>`__  |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:**                              |
|                                      |                                      |
|                                      |                                      |
|                                      | NuttX                                |
|                                      |                                      |
|                                      | **CPU:**                             |
|                                      |                                      |
|                                      | ARM CortexM4F microcontroller with   |
|                                      | DSP and FPU.                         |
|                                      |                                      |
|                                      | **Memory:**                          |
|                                      |                                      |
|                                      | 1024KB flash memory, 192KB of RAM.   |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | mems accelerometer and gyroscope.    |
|                                      |                                      |
|                                      | barometer with 10 cm resolution.     |
|                                      |                                      |
|                                      | 2 SPI expansion BUS for optional IMU |
|                                      |                                      |
|                                      | 1 sonar input.                       |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | 8 RC Input standard PPM , PPMSUM ,   |
|                                      | SBUS                                 |
|                                      |                                      |
|                                      | 8 RC Output at 490 hz                |
|                                      |                                      |
|                                      | 1 integrated high speed data flash   |
|                                      | for logging data                     |
|                                      |                                      |
|                                      | 1 Can bus 2 i2c Bus                  |
|                                      |                                      |
|                                      | 3 Serial port available one for GPS  |
|                                      | 1 for serial option 1 for serial     |
|                                      | telemetry.                           |
|                                      |                                      |
|                                      | 3 digital switch (ULN2003).          |
|                                      |                                      |
|                                      | Jtag support for onboard realtime    |
|                                      | debugger.                            |
|                                      |                                      |
|                                      | 1 Buzzer output.                     |
|                                      |                                      |
|                                      | 1 Input for control lipo voltage     |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight: ?                            |
|                                      |                                      |
|                                      | Width: 4 cm                          |
|                                      |                                      |
|                                      | Thickness: ?                         |
|                                      |                                      |
|                                      | Length: 6 cm                         |
+--------------------------------------+--------------------------------------+
| **Setup**                            | `Quick Start                         |
|                                      | Guide <https://vrbrain.wordpress.com |
|                                      | /quick-start-guide/>`__              |
+--------------------------------------+--------------------------------------+
| **Design files**                     | ?                                    |
+--------------------------------------+--------------------------------------+

Qualcomm Snapdragon Flight Kit
------------------------------

The `Qualcomm® Snapdragon Flight™ Kit (Developer’s Edition) <http://shop.intrinsyc.com/products/snapdragon-flight-dev-kit>`__
is small (58x40mm) but offers a lot of CPU power and two on-board
cameras. It contains 4 'Krait' ARM cores which run Linux (Ubuntu 14.04
Trusty, by default), and 3 'Hexagon' DSP cores which run the QURT RTOS.
In addition it includes Wi-Fi, Bluetooth connectivity, automotive-grade
GPS and many more features.

Information about using this board with ArduPilot can be found here:
:ref:`Building for Qualcomm Snapdragon Flight Kit <building-for-qualcomm-snapdragon-flight-kit>`, `QURT Port <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_QURT/README.md>`__
(Github) and `QFlight Port <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_Linux/qflight>`__
(Github).

.. warning::

   Due to some rather unusual licensing terms from Intrinsyc we
   cannot distribute binaries of ArduPilot (or any program built with the
   Qualcomm libraries). So you will have to build the firmware
   yourself.

+--------------------------------------+--------------------------------------+
| **Purchase**                         | `shop.intrinsyc.com/products/snapdra |
|                                      | gon-flight-dev-kit <http://shop.intr |
|                                      | insyc.com/products/snapdragon-flight |
|                                      | -dev-kit>`__                         |
+--------------------------------------+--------------------------------------+
| **Specifications**                   | **OS:**                              |
|                                      |                                      |
|                                      |                                      |
|                                      | Ubuntu Linux (Ubuntu 14.04 Trusty by |
|                                      | default)                             |
|                                      |                                      |
|                                      | **System on a Chip:                  |
|                                      | System-on-Chip:** Snapdragon 801     |
|                                      |                                      |
|                                      | CPU: Quad-core 2.26 GHz Krait        |
|                                      |                                      |
|                                      | DSP: Hexagon DSP (QDSP6 V5A) – 801   |
|                                      | MHz+256KL2 (running the flight code) |
|                                      |                                      |
|                                      | GPU: Qualcomm® Adreno™ 330 GPU       |
|                                      |                                      |
|                                      | **Memory:**                          |
|                                      |                                      |
|                                      | RAM: 2GB LPDDR3 PoP @931 MHz         |
|                                      |                                      |
|                                      | Storage: 32GB eMMC Flash             |
|                                      |                                      |
|                                      | **Sensors:**                         |
|                                      |                                      |
|                                      | GPS: Telit Jupiter SE868 V2 module   |
|                                      |                                      |
|                                      | Omnivision OV7251 on Sunny Module    |
|                                      | MD102A-200 (Optic Flow camera -      |
|                                      | 640×480)                             |
|                                      |                                      |
|                                      | Sony IMX135 on Liteon Module         |
|                                      | 12P1BAD11 (4K High Res camera)       |
|                                      |                                      |
|                                      | MPU: Invensense MPU-9250 9-Axis      |
|                                      | Sensor, 3x3mm QFN                    |
|                                      |                                      |
|                                      | Baro: Bosch BMP280 barometric        |
|                                      | pressure sensor                      |
|                                      |                                      |
|                                      | **Interfaces:**                      |
|                                      |                                      |
|                                      | CSR SiRFstarV @ 5Hz via UART         |
|                                      |                                      |
|                                      | uCOAX connector on-board for         |
|                                      | connection to external GPS patch     |
|                                      | antenna                              |
|                                      |                                      |
|                                      | BT/WiFi: BT 4.0 and 2G/5G WiFi via   |
|                                      | QCA6234                              |
|                                      |                                      |
|                                      | Wifi: Qualcomm® VIVE™ 1-stream       |
|                                      | 802.11n/ac with MU-MIMO † Integrated |
|                                      | digital core                         |
|                                      |                                      |
|                                      | 802.11n, 2×2 MIMO with 2 uCOAX       |
|                                      | connectors on-board for connection   |
|                                      | to external antenna                  |
|                                      |                                      |
|                                      | One USB 3.0 OTG port (micro-A/B)     |
|                                      |                                      |
|                                      | Micro SD card slot                   |
|                                      |                                      |
|                                      | Gimbal connector (PWB/GND/BLSP)      |
|                                      |                                      |
|                                      | ESC connector (2W UART)              |
|                                      |                                      |
|                                      | I2C                                  |
|                                      |                                      |
|                                      | 60-pin high speed Samtec             |
|                                      | QSH-030-01-L-D-A-K expansion         |
|                                      | connector                            |
|                                      |                                      |
|                                      | 2x BLSP (BAM Low Speed Peripheral)   |
|                                      |                                      |
|                                      | USB                                  |
|                                      |                                      |
|                                      | **Power System:**                    |
|                                      |                                      |
|                                      | 5VDC via external 2S-6S battery      |
|                                      | regulated down to 5V via APM adapter |
|                                      |                                      |
|                                      | **Weight and Dimensions:**           |
|                                      |                                      |
|                                      | Weight: ?                            |
|                                      |                                      |
|                                      | Width: 58mm for pcb (68 with         |
|                                      | pcb+connectors+camera)               |
|                                      |                                      |
|                                      | Thickness: ?                         |
|                                      |                                      |
|                                      | Length: 40mm for pcb (52 with        |
|                                      | pcb+connectors+camera)Additional     |
|                                      | information can be found at          |
|                                      | `www.intrinsyc.com/qualcomm-snapdrag |
|                                      | on-flight-details/ <http://www.intri |
|                                      | nsyc.com/qualcomm-snapdragon-flight- |
|                                      | details/>`__                         |
|                                      | (behind short survey).               |
+--------------------------------------+--------------------------------------+
| **Setup**                            | ?                                    |
+--------------------------------------+--------------------------------------+
| **Design files**                     | ?                                    |
+--------------------------------------+--------------------------------------+

