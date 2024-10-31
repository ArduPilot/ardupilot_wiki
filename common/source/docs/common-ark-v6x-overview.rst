.. _common-ark-v6x-overview:

=================================
ARKV6X Flight Controller Overview
=================================

.. image:: ../../../images/arkflow/ark_v6x.jpg
    :target: ../_images/ark_v6x.jpg
    :width: 450px

The USA-built ARKV6X flight controller is based on the `FMUV6X and Pixhawk Autopilot Bus open source standards <https://github.com/pixhawk/Pixhawk-Standards>`__.
With triple synced IMUs, data averaging, voting, and filtering is possible. The Pixhawk Autopilot Bus (PAB) form factor enables the ARKV6X to be used on any PAB-compatible carrier board, such as the `ARK Pixhawk Autopilot Bus Carrier <https://arkelectron.com/product/ark-pixhawk-autopilot-bus-carrier/>`__.

Specifications
==============

-  **Sensors**

   -  `Dual Invensense ICM-42688-P IMUs <https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/>`__
   -  `Invensense IIM-42652 Industrial IMU <https://invensense.tdk.com/products/smartindustrial/iim-42652/>`__
   -  `Bosch BMP390 Barometer <https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390/>`__
   -  `Bosch BMM150 Magnetometer <https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm150/>`__

-  **Microprocessor**

   -  `STM32H743IIK6 MCU <https://www.st.com/en/microcontrollers-microprocessors/stm32h743ii.html>`__
    
    -  480Mhz / 1MB RAM / 2MB Flash

-  **Power Requirements**

   -  5V
   -  500mA

    -  300ma for main system
    -  200ma for heater
 
-  **Other**

   -  FRAM
   -  `Pixhawk Autopilot Bus (PAB) Form Factor <https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf>`__
   -  LED Indicators
   -  MicroSD Slot
   -  USA Built
   -  Designed with a 1W heater. Keeps sensors warm in extreme conditions

-  **Additional Information**

   -  Weight: 5.0 g
   -  Dimensions: 3.6 x 2.9 x 0.5 cm

-  **Pinout**

    - For pinout of the ARKV6X see the `DS-10 Pixhawk Autopilot Bus Standard <https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf>`__


Serial Port Mapping
===================

.. list-table:: Serial Port Mapping
   :widths: 15 25 15
   :header-rows: 1

   * - UART
     - Device
     - Port
   * - USART1
     - /dev/ttyS0
     - GPS
   * - USART2
     - /dev/ttyS1
     - TELEM3
   * - USART3
     - /dev/ttyS2
     - Debug Console
   * - UART4
     - /dev/ttyS3
     - UART4 & I2C
   * - UART5
     - /dev/ttyS4
     - TELEM2
   * - USART6
     - /dev/ttyS5
     - PX4IO/RC
   * - UART7
     - /dev/ttyS6
     - TELEM1
   * - UART8
     - /dev/ttyS7
     - GPS2


More Information
================

* `ARKV6X Flight Controller <https://arkelectron.com/product/ark-fpv-flight-controller/>`_

* `ARKV6X documentation <https://arkelectron.gitbook.io/ark-documentation/flight-controllers/arkv6x>`_

[copywiki destination="plane,copter,rover,blimp"]
