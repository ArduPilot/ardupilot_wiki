.. _common-skydrones-airlink:

===========================
Sky-Drones AIRLink Overview
===========================

.. image:: ../../../images/airlink/airlink-main.jpg

`AIRLink  <https://sky-drones.com/airlink>`__ stands for Artificial Intelligence & Remote Link. The unit includes a cutting-edge drone autopilot, AI mission computer and LTE connectivity unit. AIRLink helps to reduce the time to market for new drone manufacturers from years and months down to weeks.


System Features
===============

.. image:: ../../../images/airlink/airlink-3in1.png

SmartAP AIRLink has two computers and integrated LTE Module: 

   - The flight control computer (autopilot) has a triple-redundant vibration-dampened and temperature-stabilized IMU. 
   - The powerful AI mission computer enables advanced drone software features like computer vision and obstacle avoidance, digital HD video streaming, and payload data streaming.
   - LTE and WiFi connectivity modules provide permanent broadband internet connection which is enabler for remote workflows.

**Feature highlights**

   ..  youtube:: VcBx9DLPN54
    :width: 100%


Specifications
==============


- **Sensors**

   - 3x Accelerometers, 3x Gyroscopes, 3x Magnetometers, 3x Pressure sensorss
   - GNSS, Rangefinders, Lidars, Optical Flow, Cameras
   - 3x-redundant IMU
   - Vibration dampening
   - Temperature stabilization


- **Flight Controller**

   - STM32F7, ARM Cortex M7 with FPU, 216 MHz, 2MB Flash, 512 kB RAM
   - STM32F1, I/O co-processor
   - Ethernet, 10/100 Mbps
   - LAN with AI Mission Computer
   - 8x UARTs: Telemetry 1, Telemetry 2 (AI Mission Computer), Telemetry 3, GPS 1, GPS 2, Extra UART, Serial Debug Console, IO
   - 2x CAN: CAN1, CAN2
   - USB with MAVLink
   - Serial console for debugging   
   - RC Input, SBUS input, RSSI input, PPM input
   - 16x PWM servo outputs (8 from IO, 8 from FMU)
   - 3x I2C ports
   - High-powered piezo buzzer driver
   - High-power RGB LED
   - Safety switch / LED option


- **AI Mission Computer**

   - 6-Core CPU: Dual-Core Cortex-A72 + Quad-Core Cortex-A53
   - GPU Mali-T864, OpenGL ES1.1/2.0/3.0/3.1
   - VPU with 4K VP8/9, 4K 10bits H265/H264 60fps Decoding
   - Remote power control, software reset, power down, RTC Wake-Up, sleep mode
   - RAM Dual-Channel 4GB LPDDR4
   - 16GB eMMC
   - MicroSD up to 256GB
   - Ethernet 10/100/1000 Native Gigabit
   - WiFi 802.11a/b/g/n/ac, Bluetooth
   - USB 3.0 Type C
   - 2x Video: 4-Lane MIPI CSI (FPV Camera) and 4-Lane MIPI CSI with HMDI Input (Payload Camera)

- **LTE Connectivity Module**

   - 4G LTE UMTS/HSPA(+), GSM/GPRS/EDGE
   - 1x External slot, 1x Integrated eSIM 
   - LTE Antenna, 2x2 MIMO
   - Bands: Europe, North America, Australia, Japan, Other

Set content
===========

   ..  youtube:: lex7axW8WQg
    :width: 100%

SmartAP AIRLink set includes everything needed to setup the system and get prepared for the flight. Standard set contains:
   
   - 1x AIRLink Enterprise unit
   - 1x FPV camera with CSI cable
   - 1x WiFi antenna with MMCX connector
   - 2x LTE antenna with MMCX connector
   - 1x HDMI to mini HDMI cable1x set of cables (7 cables for all connectors)
   
`AIRLink Telemetry  <https://sky-drones.com/sets/airlink-telemetry-set.html>`__ based on the Microhard LAN/IP-based RF micromodule is available as an add-on and is fully compatible with AIRLink.


Editions
========

AIRLink editions offer different integration levels required by drone manufacturers: Enterprise and Core. AIRLink Enterprise is ideal for a quick start, evaluation and prototyping while Core is optimised for deep integration and mid-high volume manufacturing. 

**AIRLink Enterprise**

.. image:: ../../../images/airlink/airlink-enterprise.jpg

SmartAP AIRLink's Enterprise edition is intended for prototyping and low to medium volume drone production. Quick and easy installation thanks to the dedicated mounting holes and integrated heatsink for power dissipation.

**AIRLink Core**

SmartAP AIRLink's Core edition is intended for medium to high volume production and deep integration with customer's hardware. It weighs only 89 g and can be attached to a metal frame for optimum cooling.

.. image:: ../../../images/airlink/airlink-core.jpg

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Parameter </th>
   <th>AIRLink Enterprise </th>
   <th>AIRLink Core </th>
   </tr>
   <tr>
   <td>Enclosure</td>
   <td>Aluminum, with integrated heatsink and fan mounting option.</td>
   <td>External heatsink or reasonable power dissipation should be provided by the design.</td>
   </tr>
   <tr>
   <td>Dimensions</td>
   <td>L103 x W61 x H37 mm</td>
   <td>L100 x W57 x H22 mm</td>
   </tr>
   <tr>
   <td>Weight</td>
   <td>198 g</td>
   <td>89 g</td>
   </tr>
   <tr>
   <td>Ambient temperature</td>
   <td>-40°C-..+50°C</td>
   <td>-40°C-..+50°C</td>
   </tr>
   </tbody>
   </table>

Features
========

- **Easy to mount**

.. image:: ../../../images/airlink/airlink-easy-to-mount.jpg


- **FPV camera comes as standard**

.. image:: ../../../images/airlink/airlink-fpv-camera.jpg


Interfaces
==========

**Left side**

.. image:: ../../../images/airlink/airlink-interfaces-left.jpg

Left side interfaces:

   - Power input with voltage & current monitoring
   - AI Mission Computer micro SD card
   - Flight Controller micro SD card
   - AI Mission Computer USB Type-C
   - PPM input, SBUS output, RSSI monitor

- **POWER - JST GH SM10B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>12V</td>
   <td>IN</td>
   <td>+12V</td>
   <td>Main power input</td>
   </tr>
   <tr>
   <td>2</td>
   <td>12V</td>
   <td>IN</td>
   <td>+12V</td>
   <td>Main power input</td>
   </tr>
   <tr>
   <td>3</td>
   <td>12V</td>
   <td>IN</td>
   <td>+12V</td>
   <td>Main power input</td>
   </tr>
   <tr>
   <td>4</td>
   <td>BAT_CURRENT</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Battery current monitoring</td>
   </tr>
   <tr>
   <td>5</td>
   <td>BAT_VOLTAGE</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Battery voltage monitoring</td>
   </tr>
   <tr>
   <td>6</td>
   <td>3V3</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>3.3V output</td>
   </tr>
   <tr>
   <td>7</td>
   <td>PWR_KEY</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Power key input</td>
   </tr>
   <tr>
   <td>8</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   <tr>
   <td>9</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   <tr>
   <td>10</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>

- **CPU SD card - microSD**

- **CPU USB - USB Type C**

- **RC Connector - JST GH SM06B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>5V output</td>
   </tr>
   <tr>
   <td>2</td>
   <td>PPM_IN</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>PPM input</td>
   </tr>
   <tr>
   <td>3</td>
   <td>RSSI_IN</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>RSSI input</td>
   </tr>
   <tr>
   <td>4</td>
   <td>FAN_OUT</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Fan output</td>
   </tr>
   <tr>
   <td>5</td>
   <td>SBUS_OUT</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>SBUS output</td>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>

- **FMU SD card - microSD**

**Right side**

.. image:: ../../../images/airlink/airlink-interfaces-right.jpg

Right side interfaces:

   - Ethernet port with power output
   - Telemetry port
   - Second GPS port
   - Spare I2C / UART port
   - Flight controller USB Type-C
   - Micro SIM Card
   - HDMI input port (payload camera)

- **ETHERNET - JST GH SM08B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Radio module power supply</td>
   </tr>
   <tr>
   <td>2</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Radio module power supply</td>
   </tr>
   <tr>
   <td>3</td>
   <td>ETH_TXP</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>Ethernet transmit positive</td>
   </tr>
   <tr>
   <td>4</td>
   <td>ETH_TXN</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>Ethernet transmit negative</td>
   </tr>
   <tr>
   <td>5</td>
   <td>ETH_RXP</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Ethernet receive positive</td>
   </tr>
   <tr>
   <td>6</td>
   <td>ETH_RXN</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Ethernet receive negative</td>
   </tr>
   <tr>
   <td>7</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   <tr>
   <td>8</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>

- **TEL3 - JST GH SM06B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Power supply output</td>
   </tr>
   <tr>
   <td>2</td>
   <td>USART2_TX</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>Telemetry 3 TX</td>
   </tr>
   <tr>
   <td>3</td>
   <td>USART2_RX</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Telemetry 3 RX</td>
   </tr>
   <tr>
   <td>4</td>
   <td>USART2_CTS</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Telemetry 3 CTS</td>
   </tr>
   <tr>
   <td>5</td>
   <td>USART2_RTS</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>Telemetry 3 RTS</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>

- **I2C3 / UART4 - JST GH SM06B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Power supply output</td>
   </tr>
   <tr>
   <td>2</td>
   <td>USART4_TX</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>UART 4 TX</td>
   </tr>
   <tr>
   <td>3</td>
   <td>USART4_RX</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>UART 4 RX</td>
   </tr>
   <tr>
   <td>4</td>
   <td>I2C3_SCL</td>
   <td>I/O</td>
   <td>+3.3V</td>
   <td>I2C3 Clock</td>
   </tr>
   <tr>
   <td>5</td>
   <td>I2C3_SDA</td>
   <td>I/O</td>
   <td>+3.3V</td>
   <td>I2C3 Data</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>

- **GPS2 - JST GH SM06B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Power supply output</td>
   </tr>
   <tr>
   <td>2</td>
   <td>USART8_TX</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>UART 8 TX</td>
   </tr>
   <tr>
   <td>3</td>
   <td>USART8_RX</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>UART 8 RX</td>
   </tr>
   <tr>
   <td>4</td>
   <td>I2C2_SCL</td>
   <td>I/O</td>
   <td>+3.3V</td>
   <td>I2C2 Clock</td>
   </tr>
   <tr>
   <td>5</td>
   <td>I2C2_SDA</td>
   <td>I/O</td>
   <td>+3.3V</td>
   <td>I2C2 Data</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>


- **FMU USB - USB Type C**

- **SIM Card - micro SIM**

- **HDMI - mini HDMI**



**Front side**

.. image:: ../../../images/airlink/airlink-interfaces-front.jpg

Front side interfaces:

   - Main GNSS and compass port
   - Main telemetry port
   - CSI camera input
   - CAN 1
   - CAN 2


- **TEL1 - JST GH SM06B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Power supply output</td>
   </tr>
   <tr>
   <td>2</td>
   <td>USART7_TX</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>Telemetry 1 TX</td>
   </tr>
   <tr>
   <td>3</td>
   <td>USART7_RX</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Telemetry 1 RX</td>
   </tr>
   <tr>
   <td>4</td>
   <td>USART7_CTS</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Telemetry 1 CTS</td>
   </tr>
   <tr>
   <td>5</td>
   <td>USART7_RTS</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>Telemetry 1 RTS</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>


- **GPS1 - JST GH SM10B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Power supply output</td>
   </tr>
   <tr>
   <td>2</td>
   <td>USART1_TX</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>GPS 1 TX</td>
   </tr>
   <tr>
   <td>3</td>
   <td>USART1_RX</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>GPS 1 RX</td>
   </tr>
   <tr>
   <td>4</td>
   <td>I2C1_SCL</td>
   <td>I/O</td>
   <td>+3.3V</td>
   <td>Mag 1 Clock</td>
   </tr>
   <tr>
   <td>5</td>
   <td>I2C1_SDA</td>
   <td>I/O</td>
   <td>+3.3V</td>
   <td>Mag 1 Data</td>
   </tr>
   <tr>
   <td>6</td>
   <td>SAFETY_BTN</td>
   <td>IN</td>
   <td>+3.3V</td>
   <td>Safety button</td>
   </td>
   </tr>
   <tr>
   <td>7</td>
   <td>SAFETY_LED</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>Safety LED</td>
   </td>
   </tr>
   <tr>
   <td>8</td>
   <td>+3V3</td>
   <td>OUT</td>
   <td>+3.3V</td>
   <td>3.3V output</td>
   </td>
   </tr>
   <tr>
   <td>9</td>
   <td>BUZZER</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Buzzer output</td>
   </td>
   </tr>
   <tr>
   <td>10</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>


- **CAN1 - JST GH SM04B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Power supply output</td>
   </tr>
   <tr>
   <td>2</td>
   <td>CAN1_H</td>
   <td>I/O</td>
   <td>+5V</td>
   <td>CAN 1 High (120Ω)</td>
   </tr>
   <tr>
   <td>3</td>
   <td>CAN1_L</td>
   <td>I/O</td>
   <td>+5V</td>
   <td>CAN 1 Low (120Ω)</td>
   </tr>
   <tr>
   <td>4</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>


- **CAN2 - JST GH SM04B-GHS-TB**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin number </th>
   <th>Pin name </th>
   <th>Direction </th>
   <th>Voltage </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>1</td>
   <td>5V</td>
   <td>OUT</td>
   <td>+5V</td>
   <td>Power supply output</td>
   </tr>
   <tr>
   <td>2</td>
   <td>CAN2_H</td>
   <td>I/O</td>
   <td>+5V</td>
   <td>CAN 2 High (120Ω)</td>
   </tr>
   <tr>
   <td>3</td>
   <td>CAN2_L</td>
   <td>I/O</td>
   <td>+5V</td>
   <td>CAN 2 Low (120Ω)</td>
   </tr>
   <tr>
   <td>4</td>
   <td>GND</td>
   <td></td>
   <td></td>
   <td>Ground</td>
   </tr>
   </tbody>
   </table>

- **CAMERA - FPC 30 pin, 0.5mm pitch**


**Rear side**

.. image:: ../../../images/airlink/airlink-interfaces-back.jpg

Rear side interfaces:

   - SBUS input
   - 16 PWM output channels
   - 2x LTE antenna sockets (MIMO)
   - WiFi antenna socket (AP & Station modes)


UART Order
==========

AIRLink has a large number of internal and external serial ports:

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Serial </th>
   <th>UART </th>
   <th>Function </th>
   </tr>
   <tr>
   <td>Serial 0</td>
   <td>USB</td>
   <td>Console</td>
   </tr>
   <tr>
   <td>Serial 1</td>
   <td>UART 7</td>
   <td>Telemetry 1</td>
   </tr>
   <tr>
   <td>Serial 2</td>
   <td>UART 5</td>
   <td>Telemetry 2 (used internally with Mission Computer)</td>
   </tr>
   <tr>
   <td>Serial 3</td>
   <td>USART 1</td>
   <td>GPS 1</td>
   </tr>
   <tr>
   <td>Serial 4</td>
   <td>UART 8</td>
   <td>GPS 2</td>
   </tr>
   <tr>
   <td>Serial 5</td>
   <td>USART 3</td>
   <td>Debug console (internal connector)</td>
   </tr>
   <tr>
   <td>Serial 6</td>
   <td>USART 2</td>
   <td>Telemetry 3</td>
   </tr>
   <tr>
   <td>Serial 7</td>
   <td>UART 4</td>
   <td>External UART</td>
   </tr>
   </tbody>
   </table>

RC Input
========

.. image:: ../../../images/airlink/airlink-rc-input.jpg

RC input is configured on the SBUS pin and is connected to IO MCU via an inverter internally. 
For PPM receivers please use RC Connector PPM pin located on the left side of the unit. 

Outputs
=======

AIRLink has 16 PWM ouputs. Main outputs 1-8 and connected to IO MCU. AUX outputs 1-8 are connected to FMU. 

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Output </th>
   <th>Timer </th>
   <th>Channel </th>
   </tr>
   <tr>
   <td>AUX 1</td>
   <td>Timer 1</td>
   <td>Channel 4</td>
   </tr>
   <tr>
   <td>AUX 2</td>
   <td>Timer 1</td>
   <td>Channel 3</td>
   </tr>
   <tr>
   <td>AUX 3</td>
   <td>Timer 1</td>
   <td>Channel 2</td>
   </tr>
   <tr>
   <td>AUX 4</td>
   <td>Timer 1</td>
   <td>Channel 1</td>
   </tr>
   <tr>
   <td>AUX 5</td>
   <td>Timer 4</td>
   <td>Channel 2</td>
   </tr>
   <tr>
   <td>AUX 6</td>
   <td>Timer 4</td>
   <td>Channel 3</td>
   </tr>
   <tr>
   <td>AUX 7</td>
   <td>Timer 12</td>
   <td>Channel 1</td>
   </tr>
   <tr>
   <td>AUX 8</td>
   <td>Timer 12</td>
   <td>Channel 2</td>
   </tr>
   </tbody>
   </table>

DShot capabiltiy can be used on the first four AUX pins.

More Information
================

For more information and instructions on setting up and using the AIRLink system see  `AIRLink Documentation  <https://docs.sky-drones.com/airlink/>`__

For technical help, support and customization please get in touch at `Sky-Drones contact page  <https://sky-drones.com/contact-us>`__

More information can be found at  `www.sky-drones.com  <https://sky-drones.com>`__

Frequently asked questions are answered in `FAQ  <https://docs.sky-drones.com/airlink/faq>`__

Reference design
================

.. image:: ../../../images/airlink/airlink-reference-design.png

AIRLink CAD model is available `here  <https://docs.sky-drones.com/airlink/cad-model>`__. 

AIRLink Reference design can be provided by request. Get in touch at `Sky-Drones contact page  <https://sky-drones.com/contact-us>`__

Firmware
========

Firmware for AIRLink can be found `here  <https://firmware.ardupilot.org/>`__ in sub-folders labeled “AIRLink”.


Where to Buy
============

Purchase from the original Sky-Drones Store `here  <https://sky-drones.com/sets/airlink-enterprise-set.html>`__.

 - Worldwide shipping with 1-2 days order processing time
 - Distributors information coming soon

[copywiki destination="plane,copter,rover,blimp"]