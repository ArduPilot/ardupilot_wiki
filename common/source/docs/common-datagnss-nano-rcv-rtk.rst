.. _common-datagnss-nano-rcv-rtk:

=======================
NANO Helix RTK receiver
=======================
NANO Helix RTK receiver is a high-precision RTK receiver designed and manufactured by DATAGNSS. 

It's based on the new generation Allystar dual-core GNSS SoC. It supports RTK functionality with a maximum data update rate of 5Hz. 
It includes two models, one is v1.0.0 without compass, and another is Rev.A with compass.

.. image:: ../../../images/datagnss-nano/NANO-RCV-02-main.png
	:target: ../_images/datagnss-nano/NANO-RCV-02-main.png

.. note:: NANO Helix RTK Receiver doesn't support Moving Base mode.

Where to Buy
============

- `DATAGNSS website <https://www.datagnss.com/>`_

Key Features
============

- GPS/QZSS/BDS/GLONASS/GALILEO/IRNSS
- Multi-bands, support L1+L5/L1+L2
- Support RTK,output rate 5Hz
- Standard UART serial interface
- Lightweight only 26g
- High performance antenna

GNSS
====
   - Allystar CYNOSURE IV GNSS SOC
   - Dual core
   - 3D accuracy: **1.5m** CEP
   - RTK accuracy: **2cm** +1PPM(H), 3cm+1PPM(V)

Interface
=========
   - UART, 230400bps default
   - SMA connector for antenna
   - Output rate 5Hz default
   - Main power supply,4.7~5.2V

Protocol
========
   - NMEA-0183 output
   - RTCMv3 input/output

Environment
===========
   - Operating temp. -20~85℃

Dimession and weight
====================
   - 65*30mm
   - 26g

Pin definition
==============
The board is connected to the autopilot via UART interface.

.. image:: ../../../images/datagnss-nano/nano-rcv-02-connector.png
	:target: ../_images/datagnss-nano/nano-rcv-02-connector.png

The 1.25mm pitch 6P connector (from Left PIN1 to PIN6) :

   -  1: GND
   -  2: NC
   -  3: NC
   -  4: Rx
   -  5: Tx
   -  6: 5V

Please note that only Rev.A includes RTK and compass.
If the model support compass, pin2 and pin3 output compass I2C interface.

ArduPilot Setup
===============
For example, NANO Helix RTK Receiver is connected to the autopilot's SERIAL4 port, it should work with :

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 5
- :ref:`GPS1_TYPE <GPS1_TYPE>` = 5

.. note:: if you use 4.5 or earlier firmware, it should be GPS_TYPE = 5.

Configuration
=============
RTK technology requires a base and a rover, with the base placed on the ground, which is referred to as the rover on the drone. 

The data from the base needs to be transmitted to the drone via telemetry radio and inputted into the RTK receiver on the rover.

.. image:: ../../../images/gem1305/setup-rtk-00.png
	:target: ../_images/gem1305/setup-rtk-00.png

NANO RTK Receiver family for UAV:

.. image:: ../../../images/datagnss-nano/NANO-total-for-UAV-800x.png
	:target: ../_images/datagnss-nano/NANO-total-for-UAV-800x.png

Base station setup
==================
Please refer to the following link for the full base station setup guide:
- `Base station setup <https://wiki.datagnss.com/index.php/Faq-how-to-setup-base-or-rover>`__

Rover station (Aircraft) setup
==============================
Please refer to the following link for the full rover setup guide:
- `Rover station setup <https://wiki.datagnss.com/index.php/Faq-how-to-setup-base-or-rover>`__

Resource
========
   - `DATAGNSS WiKi <https://wiki.datagnss.com>`__

More information
================

   - `NANO Helix RTK Receiver <https://www.datagnss.com/collections/evk/products/nano-helix-rtk-receiver>`__
   - `HELIX Antenna for RTK <https://www.datagnss.com/collections/rtk-antenna/products/smart-helix-antenna>`__
   - `RTK Antenna AGR6302G <https://www.datagnss.com/collections/rtk-antenna/products/antenna-agr6302g>`__
   - `AT400 RTK Antenna <https://www.datagnss.com/collections/rtk-antenna/products/at400-multi-band-antenna-for-rtk>`__
