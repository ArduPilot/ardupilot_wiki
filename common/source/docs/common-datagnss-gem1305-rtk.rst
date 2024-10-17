.. _common-datagnss-gem1305-rtk:

==================================
GEM1305 RTK receiver with antenna
==================================

GEM1305 is a RTK receiver with antenna designed and manufactured by DATAGNSS. 

GEM1305 is based on the new generation Allystar dual-core GNSS SoC. It supports RTK functionality with a maximum data update rate of 5Hz. 

.. image:: ../../../images/gem1305/datagnss-gem1305-01.png
	:target: ../images/gem1305/datagnss-gem1305-01.png
    

Where to Buy
============

- `DATAGNSS website <https://www.datagnss.com/>`_

Features
========

- Full constellation, multi-frequency GNSS satellite receiver
- Support RTK
- Standard UART serial interface
- Lightweight only 50g

Key Features
=============

=======================       ======================================================================
Receiver                      Allystar CYNOSURE IV GNSS SOC
GNSS                          BDS/GPS/GLONASS/Galileo/QZSS
BAND                          GPS/QZSS L1,L5, BDS B1,B2,GLONASS L1, GALILEO E1/E5a                                 
Position accuracy(RMS)        3D： **1.5m** (Horizontal, 2.5m（Vertical),
                              RTK: **1.5cm** +1PPM（Horizontal), 3.0cm+1PPM（Vertical）                                   
Acquisition                   Cold starts<30S, RTK coverage time<10s
Data update rate              5Hz
Baud rate                     230400bps default
Differential data             RTCM3.X 
Data protocol                 NMEA-0184 V3.0,4.x
Operating Voltage             4.7~5.2V
Operating temperature         -20~85℃
Size                          50*50*12mm
Weigh                         50g
=======================       ======================================================================

Pin definition
==============

The board is connected to the autopilot via UART interface.

The 1.25mm pitch 6P connector :

   -  1: GND
   -  2: NC
   -  3: PPS
   -  4: Rx
   -  5: Tx
   -  6: 5V

Please note that the board only includes RTK.


Package List
============
- GEM1305
- 6P-to-6P cable
