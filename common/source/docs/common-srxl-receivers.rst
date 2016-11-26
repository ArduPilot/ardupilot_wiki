.. _common-srxl-receivers:

============================================================
SRXL R/C Receivers
============================================================

This article explains how to use SRXL receivers with ArduPilot.

Overview
========
SRXL is a serial protocol which transfers control data of a R/C receiver to a flight controler or other device. In case of ArduPilot, the R/C receiver transfers servo outputs as a sum signal. ArduPilot is decoding the SRXL datastream of the R/C receiver depending on the SRXL variant identified and extracts the servo output signals. This technique enables the user to connect a SRXL R/C receiver to the pixhawk using a single cable connection similar to a PPM sum signal.

.. note::

   The following SRXL variants are supported by ArduPilot
     #. **MULTIPLEX** SRXL version 1 "12-channel"
     #. **MULTIPLEX** SRXL version 2 "16-channel"
     #. **JR propo** X.BUS Mode B
     #. **SPEKTRUM** SRXL
      
How to use **MULTIPLEX** SRXL Receivers
=======================================








How to use **SPEKTRUM** Receivers
=================================

