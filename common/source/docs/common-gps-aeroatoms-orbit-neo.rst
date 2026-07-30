.. _common-gps-aeroatoms-orbit-neo:

==========================================
AeroAtoms Orbit Neo RTK GNSS + Compass
==========================================

.. image:: ../../../images/aeroatoms/orbit-neo/orbit-neo.png
   :align: center
   :width: 450px

Overview
========

The **AeroAtoms Orbit Neo** is a DroneCAN GNSS module for ArduPilot-compatible vehicles. It combines the u-blox ZED-F9P multi-band GNSS receiver with an integrated magnetometer and barometer, providing a compact navigation solution for multirotors, fixed-wing aircraft, VTOLs and ground vehicles.

Orbit Neo supports concurrent reception of GPS, Galileo, BeiDou, NavIC and GLONASS constellations with RTK positioning capability for centimeter-level accuracy when correction data is available.

The module communicates over DroneCAN and is automatically detected by ArduPilot.

.. note::

   AeroAtoms Orbit Neo is supported by ArduPilot using the DroneCAN protocol.

Key Capabilities
================

* u-blox ZED-F9P multi-band GNSS receiver
* Concurrent reception of GPS, Galileo, BeiDou, NavIC and GLONASS
* RTK positioning support
* DroneCAN communication
* Integrated magnetometer
* Integrated barometer
* Navigation update rate up to 10 Hz

Specifications
==============

.. list-table::
   :widths: 35 65
   :header-rows: 0

   * - GNSS Receiver
     - u-blox ZED-F9-15B
   * - Intended Application
     - UAV / Drone Navigation
   * - GNSS Systems
     - GPS, Galileo, BeiDou, NavIC, GLONASS
   * - GNSS Bands
     - L1C/A, L5, L1OF, E1B/C, E5a, B1I, B2a
   * - RTK Support
     - Yes (Rover)
   * - Magnetometer
     - IIS2MDC / IST8310
   * - Barometer
     - ICP-20100 / DPS368XTSA1
   * - Communication
     - DroneCAN
   * - Navigation Rate
     - Up to 10 Hz
   * - CAN Baud Rate
     - Up to 8 Mbit/s
   * - Supply Voltage
     - 5 V
   * - Current Consumption
     - 150 mA
   * - Base Station Mode
     - Not Supported (Rover Only)

Dimensions
==========

.. image:: ../../../images/aeroatoms/orbit-neo/orbit-neo-dimensions.png
   :align: center
   :width: 450px

Documentation
=============

Complete documentation for the AeroAtoms Orbit Neo is available on the AeroAtoms documentation portal.

The documentation includes:

* Product Overview
* Specifications
* Quick Start Guide
* User Guide
* RTK Rover Setup
* NTRIP Configuration
* Mechanical Dimensions
* Firmware Updates

Official documentation:

* https://docs.aeroatoms.com/orbit-neo/

Support
=======

For documentation, firmware releases and purchasing information, visit:

* Documentation: https://docs.aeroatoms.com
* Store: https://shop.aeroatoms.com
* Website: https://aeroatoms.com