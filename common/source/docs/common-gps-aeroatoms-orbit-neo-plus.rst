.. _common-gps-aeroatoms-orbit-neo-plus:

==================================
AeroAtoms Orbit Neo Plus
==================================

.. image:: ../../../images/aeroatoms/orbit-neo-plus/orbit-neo-plus.png
   :align: center
   :width: 350px

Overview
========

The **AeroAtoms Orbit Neo Plus** is a high-precision RTK GNSS navigation module
designed for ArduPilot-compatible vehicles. It integrates the u-blox ZED-F9P
GNSS receiver, IST8310 magnetometer, and DPS368XTSA1 barometer to provide
centimeter-level positioning for multirotors, fixed-wing aircraft, VTOLs, and
ground vehicles.

Compared to the Orbit Neo, Orbit Neo Plus adds **Moving Baseline** support for
dual-antenna heading, along with an integrated **LNA + SAW filter** and **EMI
shielding** for improved GNSS performance in challenging RF environments.

Key Capabilities
================

* u-blox ZED-F9P multi-band RTK GNSS receiver
* STM32H743 high-performance microcontroller
* Centimeter-level RTK positioning
* Moving Baseline support for dual-antenna heading
* Concurrent reception of GPS, Galileo, BeiDou, NavIC and GLONASS
* Integrated LNA + SAW filter
* Integrated EMI shielding
* Integrated IST8310 magnetometer
* Integrated DPS368XTSA1 barometer
* DroneCAN communication
* Navigation update rate up to 10 Hz

Specifications
==============

.. list-table::
   :widths: 35 65
   :header-rows: 0

   * - GNSS Receiver
     - u-blox ZED-F9P-15B
   * - Intended Application
     - UAV / Drone Navigation
   * - MCU
     - STM32H743
   * - GNSS Systems
     - GPS, Galileo, BeiDou, NavIC, GLONASS
   * - GNSS Bands
     - L1C/A, L5, L1OF, E1B/C, E5a, B1I, B2a
   * - RTK Support
     - Yes (Centimeter-level Accuracy)
   * - Moving Baseline
     - Yes (Dual-Antenna Heading)
   * - Horizontal Accuracy
     - Centimeter-level with RTK
   * - Signal Enhancement
     - LNA + SAW Filter
   * - EMI Shielding
     - Integrated
   * - Magnetometer
     - IST8310
   * - Barometer
     - DPS368XTSA1
   * - Communication
     - DroneCAN
   * - Navigation Rate
     - Up to 10 Hz
   * - CAN Baud Rate
     - Up to 8 Mbit/s
   * - Supply Voltage
     - 5 V
   * - Current Consumption
     - <150 mA
   * - Dimensions
     - Ø75.8 × 25.48 mm
   * - Weight
     - 85 g

Dimensions
==========

The mechanical dimensions of the AeroAtoms Orbit Neo Plus are shown below.

.. image:: ../../../images/aeroatoms/orbit-neo-plus/orbit-neo-plus-dimensions.png
   :align: center
   :width: 450px

Documentation
=============

Complete documentation for the AeroAtoms Orbit Neo Plus is available on the
AeroAtoms documentation portal.

The documentation includes:

* Product Overview
* Specifications
* Mechanical Dimensions
* User Guide
* Moving Baseline Configuration
* Firmware Updates

Official documentation:

* `Orbit Neo Plus Documentation <https://docs.aeroatoms.com/orbit-neo-plus/overview/>`_

Links
=====

* `Documentation <https://docs.aeroatoms.com>`_
* `Store <https://shop.aeroatoms.com>`_
* `AeroAtoms Website <https://aeroatoms.com>`_