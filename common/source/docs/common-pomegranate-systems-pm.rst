.. _common-pomegranate-systems-pm:

=============================================
Pomegranate Systems CAN/DroneCAN Power Module
=============================================

.. image:: ../../../images/pomegranate-systems-dronecan-power-module.png
   :target: ../_images/pomegranate-systems-dronecan-power-module.png


Intelligent power monitor, accurate fuel gauge, and efficient 5V/2A power supply with a CANbus interface.

-    High-resolution current and voltage monitoring with state of charge integration.
-    Power monitoring for battery, 5V, and 3V rails.
-    Supports 2-6S batteries (6-26V) and currents of up to 100A, when actively cooled.
-    CANbus interface using DroneCAN v0 protocol (PX4 / ArduPilot Compatible).
-    Efficient 5V switching regulator provides up to 2A of low noise current.
-    Daytime visible state of charge LED
-    Electronic CANbus termination simplifies wiring.
-    Multiple modules on the same bus can monitor multiple power sources.
-    Board-mounted battery connectors and M2.5 mounting holes provide a robust battery entry point.
-    Open Source Firmware

.. note::

   Pomegranate Systems also sell a `Power Monitor <https://store.p-systems.io/products/power-monitor>`__,
   which uses the same DroneCAN interface and is offered in low (40A), standard (80A) and
   high (120A) current variants, but has no onboard 5V regulator and so requires an
   independent 5V supply. Check the availability of both parts before ordering.

Configuration and Setup
=======================

.. youtube:: 5bUyms6UPao


Where to Buy
============

- `Power Module <https://store.p-systems.io/products/power-module>`__ - the module described above, with the 5V/2A regulator
- `Power Monitor <https://store.p-systems.io/products/power-monitor>`__ - same DroneCAN monitoring, no 5V regulator