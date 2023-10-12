.. _common-rangefinder-nooploop-tofsense-f.rst:

===================
Nooploop TOFSense F
===================

.. note::
    Support for this sensor is available in firmware versions 4.5 and later.

The `TOFSense F/FP <https://ftp.nooploop.com/software/products/tofsense-f/doc/TOFSense-F_Datasheet_V1.2_en.pdf>`__ is
lightweight rangefinder module that provides fast and accurate distance measurements up to 25 meters (FP variant only).
ArduPilot currently supports this sensor only via UART. I2C support will be added in the future.


Connecting via UART to Autopilot
================================

The ame steps as the Nooploop TOFSense P (UART) can be followed, as linked :ref:`here <common-rangefinder-nooploop-tofsense-p>`.
:ref:`RNGFND1_MAX_CM <RNGFND1_MAX_CM>` can be changed as per the sensor specifications (1500 for TOFSense F and 2500 for TOFSense FP)

Connecting via I2C
==================
future driver addition


Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look for "rangefinder1".
