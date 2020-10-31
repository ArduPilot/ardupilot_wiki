.. _common-frsky-passthrough:

====================================
Passthrough FrSky Telemetry Protocol
====================================

Overview
========

Passthrough FrSky Telemetry is the latest protocol available in ArduPilot and it allows the transmission of raw data directly to OpenTX without any preprocessing by OpenTX. The passthrough telemetry protocol is optimized specifically for the FrSky datalink bandwidth and enhancements include having multiple data elements in a single telemetry packet and using floating point representation (e.g. we don't really need centimeter precision altitude to be displayed on screen when flying at 100m). This results in an improved data refresh rate such that information such as attitude (roll, pitch, yaw) is displayed without any perceptible lag.

Compared to older Repurposed FrSky telemetry, passthrough telemetry contains ArduPilot specific information such as flight modes, error messages, warnings, and failsafes.

ArduPilot's Passthrough FrSky telemetry protocol is an open protocol that is free to use. `Yaapu FrSky Telemetry Script for OpenTX <common-frsky-yaapu>`_  is a free script for OpenTX that will display it on your TX. `FlightDeck <http://www.craftandtheoryllc.com/flightdeck-taranis-user-interface-for-ardupilot-arducopter-arduplane-pixhawk-frsky-telemetry-smartport/>`__ is an OpenTX paid app for Taranis that uses passthrough telemetry.

The ArduPilot passthrough telemetry specification including data packets and message rates is available as a `spreadsheet <https://cdn.rawgit.com/ArduPilot/ardupilot_wiki/33cd0c2c/images/FrSky_Passthrough_protocol.xlsx>`__.


Required Hardware
=================

.. image:: ../../../images/frsky_requiredhardware_flightdeck.jpg
    :target: ../_images/frsky_requiredhardware_flightdeck.jpg

Common FrSky Telemetry Setup with OpenTX transmitter running :ref:`FlightDeck <common-frsky-flightdeck>` or :ref:`Yaapu Telemetry Script <common-frsky-yaapu>`.

* An ArduPilot compatible autopilot. If using an autopilot with an F4 processor, you may need an external bi-directional inverter. See :ref:`common-connecting-sport-fport`.

* An OpenTX compatible TX.

* An FrSky SmartPort receiver 


Firmware Configuration
======================

Passthrough FrSky Telemetry is enable by setting ``SERIAL#_PROTOCOL`` to ``10`` using your favorite GCS application.

For information on how to configure ArduPilot for FrSky telemetry, please go :ref:`here <common-frsky-telemetry>`.
