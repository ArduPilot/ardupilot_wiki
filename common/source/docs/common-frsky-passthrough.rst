.. _common-frsky-passthrough:

====================================
Passthrough FrSky Telemetry Protocol
====================================

Overview
========

Passthrough FrSky Telemetry is the latest protocol available in ArduPilot and it allows the transmission of raw data directly to OpenTX without any preprocessing by OpenTX and without the need for discovery. The passthrough telemetry protocol is optimized specifically for the FrSky datalink bandwidth and enhancements include having multiple data elements in a single telemetry packet and using floating point representation (e.g. we don't really need centimeter precision altitude to be displayed on screen when flying at 100m). This results in an improved data refresh rate such that information such as attitude (roll, pitch, yaw) is displayed without any perceptible lag.

Compared to regular FrSky telemetry, passthrough telemetry contains ArduPilot specific information such as flight modes, error messages, warnings, and failsafes.

ArduPilot's Passthrough FrSky telemetry protocol is an open protocol that is free to use. `FlightDeck <http://www.craftandtheoryllc.com/flightdeck-taranis-user-interface-for-ardupilot-arducopter-arduplane-pixhawk-frsky-telemetry-smartport/>`__ is an OpenTX paid app for Taranis that uses passthrough telemetry.

The ArduPilot passthrough telemetry specification including data packets and message rates is available as a `spreadsheet <https://cdn.rawgit.com/ArduPilot/ardupilot_wiki/33cd0c2c/images/FrSky_Passthrough_protocol.xlsx>`__.

An example on how to decode all messages available via Passthrough to be displayed in a telemetry lua script can be dowloaded for free from this `repo <https://github.com/jplopezll/OpenTX_FrSkySPort_passthrough_master/>`__.

Required/Compatible Hardware
============================

.. image:: ../../../images/frsky_requiredhardware_flightdeck.jpg
    :target: ../_images/frsky_requiredhardware_flightdeck.jpg

Common FrSky Telemetry Setup with Taranis RC transmitter running :ref:`FlightDeck <common-frsky-flightdeck>`.

* An ArduPilot compatible flight controler from this list:
	
	- Pixhawk
	- Pixhawk derivatives (clones)
	- The Cube
	- PixRacer
	- Pixhack
	- (Linux Boards have not been tested and APM2.6 is not supported)

* A Taranis X9D, X9D+, X9E, Q X7 or a device running ErSky9x, such as
  the Turnigy 9XR Pro.

* An X series FrSky SmartPort receiver from this list:
	
	- X4R
	- X4R-SB
	- X6R
	- **X8R** (recommended for medium to large airframes)
	- **XSR** (recommended for mini to medium airframes)

* A SmartPort telemetry cable to connect the ArduPilot compatible autopilot on the FrSky SmartPort bus (Pixhawk, The Cube, PixRacer and Pixhack cables available from `Craft and Theory <http://www.craftandtheoryllc.com/product-category/telemetry-cables/>`__)

.. image:: ../../../images/FrSky_Hardware3.jpg
    :target: http://www.craftandtheoryllc.com/product/package-flightdeck-taranis-opentx-ardupilot-arducopter-pixhawk-2-cube-servo-frsky-x9d-x7-q-x7-qx7-telemetry-smartport-smart-port-serial/#configuration

For information on how to connect the FrSky equipment together, please go :ref:`here <common-frsky-equipment>`.

Firmware Configuration
======================

Passthrough FrSky Telemetry is enable by setting ``SERIAL#_PROTOCOL`` to ``10`` using your favorite GCS application.

For information on how to configure ArduPilot for FrSky telemetry, please go :ref:`here <common-frsky-configMP>`.
