.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp"]


[site wiki="copter"]

on :ref:`turtle-mode` page, add the following notes:
====================================================


-  Turtle mode cannot be entered unless throttle is zero
-  Upon entry to turtle mode the motors stay disarmed, but the notfiy LEDs flash
-  Raising the throttle, the motors arm, and motors spin. Lowering throttle to zero disarms the motors
-  Motors spin only when throttle is raised


on :ref:`common-transmitter-tuning` page, add:
==============================================

under TUNE parameter table:

+--------+-------------------------+----------------------------------------------------------------------+
|Value	 |Meaning                  | Parameter                                                            |
+========+=========================+======================================================================+
|59      |Position Control Max     |  :ref:`PSC_ANGLE_MAX<PSC_ANGLE_MAX>`                                 |
|        | Lean Angle              |                                                                      |
+--------+-------------------------+----------------------------------------------------------------------+

on :ref:`airspeed-estimation` page, add to Barometer Position Error section:
============================================================================

- :ref:`BARO1_WCF_UP<BARO1_WCF_UP>`: Pressure error coefficient in positive Z direction (climbing)
- :ref:`BARO1_WCF_DN<BARO1_WCF_DN>`: Pressure error coefficient in negative Z direction (descending)

[site wiki="copter"]

add MAX GAIN and TUNE CHECK to TradHeli Autotune:
=================================================

see: https://github.com/ArduPilot/ardupilot_wiki/pull/4954

add Weathervaning to Copter:
============================

see: https://github.com/ArduPilot/ardupilot_wiki/pull/4961

add TradHeli manual autorotation setup:
=======================================

see: https://github.com/ArduPilot/ardupilot_wiki/pull/4966

on :ref:`common-airspeed-sensor` page, add new param:
=====================================================

- :ref:`ARSPD_ENABLE<ARSPD_ENABLE>` = 1 to allow use of airspeed sensor and to show other airspeed parameters

[/site]
[site wiki="rover"]

on :ref:`wind-vane` page, add at bottom link to:
================================================

`Connecting Bluetooth Windvane to ArduPilot <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_WindVane/Tools/Bluetooth%20NMEA%20receiver/Bluetooth%20NMEA%20receiver.md>`_
[/site]
[site wiki="rover"]

add Optical Flow to Rover:
==========================

see: https://github.com/ArduPilot/ardupilot_wiki/pull/4965
[/site]
