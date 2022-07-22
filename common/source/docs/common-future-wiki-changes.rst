.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp,dev"]

On :ref:`common-ice` page add section:
======================================

Idle and redline governors
--------------------------

These features rely on having a source for engine RPM. See :ref:`common-rpm` for more information.

The idle governor allows the autopilot to adjust the throttle to maintain an RPM value when the commanded throttle is low. This can be useful when the engine is waiting for takeoff and reduces the workload on the pilot during that time. Increasing the throttle command will give the expected throttle response.

:ref:`ICE_IDLE_PCT<ICE_IDLE_PCT>` This is the minimum percentage throttle output while running, this includes being disarmed, but not while outputs are disabled by the safety switch .
- :ref:`ICE_IDLE_RPM<ICE_IDLE_RPM>` This configures the RPM that will be commanded by the idle governor. Set to -1 to disable.
- :ref:`ICE_IDLE_DB<ICE_IDLE_DB>` This configures the RPM deadband that is tolerated before adjusting the idle setpoint.
- :ref:`ICE_IDLE_SLEW<ICE_IDLE_SLEW>` This configures the slewrate used to adjust the idle setpoint in percentage points per second

The redline governor will slowly reduce the throttle if the RPM remains above the given RPM value. Generally, this RPM value is provided by the manufacturer of the engine. If the commanded throttle drops faster or lower than the point the governor started, then the governor will be released and normal control of the throttle will return to normal throttle response. A warning message will appear on the GCS. Setting :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 1 allows disabling the throttle actions of the redline governor, but still displays the GCS warning message.

- :ref:`ICE_REDLINE_RPM<ICE_REDLINE_RPM>` Maximum RPM for the engine provided by the manufacturer. A value of 0 disables this feature.

[site wiki="rover"]

Add info about steering speed scaling option
============================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/4423

[/site]
[site wiki="plane"]

Add info on altitude control in LOITER mode
===========================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/4457

Add new tuning options for :ref:`common-transmitter-tuning`
===========================================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/4458

Add new mission command, NAV_DELAY
==================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/4465

[/site]
