.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp"]


[site wiki="plane"]
on :ref:`automatic-takeoff` page, add at bottom a new section:
===============================================================

Catapult Launch without an Airspeed Sensor
==========================================
Taking off without an airspeed sensor using a catapult may cause less than maximum throttle to be used due to high initial climb rates. For heavy vehicles, this may result in a stall due to the long time constants used in TECS to adjust throttle after the initial launch. The parameter :ref:`TKOFF_THR_MAX_T<TKOFF_THR_MAX_T>` can be used to force maximum throttle for a time, irrespective of climb rates from an initial catapult launch to allow the vehicle to obtain sufficient speed.

on :ref:`common-mavlink-mission-command-messages-mav_cmd` page:
===============================================================

under DO_CHANGE_SPEED command add:" For Airspeed, a value of -2.0 indicates return to normal cruise airspeed" to speed parameter description.

on :ref:`acro-mode` page, in section "Acro Locking", add:
=========================================================

It is recommended that it be set to "2", instead of "1", in order to use a quarternion based control system with much better performance than the older system. In order for this to be effective, yaw rate control (:ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>`) must be "1" and the yaw rate controller tuned using :ref:`Autotune <automatic-tuning-with-autotune>` for best performance.

on the :ref:`automatic-tuning-with-autotune` page:
==================================================

add in the setup section:
-------------------------

The :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>` bitmask selects which axes will be tuned while in Autotune. Default is roll, pitch and yaw.

remove in the setup section:
----------------------------

Tuning the yaw axis can only be done in AUTOTUNE mode, or using the ``RCx_OPTION`` switch set to 107, ``but only in ACRO mode`` with :ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>` = 1.

remove in Acro Mode Yaw Rate Controller section:
------------------------------------------------

Autotuning YAW can also be done ``in ACRO mode`` using an auxiliary switch set with the ``RCx_OPTION`` to 107.

change note in the YAW Controller section to:
---------------------------------------------

.. note:: while AutoTuning with this controller enabled, roll inputs will result in yaw outputs also, allowing more coordinated turns with the yaw controller active. This will normally result in simultaneously tuning the yaw controller with the roll controller, but not necessarily completing the yaw tune when the roll tune finishes. Also, there may be seemingly excessive rudder applied initially in the roll tune on vehicles with large yaw authority, until the tune progresses.

On :ref:`apms-failsafe-function` page:
======================================

Add note to Battery Failsafe Section:

.. note:: the battery low failsafe voltage must be higher than the battery critical failsafe voltage or a pre-arm error will occur.

On :ref:`automatic-landing` page, under Controlling the Flare:
==============================================================

The landing controller sets a point before the touchdown as the expected flare start point. This "flare_aim" point is calculated from the :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>` and :ref:`TECS_LAND_SINK<TECS_LAND_SINK>` for the expected duration of the flare before the actual touchdown. If consistently landing long or short, this point can be adjusted using the :ref:` TECS_FLARE_AIM<TECS_FLARE_AIM>` parameter. If landing too short, decrease the percentage from its default of 50%, conversely, increasing it if landing too long.

The transition from the glide-slope sink rate to the flare sink rate is controlled by the :ref:`TECS_FLARE_HGT<TECS_FLARE_HGT>` parameter. The start of the flare will occur at :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>` and the sink rate will be gradually adjusted to :ref:`TECS_LAND_SINK<TECS_LAND_SINK>` at the :ref:`TECS_FLARE_HGT<TECS_FLARE_HGT>` to avoid a rapid pitch change at the beginning of the flare, which would tend to create a "ballooning" effect at the start of the flare. :ref:`TECS_FLARE_HGT<TECS_FLARE_HGT>` should be lower than :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>`.

On :ref:`precision-autolanding` page, under the Approach Airspeed section, add:
===============================================================================

The :ref:`LAND_WIND_COMP<LAND_WIND_COMP>` parameter controls how much headwind compensation is used when landing. Headwind speed component multiplied by this parameter is added to :ref:`TECS_LAND_ARSPD<TECS_LAND_ARSPD>` value. Set to 0 to disable this. 

.. note:: The target landing airspeed value is still limited to being lower than :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>`.

[/site]

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
[/site]

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
