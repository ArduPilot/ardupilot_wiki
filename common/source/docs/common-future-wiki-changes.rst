.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp"]


on :ref:`common-tbs-rc` page, add a section on ELRS:
====================================================

ELRS can be setup in the same manner as CRSF, however, bit 13 of :ref:`RC_OPTIONS<RC_OPTIONS>` should be set to alter the baudrate from 416KBaud that CRSF uses, to 420KBaud that ELRS uses.

add UM982 Moving Baseline GPS:
==============================

see : https://github.com/ArduPilot/ardupilot_wiki/pull/4967

on :ref:`common-auxiliary-functions`, add:
==========================================


+----------------------+----------------------------+----------+---------+---------+
| **RCx_OPTION value** | **Feature Description**    |**Copter**|**Plane**|**Rover**|
+----------------------+----------------------------+----------+---------+---------+
|       170            |  QSTABILIZE mode           |          |  x      |         |
+----------------------+----------------------------+----------+---------+---------+
|       171            |  Compass Calibration       |    x     |  x      |    x    |
+----------------------+----------------------------+----------+---------+---------+

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Option</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>Compass Calibration</strong></td>
   <td>

Switching to high will behave the same as if the Start button for :ref:`onboard calibration <onboard_calibration>` had been pressed. Returning the switch to low will cancel the calibration if still in progress.

.. raw:: html

   </td>
   </tr>
   </tbody>
   </table>

on :ref:`common-compass-calibration-in-mission-planner` page:
=============================================================

change content of Onboard Calibration using Stick Gestures (no GCS) section to be started by RC switch (171) instead of stick gestures.


on :ref:`common-downloading-and-analyzing-data-logs-in-mission-planner` page:
=============================================================================

under Logging Parameters, change:

- :ref:`LOG_DISARMED<LOG_DISARMED>`: Setting to one will start logging when power is applied, rather than at the first arming of the vehicle. Usefull when debugging pre-arm failures.

to

- :ref:`LOG_DISARMED<LOG_DISARMED>`: Setting to 1 will start logging when power is applied, rather than at the first arming of the vehicle. Usefull when debugging pre-arm failures. Setting to 2 will only log on power application other than USB power to prevent logging while setting up on the bench.

on :ref:`common-powermodule-landingpage`, add:
==============================================

.. toctree::
    :maxdepth: 1

    Synthetic Current Sensor/Analog Voltage Monitor <common-synthetic-current-monitor>

add a new section:
------------------

EFI Fuel Monitoring, See :ref:`common-efi` section on using :ref:`BATT_MONITOR<BATT_MONITOR>` = 27 (EFI).

on :ref:`common-efi` page add section:

Using a Battery Monitor to Report Fuel Flow and Consumption

If an EFI is used in the system, either thru a LUA driver or the built-in drivers above, the fuel flow and consumption can be monitored using :ref:`BATT_MONITOR<BATT_MONITOR>` = 27. The fuel flow in liters/hour will be reported as amps, while the fuel consumed in milliliters will be reported as mah. 

.. note:: the MAVLink command to reset the fuel consumed does not work with this monitor.


on :ref:`common-uavcan-setup-advanced`, :ref:`mission-planner-initial-setup`, :ref:`common-slcan-f4`, and :ref:`common-slcan-f7h7` pages add the following note:
================================================================================================================================================================
.. note:: SLCAN access via COM port is disabled when armed to lower cpu load. Use SLCAN via MAVLink instead.

on :ref:`common-external-ahrs` add:
===================================

under Supported Systems add:

  - VectorNav VN-100AHRS

under Setup replace with:

VectorNav300 or Parker Lord
---------------------------

    - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)

    - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV) or 2 (Parker Lord)

This will replace ArduPilot’s internally generated INS/AHRS subsystems with the external system

VectorNav100
------------

    - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 3 (ArduPilot's EKF3)

    - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV)

    - :ref:`EAHRS_OPTIONS<EAHRS_OPTIONS>` bit 0 set to 1 ("1" value) to disable its compensation of the sensor biases, letting EKF3 do that (since there is no internal GPS to provide the best estimates)

- for all of the above, set the ``SERIALx_PROTOCOL`` to “36” (AHRS) and ``SERIALx_BAUD`` to “115” (unless you have changed the external unit’s baud rate from its default value) for the port which is connected to the external AHRS unit.

on :ref:`common-efi` page, add:
===============================

In addition, ArduPilot allows the addition of new EFI controller drivers via :ref:`common-lua-scripts`. For examples, see the `HFE CAN EFI driver <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/drivers/EFI_HFE.md>`__ or the `SkyPower CAN driver <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/drivers/EFI_SkyPower.md>`__

on :ref:`common-camera-runcam` page, under Setup, add:
======================================================

For RunCam2 4K camera, set :ref:`CAM_RC_TYPE<CAM_RC_TYPE>` = 5.

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
