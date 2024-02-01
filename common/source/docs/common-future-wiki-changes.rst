.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.5 release and is currently in master ("latest")

[copywiki destination="plane,copter,rover,blimp"]



on :ref:`common-rcoutput-mapping` page under, RCIN1Scaled to RCIN16Scaled¶ section, add note:
=============================================================================================

.. note:: normally passthru output will hold their last valid value during an RC failsafe. By setting the :ref:`SERVO_RC_FS_MSK<SERVO_RC_FS_MSK>`, selected passthru outputs can be set as if their input channel went to neutral. This is helpful for outputs controlling servo gimbals, or other manually controlled functions.



on :ref:`common-buzzer` page, add the following to the buzzer sounds table:
===========================================================================

- Gryo initialisation complete    7 short beeps
- Ready to ARM          beep-beep-beep-beeeeeep

`BlackBox Logger using ArduPilot Plane <https://github.com/ArduPilot/ardupilot_wiki/pull/5227>`__



[site wiki="plane"]
Add AUTOTUNE_OPTIONS
====================



on :ref:`arming-your-plane` page, under "How to Arm":
=====================================================

add note to Rudder Arming:

.. note:: when rudder arming in QuadPlanes with an autotakeoff, the motors will spin at :ref:`Q_M_SPIN_ARM<Q_M_SPIN_ARM>` and not takeoff until the rudder stick is returned to neutral. Similarly, for normal plane MODE TAKEOF, or autotakeoffs, the arming will not actually occur until the rudder stick is returned to neutral to prevent the takeoff starting with full right rudder.


[/site]
[site wiki="copter"]


on :ref:`gcs-failsafe` page:
============================

Add to failsafe settings:

- **BRAKE or LAND** (Value 7): switch to BRAKE mode if included in firmware or to LAND mode, if not.

on :ref:`follow-mode` page:
===========================

Add the parameter:

- :ref:`FOLL_OPTIONS<FOLL_OPTIONS>`: set bit 0 to "1" to enable the :ref:`common-mount-targeting` to follow the target vehicle.

[/site]
[site wiki="blimp"]


[/site]
