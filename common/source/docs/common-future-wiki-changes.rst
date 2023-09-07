.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.5 release

[copywiki destination="plane,copter,rover,blimp"]

Add note about new Mag Field pre-arm check
==========================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/5404

Add DroneCAN ESC reversing capability
=====================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/5398

Expand the number of possible power/battery monitors up to 16
=============================================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/5396

on :ref:`common-blheli32-passthru` page add note for ESC-Configurator tool:
===========================================================================

.. note:: firmware 4.5 or later required to use this tool.


Add new FENCE_OPTIONS bit:
==========================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5325


Update Gremsy ZIO camera configuration:
=======================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5319

Add new MAVLink Camera/Gimbal commands:
=======================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5307

on :ref:`common-matekf765-wse` page:
====================================

- add that outputs 1-4 are bi-directional dshot capable

on :ref:`common-dshot-escs` page, change the param setup under DShot commands to:
=================================================================================

- set :ref:`SERVO_DSHOT_ESC<SERVO_DSHOT_ESC>` should be set to the type of DShot ESCs being used. This is required especially if Bi-Directional DShot telemetry or Extended Telemetry is being used.

on :ref:`common-efi` page, add:
===============================

see https://github.com/ArduPilot/ardupilot_wiki/pull/5288

on :ref:`common-osd-overview` page, add:
=========================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/5280

on :ref:`planner:mission-planner-flight-data` page, under the ACTIONs list, add a note:
=======================================================================================

.. note:: the ability to change modes from the ground station can be restricted, by mode, using the :ref:`FLTMODE_GCSBLOCK<FLTMODE_GCSBLOCK>` parameter.

on :ref:`common-cuav-pixhawkv6X` and :ref:`common-holybro-pixhawk6X` pages, add:
================================================================================

to their firmware and output sections, that Bi-Directiona DShot firmware variations are available.

on :ref:`common-rcoutput-mapping` page under, RCIN1Scaled to RCIN16Scaled¶ section, add note:
=============================================================================================

.. note:: normally passthru output will hold their last valid value during an RC failsafe. By setting the :ref:`SERVO_RC_FS_MSK<SERVO_RC_FS_MSK>`, selected passthru outputs can be set as if their input channel went to neutral. This is helpful for outputs controlling servo gimbals, or other manually controlled functions.

on :ref:`common-downloading-and-analyzing-data-logs-in-mission-planner` page:
=============================================================================

see https://github.com/ArduPilot/ardupilot_wiki/pull/5227

on :ref:`common-buzzer` page, add the following to the buzzer sounds table:
===========================================================================

- Gryo initialisation complete    7 short beeps
- Ready to ARM          beep-beep-beep-beeeeeep

`BlackBox Logger using ArduPilot Plane <https://github.com/ArduPilot/ardupilot_wiki/pull/5227>`__

Added new battery monitor param for some INA2XX based sensors:
==============================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5241

Dropped included bootloader on some 1MB flash boards:
=====================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5242

[site wiki="plane"]

on :ref:`arming-your-plane` page, under "How to Arm":
=====================================================

add note to Rudder Arming:

.. note:: when rudder arming in QuadPlanes with an autotakeoff, the motors will spin at :ref:`Q_M_SPIN_ARM<Q_M_SPIN_ARM>` and not takeoff until the rudder stick is returned to neutral. Similarly, for normal plane MODE TAKEOF, or autotakeoffs, the arming will not actually occur until the rudder stick is returned to neutral to prevent the takeoff starting with full right rudder.

on :ref:`takeoff-mode` page:
============================

`Takeoff Mode enhancement <https://github.com/ArduPilot/ardupilot_wiki/pull/5173>`__

on :ref:`apms-failsafe-function` page, add long failsafe actions for Mode Takeoff:
==================================================================================

See: https://github.com/ArduPilot/ardupilot_wiki/pull/5353

on :ref:`quadplane-vtol-tuning-process` page:
=============================================

`Motors option bit <https://github.com/ArduPilot/ardupilot_wiki/pull/5218>`__

[/site]
[site wiki="copter"]
on :ref:`traditional-helicopter-tailrotor-setup` page, add info for DDFP tail rotor motor thrust linearization:
===============================================================================================================

See: https://github.com/ArduPilot/ardupilot_wiki/pull/5328

on :ref:`circle-mode` page, under Circle Control Options, add:
==============================================================

`Circle option bit 3 <https://github.com/ArduPilot/ardupilot_wiki/pull/5248>`__

on :ref:`setting-up-for-tuning` page:
=====================================

`Motors option bit <https://github.com/ArduPilot/ardupilot_wiki/pull/5218>`__

on :ref:`gcs-failsafe` page:
============================

Add to failsafe settings:

- **BRAKE or LAND** (Value 7): switch to BRAKE mode if included in firmware or to LAND mode, if not.

on :ref:`follow-mode` page:
===========================

Add the parameter:

- :ref:`FOLL_OPTIONS<FOLL_OPTIONS>`: set bit 0 to "1" to enable the :ref:`common-mount-targeting` to follow the target vehicle.


[/site]
