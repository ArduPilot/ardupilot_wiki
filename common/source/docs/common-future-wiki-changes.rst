.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.5 release and is currently in master ("latest")

[copywiki destination="plane,copter,rover,blimp"]
Add RC_Targeting automatic mode change to Mount
===============================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5718

Add IMU RAW logging option and update Notch Filter Setup Guide
==============================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5709

Add CAM/MNT instance parameter
==============================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5704

Add Camera Options parameter
============================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5694

Add new Relay Functions
=======================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5686

Add extAHRS RC switch option
============================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5677

Add Networking Features
=======================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5656

Add HERE4 AP_Periph Firmware
============================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5646

Add Hobbywing DatalinkV2 Telemetry Hub
======================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5643

Add Battery Monitor AuxInfo Option
==================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5640

Add Trimble PX-1 GSOF GPS
=========================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5540

Add MicroStrain7 and InertialLabs external AHRS
===============================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5632

Add Scripting checksum pre-arms
===============================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5630

Add SERIAL port DroneCAN passthru feature
=========================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5614

Add parameter for maximum number of files logged
================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5606

Add info on sharing DroneCAN bus with second protocol
=====================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5602

Add SBF GPS-for-Yaw 
===================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5583

Add polynomial correction params to fuel level batt monitor
===========================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5575

Expand selection of harmonic filter center frequencies
======================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5549

Add option to suppress arming GCS messages
==========================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5508

Add OSD option for aviation style horizon
=========================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5497

Add Battery SOC and Gimbal POI applets
======================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5495

Add ICE option to prevent starting when disarmed
================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5465

Add new NeoPixel LED type
=========================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5467

Add NoopLoop TOF P/F and NanoRadar rangefinder support
======================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5356

Add DShot capability to IOMCU outputs ("MAIN" outputs)
======================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5441

Add TSYS03 temp sensor
======================

see https://github.com/ArduPilot/ardupilot_wiki/pull/5427

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
Add AUTOTUNE_OPTIONS
====================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5713

Add AHRS_OPTIONS
================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5635

Add Hirth Engine/EFI
====================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5581

Add new method using forward motor/tilt to assist in holding position
=====================================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5532

Add MAVLink command option to start ICE even if normally prevented while disarmed
=================================================================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5505

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
Add altitude limits to THROW Mode
=================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5720

Add RTL altitude control improvement
====================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5691

Add Fast Precision Landing speed option
=======================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5598

Add Heli DDFP parameters
========================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5591

Add winch options
=================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5472

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
[site wiki="blimp"]

Add RTL mode for Blimp
======================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5410
[/site]
[site wiki="rover"]
Add Rover CIRCLE mode AUX switch
================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5600
[/site]
[site wiki="rover,copter"]

Add fast waypoint Dijkstras option
==================================

See https://github.com/ArduPilot/ardupilot_wiki/pull/5688
[/site]
