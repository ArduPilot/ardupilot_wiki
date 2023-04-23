.. _rover-failsafes:

=========
Failsafes
=========

Rover supports three failsafe mechanisms as described below.

Radio Failsafe (aka Throttle Failsafe)
======================================

.. image:: ../images/rover-failsafe-rc.jpg
    :target: ../_images/rover-failsafe-rc.jpg

This failsafe is triggered if the connection between the user's transmitter and the receiver on the vehicle is lost for at least :ref:`FS_TIMEOUT <FS_TIMEOUT>` seconds.

- the loss of transmitter/receiver connection is detected by:

  - no signals being sent from the receiver to the autopilot board OR
  - the throttle channel (normally input channel 3) value falling below the :ref:`FS_THR_VALUE <FS_THR_VALUE>` parameter value

- set :ref:`FS_THR_ENABLE <FS_THR_ENABLE>` to "1" to enable this failsafe
- if :ref:`FS_ACTION <FS_ACTION>` is "1", the vehicle will :ref:`RTL <rtl-mode>` to home, if "2" the vehicle will :ref:`Hold <hold-mode>`, if "3" or "4" the vehicle will attempt to use :ref:`SmartRTL <smartrtl-mode>` but if this mode cannot be engaged the vehicle will :ref:`RTL <rtl-mode>` or :ref:`Hold <hold-mode>` respectively.
- once the transmitter/receiver connection is restored, the user must use the transmitter's mode switch to re-take control of the vehicle in :ref:`Manual <manual-mode>` (or any other mode)

Battery Failsafe
================

.. note::

    This failsafe requires the vehicle have a working :ref:`Power Module <common-powermodule-landingpage>`.

.. note:: ArduPilot firmware versions 4.0 and later support up to 10 batteries/power monitors. All the  discussion below applies to those optional batteries also. Each can trigger a failsafe and each can have different actions and setup values. In addition, a group of batteries can be treated as a single unit, see ``BATTx_MONITOR`` = 10.

When the failsafe will trigger
------------------------------

If enabled and set-up correctly the battery failsafe will trigger if the main battery's

-  voltage drops below the voltage held in the :ref:`BATT_LOW_VOLT <BATT_LOW_VOLT>` parameter (or FS_BATT_VOLTAGE in older versions) for more than 10 seconds. If set to zero (the default value) the voltage based trigger will be disabled.
-  remaining capacity falls below the :ref:`BATT_LOW_MAH <BATT_LOW_MAH>` parameter (or FS_BATT_MAH in older versions) 20% of the battery's full capacity is a good choice (i.e. "1000" for a 5000mAh battery).  If set to zero (the default) the capacity based trigger will be disabled (i.e. only voltage will be used)

What will happen
----------------

When the failsafe is triggered:

-  Buzzer will play a loud low-battery alarm
-  LEDs will flash yellow
-  A warning message will be displayed on the ground station's HUD (if telemetry is connected)
-  :ref:`BATT_FS_LOW_ACT <BATT_FS_LOW_ACT>` configures the failsafe action to take.  "0" to take no action (default), "1" to change into :ref:`RTL <rtl-mode>`, "2" to change to :ref:`Hold <hold-mode>`, if "3" or "4" the vehicle will attempt to use :ref:`SmartRTL <smartrtl-mode>` but if this mode cannot be engaged the vehicle will :ref:`RTL <rtl-mode>` or :ref:`Hold <hold-mode>` respectively.  "5" will disarm the vehicle.

Two-Stage Battery Failsafe
--------------------------

Rover also includes a two-layer battery failsafe.  This allows setting up a follow-up action if the battery voltage or remaining capacity falls below an even lower threshold.

- :ref:`BATT_CRT_VOLT <BATT_CRT_VOLT>` - holds the secondary (lower) voltage threshold.  Set to zero to disable. Default is zero.
- :ref:`BATT_CRT_MAH <BATT_CRT_MAH>` - holds the secondary (lower) capacity threshold.  Set to zero to disable. Default is zero.
- :ref:`BATT_FS_CRT_ACT <BATT_FS_CRT_ACT>` - holds the secondary action to take. It has the same options and default as :ref:`BATT_FS_LOW_ACT <BATT_FS_LOW_ACT>`.

Advanced Battery Failsafe Settings
----------------------------------

- :ref:`BATT_FS_VOLTSRC <BATT_FS_VOLTSRC>` allows configuring whether the raw battery voltage or a sag corrected voltage is used
- :ref:`BATT_LOW_TIMER <BATT_LOW_TIMER>` can configure how long the voltage must be below the threshold for the failsafe to trigger (10 sec default)
- ``BATTx_`` parameters can be setup to trigger the failsafe on other batteries

GCS Failsafe (aka Telemetry Failsafe)
=====================================

Prior to Rover-4.4
------------------

This failsafe is triggered if the vehicle stops receiving `heartbeat messages <https://mavlink.io/en/messages/common.html#HEARTBEAT>`__ from the ground station for at least :ref:`FS_TIMEOUT <FS_TIMEOUT>` seconds.

Starting from Rover-4.4
-----------------------

Significant improvements have been made in Rover-4.4 and later for Ground Control Failsafe .

The Ground Station Control (GCS) failsafe controls how Rover will behave if contact with the GCS is lost.
The GCS failsafe monitors the time since the last MAVLink heartbeat from the GCS. If no heartbeat is received :ref:`FS_GCS_TIMEOUT <FS_GCS_TIMEOUT>` seconds (Default is 5 seconds), the GCS failsafe event will trigger based on your parameter settings. Note that if no GCS is ever connected, the GCS failsafe will remain inactive regardless of parameter settings.

Enabling the failsafe in all versions
-------------------------------------

In parameters list, set the :ref:`FS_GCS_ENABLE <FS_GCS_ENABLE>` parameter to:

-  **Disabled** (Value 0) will disable the GCS failsafe entirely.
-  **Enabled** (Value 1) will execute the FS_ACTION when Failsafe trigger.
-  **Enabled Continue with Mission in Auto Mode** (Value 2) will ignore the failsafe in an Auto Mode mission.

The action done on GCS Failsafe is controlled by the :ref:`FS_ACTION <FS_ACTION>` parameter:

- **Nothing** (Value O) will do nothing.
- **RTL** (Value 1) will active RTL to go home.
- **Hold** (Value 2) will active Hold Mode and stay in place.
- **SmartRTL or RTL** (Value 3) will active SmartRTL mode to go back home or RTL if SmartRTL doesn't work.
- **SmartRTL or Hold** (Value 4)  will active SmartRTL mode to go back home or Hold mode if Smart RTL doesn't work.

You must use the transmitter's mode switch to re-take control of the vehicle in :ref:`Manual <manual-mode>` (or any other mode), or change modes with the GCS itself, if communication is re-established.


Crash Check
===========

If enabled by setting the :ref:`FS_CRASH_CHECK <FS_CRASH_CHECK>` parameter to "1" (for :ref:`Hold <hold-mode>`) or "2" (for :ref:`Hold <hold-mode>` and Disarm) this failsafe will switch the vehicle to Hold and then (optionally) disarm the vehicle if all the following are true for at least 2 seconds:

- the vehicle is in :ref:`Auto <auto-mode>`, :ref:`Guided <guided-mode>`, :ref:`RTL <rtl-mode>` or :ref:`SmartRTL <smartrtl-mode>` mode
- velocity falls below 0.08m/s (i.e. 8cm/s)
- the vehicle is turning at less than 4.5 deg/s
- demanded throttle to the motors (from the pilot or autopilot) is at least 5%

Hold Mode Failsafes
===================

The :ref:`FS_OPTIONS<FS_OPTIONS>` bitmask parameter determines if failsafes will be recognized while in HOLD Mode. If bit 0 is set, then failsafes will be recognized and acted upon. If not, failsafes will be ignored in HOLD mode (default).

Hardware Watchdog
=================

See :ref:`common-watchdog` for details.
