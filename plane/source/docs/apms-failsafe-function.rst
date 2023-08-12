.. _apms-failsafe-function:

=======================
Plane Failsafe Function
=======================

Plane has a limited failsafe function which is designed to do four
things:

#. Detects a RC Failsafe condition and then initiating a defined response, such as returning to home. Detection of an RC Failsafe is either a complete loss, or corruption, of RC signals, or the receiver sets a FS bit in its data stream for those protocols supporting it (SBUS, etc.), or that the throttle channel PWM value falls below a certain point set by :ref:`THR_FS_VALUE<THR_FS_VALUE>`. This RC failsafe must be enabled by setting :ref:`THR_FAILSAFE<THR_FAILSAFE>` = 1.
#. Optionally, detect loss of telemetry (GCS Failsafe) and take an programmable action, such as switching to return to launch (RTL) mode.

Either of the above have two phases: Short Failsafe which occurs a programmable time after loss of RC or telemetry, which allows optionally circling to try to recover the signals, and if the loss persists longer, a Long Failsafe which determines what long term action is to be taken.

3. Detect loss of GPS for more than 20 seconds and switch into Dead Reckoning mode until GPS signal is regained. See https://youtu.be/0VMx2u8MlUU for a demo.
#. Optionally, detect low battery conditions (low voltage/remaining capacity) and initiate a programmable response, such as returning to home. ArduPilot supports this on multiple batteries.

Here's what the failsafe **will not do**:

- Detect if one or more individual RC channel has failed or become disconnected
- Detect if you're flying too far away or are about to hit the ground
- Detect autopilot hardware failures, such as low-power brownouts or in-air reboots
- Detect if the Plane software is not operating correctly
- Detect other problems with the aircraft, such as motor failures 
- Otherwise stop you from making setup or flight mistakes


.. note:: See :ref:`advanced-failsafe-configuration` for extended failsafe configurations.


RC Failsafe
===========

.. _apms-failsafe-function_throttle_failsafe:

Radio Signal Failure
--------------------

If the received signal is lost or the control information corrupted, or the receiver sets its "failsafe bit" in protocols which have this (like Sbus, FPort, etc.), then an RC Failsafe condition occurs and the actions described in the :ref:`RC Failsafe Actions<fs_actions>` section below will be taken, if the :ref:`THR_FAILSAFE<THR_FAILSAFE>` parameter is 1.

.. note:: by setting :ref:`RC_OPTIONS<RC_OPTIONS>` bit 2, you can force ArduPilot to ignore the "failsafe" bits in the protocol, and only initiate RC Failsafe due to missing or corrupted control information.

Throttle Failsafe
-----------------

In addition, if the throttle signal falls below a threshold set by :ref:`THR_FS_VALUE<THR_FS_VALUE>` and the :ref:`THR_FAILSAFE<THR_FAILSAFE>` is = 1, an RC Failsafe condition will be entered, a Ground Control Station text message ("Throttle Failsafe On") will be sent (to differentiate from a Radio Signal Failure Failsafe), and the actions described in the RC Failsafe Actions section below will be taken.

Throttle Failsafe Setup
-----------------------

.. note:: Throttle Failsafe is not required. If you wish to have failsafe protection against RC signal loss, but not setup a throttle signal controlled failsafe, then set :ref:`THR_FS_VALUE<THR_FS_VALUE>` lower than the lowest RC throttle signal that can be sent to the autopilot from the receiver.

In order to activate the Throttle Failsafe, the throttle signal received by the autopilot must be below :ref:`THR_FS_VALUE<THR_FS_VALUE>`. Once setup, this can be controlled by the pilot to initiate Throttle failsafe intentionally (for testing, or instead of setting up an RTL position on the flight mode switch), or by having the receiver, itself, send that value when it loses RC signal.

.. note:: having the receiver send a pre-set failsafe throttle value upon signal loss is NOT recommended and can lead to issues if a battery level failsafe is setup in QuadPlanes. Setting the receiver to send "no pulses" is much preferred. However, some very old receivers will only send a low throttle level in failsafe, see the section below, :ref:`Old Receivers <old_RX>`.

The system must be setup such that the throttle channel's signal can go below :ref:`THR_FS_VALUE<THR_FS_VALUE>` (Default is 950), but still be above it for normal low throttle stick operation. Before doing the :ref:`RC Calibration<common-radio-control-calibration>` setup step which determines normal operating ranges for the throttle channel and others, make sure that the low throttle stick position on your transmitter is above :ref:`THR_FS_VALUE<THR_FS_VALUE>`. This can be done several ways:

- When you do the :ref:`RC Calibration <common-radio-control-calibration>` setup step, change the trim tab for the throttle channel to adjust its signal 40-50us above :ref:`THR_FS_VALUE<THR_FS_VALUE>` at low throttle stick. This will be the normal operating position. Lowering the trim tab and setting the :ref:`THR_FS_VALUE<THR_FS_VALUE>` to that value allows initiating a failsafe at low trim.
- Setup a transmitter switch that you will use to force failsafe such that it forces the throttle channel signal, using a mix, to below :ref:`THR_FS_VALUE<THR_FS_VALUE>` when activated, allowing normal operation otherwise.

.. _fs_actions:

RC Failsafe Operation
---------------------

-  When RC Failsafe is entered, all RC inputs (except throttle in the case of Throttle Failsafe), are ignored as the autopilot takes its failsafe actions.
-  First, the autopilot will go into Short Failsafe when it detects RC Failsafe for more than :ref:`FS_SHORT_TIMEOUT<FS_SHORT_TIMEOUT>` seconds.
-  A message will be displayed on your Ground Control Station(GCS), or OSD, if its message panel is enabled, that a Short Failsafe is active, and the autopilot will take the :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>`, if enabled.  The default is CIRCLE mode. This is intended to possibly allow the vehicle's changing orientation to re-acquire the signal, but other actions can be assigned. See :ref:`FS_SHORT_ACTN parameter below <failsafe-parameters>` for how each mode responds to the selected action value.
-  If the condition causing the Short Failsafe is removed, the vehicle will return to the previous mode, and a message will be displayed that Short Failsafe is cleared. If it was a Throttle Failsafe that caused the RC Failsafe, and throttle was increased in order to exit, then an additional message will be sent stating that the Throttle Failsafe is OFF.
-  If the condition causing the Short Failsafe persists longer than :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` seconds the autopilot will go into Long Failsafe, send a message to the GCS that it has been entered, and execute the :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` action, if enabled. The default setting for Long Failsafe action to take is RTL (Return to Launch). See :ref:`FS_LONG_ACTN parameter below <failsafe-parameters>` for how each mode responds to the selected action value.
-  If the RC Failsafe condition is later exited, a message will be displayed that the Long Failsafe is cleared, but the flight mode will not revert. If it was a Throttle Failsafe that caused the RC Failsafe, and throttle was increased in order to exit, then an additional message will be sent stating that the Throttle Failsafe is OFF.

.. note:: The action set by :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` will continue even if your RC signal is reacquired, if the flight mode is the same as it was before the failsafe action began. Once RC signal is reacquired, the :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` can be exited via a mode change on the :ref:`FLTMODE_CH<FLTMODE_CH>`. If the mode on the RC transmitter was changed during the failsafe period, then this changed mode is entered after the RC signal is restored. In addition, other failsafes, such as battery failsafe, can also change the mode, if they occur subsequently to the RC signal loss.

Bench Testing RC Failsafe
-------------------------

#. Power up the system and verify that you are seeing RC control in the Mission Planner SETUP->Mandatory Hardware->Radio Calibration tab and in a non-auto mode (Manual, Stabilize, FBW are ok). Check that normal throttle movements to idle do NOT trigger a failsafe and normal control stick movements are observed.
#. Switch to Mission Planners DATA View tab. Turn off the transmitter. After :ref:`FS_SHORT_TIMEOUT<FS_SHORT_TIMEOUT>` seconds, if enabled, you should see the flight mode switch to :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>`. After :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` sec, if enabled, the flight mode should then switch to :ref:`FS_LONG_ACTN<FS_LONG_ACTN>`. Turn the transmitter back on and change flight modes. The Long Failsafe flight mode should change to the selected mode.
#. If Throttle Failsafe is setup (ie via trim tab or transmitter switch). Check that it operates correctly by activating it and watching for Short and Long Failsafes to occur.

If you observe this behavior, your RC Failsafe function has been set up correctly. If not, recheck that the parameters above have been set correctly.

.. _old_RX:

Older Receivers
---------------

Some very old RC receivers cannot be set to send "no pulses" when losing RC signal and simple hold the ROLL/PITCH/YAW RC channels at their last value and set the throttle channel to its minimum PWM value (low throttle). For those, the only way to setup an RC failsafe is to set the :ref:`THR_FS_VALUE<THR_FS_VALUE>` to slightly above that value and use the transmitters trim tab to raise the idle stick value 40-50us above that for normal operation.

.. note:: be sure to do :ref:`ESC calibration<common-esc-calibration>` after you have setup the failsafes and throttle ranges.

**Transmitter Tutorials:**

`Spektrum Setup <https://diydrones.com/profiles/blogs/spektrum-dx8-and-ar8000-failsafe-setup>`__


GCS Failsafe
============

**How it works.** When flying while using telemetry on the GCS, the
autopilot can be programmed to trigger into failsafe mode if it loses
telemetry. In the event that the autopilot stops receiving MAVlink
(telemetry protocol) heartbeat messages. :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` applies just in the case of a long Throttle Failsafe. See :ref:`FS_LONG_ACTN parameter below <failsafe-parameters>` for how each mode responds to the selected action value.

**Setup.**

#. Set :ref:`FS_GCS_ENABL<FS_GCS_ENABL>` to 1 to enable it.
#. Connect to the Mission Planner via telemetry. Verify on the bottom
   right corner of the HUD that you are “flying” in a non auto mode
   (Manual, Stabilize, FBW are ok).
#. Unplug one of the telemetry radios. After a few minutes power off
   your autopilot. (Remember the autopilot will not go fully into failsafe
   until :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` seconds of MAVLink inactivity have passed).
#. Connect your autopilot to the mission planner and pull the logs.
   Verify on the log that the autopilot went into RTL after :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` sec of MAVLink inactivity.

Configuring for Ground Control Station Control beyond RC range
--------------------------------------------------------------

If the telemetry range exceeds the RC transmitter range, then it may be desired to prevent loss of RC signal from initiating a failsafe. Reliance on the above GCS failsafe would be then be used to provide failsafe protection. In order to prevent the RC system from interfering with GCS operation, set :ref:`THR_FAILSAFE<THR_FAILSAFE>` = 2. This prevents the RC failsafe action from being taken, but still detects the failsafe condition and ignores the RC inputs, preventing possible interference to Ground Control Station control of the vehicle. Control via the RC system can be resumed once back into its range.

Configuring for valid RC outputs while in RC Failsafe
=====================================================

Normally, the RC channels are ignored when in RC Failsafe (except the throttle channel, but for failsafe detection exit only). Sometimes it is desirable to allow the preset signal loss values( for receivers capable of this ), to be used in the event of an RC failsafe. For example, parachute activation, or other controls via RC passthrough (see :ref:`common-auxiliary-functions`) could be desired when in RC failsafe. For receivers with this capability and which use a FS data bit, setting :ref:`RC_OPTIONS<RC_OPTIONS>` bit 2 to "1", can accomplish this. In this case, the FS bit is ignored. Upon RC signal loss the receiver would go to its pre-set channel outputs values, but a failsafe action would not be taken by ArduPilot, since the receiver is still outputting valid data as far as ArduPilot can detect. The fixed RC channel values would processed as normal by ArduPilot.


.. note:: In this setup, it is usually necessary to make sure that the flight mode channel will force an RTL or AUTO mission to return the vehicle when the receiver loses RC signal, since no failsafe action will be taken, otherwise. The values of the flight control channels for Roll, Pitch, Yaw and Throttle need to be appropriately set also (usually neutral positions).

.. warning:: Since the autopilot cannot know if the RC link is lost in this configuration, it is possible to get into dangerous situations, especially with QuadPlanes. For example, you are low on battery and far away, and the battery failsafe is active and attempting a VTOL land to prevent a crash. As it drops out of RC range, it will switch to the RC failsafe mode set in the receiver, and attempt to execute that, canceling the battery failsafe action, and ultimately resulting in a crash.

.. _plane-battery-failsafe:

Battery Failsafe
================

.. note::

    This failsafe requires the vehicle have a working :ref:`Power Module <common-powermodule-landingpage>`.

.. note:: ArduPilot supports up to 10 batteries/power monitors. All the  discussion below applies to those optional batteries also. Each can trigger a failsafe and each can have different actions and setup values. In addition, a group of batteries can be treated as a single unit, see ``BATTx_MONITOR`` = 10.

.. note:: the battery low failsafe voltage must be higher than the battery critical failsafe voltage or a pre-arm error will occur.


When the failsafe will trigger
------------------------------

If enabled and set-up correctly the battery failsafe will trigger if the main battery's

-  voltage drops below the voltage held in the :ref:`BATT_LOW_VOLT <BATT_LOW_VOLT>` parameter (or FS_BATT_VOLTAGE in older versions) for more than 10 seconds. If set to zero (the Plane default value) the voltage based trigger will be disabled.
-  remaining capacity falls below the :ref:`BATT_LOW_MAH <BATT_LOW_MAH>` parameter (or FS_BATT_MAH in older versions) 20% of the battery's full capacity is a good choice (i.e. "1000" for a 5000mAh battery).  If set to zero the capacity based trigger will be disabled (i.e. only voltage will be used)

What will happen
----------------

When the failsafe is triggered:

-  Buzzer will play a loud low-battery alarm
-  LEDs will flash yellow
-  A warning message will be displayed on the ground station's HUD (if telemetry is connected)
-  :ref:`BATT_FS_LOW_ACT<BATT_FS_LOW_ACT>`  will be executed

Two-Stage Battery Failsafe
--------------------------

Plane includes a two-layer battery failsafe.  This allows setting up a follow-up action if the battery voltage or remaining capacity falls below an even lower threshold.

- :ref:`BATT_CRT_VOLT <BATT_CRT_VOLT>` - holds the secondary (lower) voltage threshold.  Set to zero to disable. Default is zero.
- :ref:`BATT_CRT_MAH <BATT_CRT_MAH>` - holds the secondary (lower) capacity threshold.  Set to zero to disable. Default is zero.
- :ref:`BATT_FS_CRT_ACT <BATT_FS_CRT_ACT>` - holds the secondary action to take.  A reasonable setup would be to have :ref:`BATT_FS_LOW_ACT <BATT_FS_LOW_ACT>` = 2 (RTL) and :ref:`BATT_FS_CRT_ACT <BATT_FS_CRT_ACT>` = 1 (Land)

Advanced Battery Failsafe Settings
----------------------------------

- :ref:`BATT_FS_VOLTSRC <BATT_FS_VOLTSRC>` allows configuring whether the raw battery voltage or a sag corrected voltage is used
- :ref:`BATT_LOW_TIMER <BATT_LOW_TIMER>` can configure how long the voltage must be below the threshold for the failsafe to trigger
- ``BATTx_`` parameters can be setup to trigger the failsafe on other batteries

Battery Failsafe Actions
------------------------

The following is a description of the actions that can be taken for battery failsafes:

+-----+------------------+-----------------------------------------------------------------------------+
+Value| Action           |     Description                                                             +
+=====+==================+=============================================================================+
+ 0   | None             | Do nothing except warn                                                      +
+-----+------------------+-----------------------------------------------------------------------------+
+ 1   | RTL              | Switch to :ref:`RTL<rtl-mode>` mode                                         +
+-----+------------------+-----------------------------------------------------------------------------+
+ 2   | Land             | Switch to AUTO mode and execute nearest DO_LAND sequence, if in mission     +
+-----+------------------+-----------------------------------------------------------------------------+
+ 3   | Terminate        |  Disarm                                                                     +
+-----+------------------+-----------------------------------------------------------------------------+
+ 4   | QLAND            | If QuadPlane, switch to :ref:`qland-mode`, otherwise do nothing             +
+-----+------------------+-----------------------------------------------------------------------------+
+ 5   | Parachute        |  Trigger Parachute (Critical action only)                                   +
+-----+------------------+-----------------------------------------------------------------------------+
+ 6   | LOITER_TO_QLAND  | If QuadPlane, switch to LOITER_TO_QLAND mode,                               +
+     |                  | otherwise do nothing                                                        +
+-----+------------------+-----------------------------------------------------------------------------+

.. _failsafe-parameters:

Failsafe Parameters and their Meanings
======================================

Short failsafe action (:ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` )
------------------------------------------------------------

The action to take on a short (:ref:`FS_SHORT_TIMEOUT<FS_SHORT_TIMEOUT>` seconds) RC failsafe event .

No Action is ever taken for Short FailSafe in these modes:

- CIRCLE
- RTL
- TAKEOFF
- QRTL
- QLAND
- LOITER to Alt and QLAND

:ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` = 3 disables taking action in ANY mode

.. note:: if in AutoLanding in AUTO, it will always continue to the landing

In QuadPlanes, Short FailSafe will force QLAND by default, RTL if bit 20 of :ref:`Q_OPTIONS<Q_OPTIONS>` is set, or QRTL if bit 5 of :ref:`Q_OPTIONS<Q_OPTIONS>` is set, if entered from these modes:

- QSTABILIZE
- QHOVER
- QLOITER
- QACRO
- QAUTOTUNE

Otherwise:

+----------------------+------------------------+-------------------------+
|FS_SHORT_ACTN         |  Mode                  |   Action Taken          |
+======================+========================+=========================+
| 0 - CONTINUE if in   | MANUAL                 | CIRCLE unless           |
+  AUTO, or CIRCLE     +------------------------+  emergency landing      +
|                      | ACRO                   |  switch is active,      |
+                      +------------------------+  then FBWA              +
|                      |STABILIZE               |                         |
+                      +------------------------+                         +
|                      |FBWA                    |                         |
+                      +------------------------+                         +
|                      |FBWB                    |                         |
+                      +------------------------+                         +
|                      |CRUISE                  |                         |
+                      +------------------------+                         +
|                      |AUTOTUNE                |                         |
+                      +------------------------+                         +
|                      |TRAINING                |                         |
+                      +------------------------+-------------------------+
|                      |LOITER                  |  No Change              |
+                      +------------------------+                         +
|                      |AUTO                    |                         |
+                      +------------------------+                         +
|                      |THERMAL                 |                         |
+                      +------------------------+                         +
|                      |AVOID_ADSB              |                         |
+                      +------------------------+                         +
|                      |GUIDED                  |                         |
+----------------------+------------------------+-------------------------+

+----------------------+------------------------+-------------------------+
|FS_SHORT_ACTN         |  Mode                  |   Action Taken          |
+======================+========================+=========================+
| 1 -CIRCLE            | MANUAL                 | CIRCLE unless           |
+                      +------------------------+  emergency landing      +
|                      | ACRO                   |  switch is active,      |
+                      +------------------------+  then FBWA              +
|                      |STABILIZE               |                         |
+                      +------------------------+                         +
|                      |FBWA                    |                         |
+                      +------------------------+                         +
|                      |FBWB                    |                         |
+                      +------------------------+                         +
|                      |CRUISE                  |                         |
+                      +------------------------+                         +
|                      |AUTOTUNE                |                         |
+                      +------------------------+                         +
|                      |TRAINING                |                         |
+                      +------------------------+-------------------------+
|                      |LOITER                  |  CIRCLE                 |
+                      +------------------------+                         +
|                      |AUTO                    |                         |
+                      +------------------------+                         +
|                      |THERMAL                 |                         |
+                      +------------------------+                         +
|                      |AVOID_ADSB              |                         |
+                      +------------------------+                         +
|                      |GUIDED                  |                         |
+----------------------+------------------------+-------------------------+

+----------------------+------------------------+-------------------------+
|FS_SHORT_ACTN         |  Mode                  |   Action Taken          |
+======================+========================+=========================+
| 2 -GLIDE             | MANUAL                 | GLIDE/FBWB unless       |
+  or                  +------------------------+  emergency landing      +
|  4 - FBWB(ALT HOLD)  | ACRO                   |  switch is active,      |
+                      +------------------------+  then GLIDE             +
|                      |STABILIZE               |                         |
+                      +------------------------+                         +
|                      |FBWA                    |                         |
+                      +------------------------+                         +
|                      |FBWB                    |                         |
+                      +------------------------+                         +
|                      |CRUISE                  |                         |
+                      +------------------------+                         +
|                      |AUTOTUNE                |                         |
+                      +------------------------+                         +
|                      |TRAINING                |                         |
+                      +------------------------+-------------------------+
|                      |LOITER                  | GLIDE/FBWB              |
+                      +------------------------+                         +
|                      |AUTO                    |                         |
+                      +------------------------+                         +
|                      |THERMAL                 |                         |
+                      +------------------------+                         +
|                      |AVOID_ADSB              |                         |
+                      +------------------------+                         +
|                      |GUIDED                  |                         |
+----------------------+------------------------+-------------------------+

Long failsafe action (:ref:`FS_LONG_ACTN<FS_LONG_ACTN>` )
---------------------------------------------------------

The action to take on a long (:ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` seconds) RC failsafe event. :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` should be set longer than :ref:`FS_SHORT_TIMEOUT<FS_SHORT_TIMEOUT>`.

No Action is ever taken for Long FailSafe in these modes:

- RTL
- QRTL
- QLAND
- LOITER to Alt and QLAND

In QuadPlanes, Long FailSafe will force QLAND by default, RTL if bit 20 of :ref:`Q_OPTIONS<Q_OPTIONS>` is set, or QRTL if bit 5 of :ref:`Q_OPTIONS<Q_OPTIONS>` is set, if entered from these modes:

- QSTABILIZE
- QHOVER
- QLOITER
- QACRO
- QAUTOTUNE

Otherwise:

+----------------------+------------------------+-------------------------+
|FS_LONG_ACTN          |  Mode                  |   Action Taken          |
+======================+========================+=========================+
| 0 - CONTINUE if in   | MANUAL                 | RTL unless              |
+  AUTO, or RTL        +------------------------+  emergency landing      +
|                      | ACRO                   |  switch is active,      |
+                      +------------------------+  then GLIDE             +
|                      |STABILIZE               |                         |
+                      +------------------------+                         +
|                      |FBWA                    |                         |
+                      +------------------------+                         +
|                      |FBWB                    |                         |
+                      +------------------------+                         +
|                      |CRUISE                  |                         |
+                      +------------------------+                         +
|                      |AUTOTUNE                |                         |
+                      +------------------------+                         +
|                      |TRAINING                |                         |
+                      +------------------------+                         +
|                      |LOITER                  |                         |
+                      +------------------------+                         +
|                      |THERMAL                 |                         |
+                      +------------------------+                         +
|                      |CIRCLE                  |                         |
|                      +------------------------+                         |
|                      |TAKEOFF                 |                         |
+                      +------------------------+-------------------------+
|                      |AUTO                    |  No Change              |
+                      +------------------------+                         +
|                      |AVOID_ADSB              |                         |
+                      +------------------------+                         +
|                      |GUIDED                  |                         |
+----------------------+------------------------+-------------------------+

+----------------------+------------------------+-------------------------+
|FS_LONG_ACTN          |  Mode                  |   Action Taken          |
+======================+========================+=========================+
| 1 - RTL              | MANUAL                 | RTL unless              |
+                      +------------------------+  emergency landing      +
|                      | ACRO                   |  switch is active,      |
+                      +------------------------+  then GLIDE             +
|                      |STABILIZE               |                         |
+                      +------------------------+                         +
|                      |FBWA                    |                         |
+                      +------------------------+                         +
|                      |FBWB                    |                         |
+                      +------------------------+                         +
|                      |CRUISE                  |                         |
+                      +------------------------+                         +
|                      |AUTOTUNE                |                         |
+                      +------------------------+                         +
|                      |TRAINING                |                         |
+                      +------------------------+                         +
|                      |LOITER                  |                         |
+                      +------------------------+                         +
|                      |THERMAL                 |                         |
+                      +------------------------+                         +
|                      |CIRCLE                  |                         |
|                      +------------------------+                         |
|                      |TAKEOFF                 |                         |
+                      +------------------------+-------------------------+
|                      |AUTO                    |                         |
+                      +------------------------+                         +
|                      |AVOID_ADSB              |  RTL                    |
+                      +------------------------+                         +
|                      |GUIDED                  |                         |
+----------------------+------------------------+-------------------------+

+----------------------+------------------------+-------------------------+
|FS_LONG_ACTN          |  Mode                  |   Action Taken          |
+======================+========================+=========================+
| 2 - GLIDE or         | MANUAL                 | GLIDE,AUTO or PARACHUTE |
+  4- AUTO or          +------------------------+  unless emergency       +
|  3- PARACHUTE        | ACRO                   |  landing switch is      |
+                      +------------------------+  active, then GLIDE     +
|                      |STABILIZE               |                         |
+                      +------------------------+                         +
|                      |FBWA                    |                         |
+                      +------------------------+                         +
|                      |FBWB                    |                         |
+                      +------------------------+                         +
|                      |CRUISE                  |                         |
+                      +------------------------+                         +
|                      |AUTOTUNE                |                         |
+                      +------------------------+                         +
|                      |TRAINING                |                         |
+                      +------------------------+                         +
|                      |LOITER                  |                         |
+                      +------------------------+                         +
|                      |THERMAL                 |                         |
+                      +------------------------+                         +
|                      |CIRCLE                  |                         |
|                      +------------------------+                         |
|                      |TAKEOFF                 |                         |
+                      +------------------------+-------------------------+
|                      |AUTO                    |                         |
+                      +------------------------+                         +
|                      |AVOID_ADSB              | GLIDE,AUTO or PARACHUTE |
+                      +------------------------+                         +
|                      |GUIDED                  |                         |
+----------------------+------------------------+-------------------------+

.. note: in Mode TAKEOFF, Long Failsafe Action is postponed until :ref:`TKOFF_LVL_ALT<TKOFF_LVL_ALT>` is obtained unless action is GLIDE or PARACHUTE, which would occur immediately .

GCS failsafe enable (:ref:`FS_GCS_ENABL<FS_GCS_ENABL>` )
--------------------------------------------------------

Enable ground control station telemetry failsafe. Failsafe will trigger
after :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` seconds of no MAVLink heartbeat or RC Override messages.

.. warning:: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground if it loses contact with your ground station. While the code attempts to verify that the plane is indeed flying and not on the ground before entering this failsafe, it is safer if this option is enabled on an electric plane, to either use a separate motor arming switch or remove the propeller in any ground testing, if possible.

There are three possible enabled settings. Seeing :ref:`FS_GCS_ENABL<FS_GCS_ENABL>` to 1 means that GCS failsafe will be triggered when the aircraft has not received a MAVLink HEARTBEAT message. Setting :ref:`FS_GCS_ENABL<FS_GCS_ENABL>` to 2 means that GCS failsafe will be triggered on either a loss of HEARTBEAT messages, or a RADIO_STATUS message from a MAVLink enabled telemetry radio indicating that the ground station is not receiving status updates from the aircraft, which is indicated by the RADIO_STATUS.remrssi field being zero (this may happen if you have a one way link due to asymmetric noise on the ground station and aircraft radios).Setting :ref:`FS_GCS_ENABL<FS_GCS_ENABL>` to 3 means that GCS failsafe will be triggered by Heartbeat(like option one), but only in AUTO mode. WARNING: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground it it loses contact with your ground station. If this option is enabled on an electric plane then you should enable :ref:`ARMING_REQUIRE<ARMING_REQUIRE>`.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>VALUE</th>
   <th>MEANING</th>
   </tr>
   <tr>
   <td>0</td>
   <td>Disabled</td>
   </tr>
   <tr>
   <td>1</td>
   <td>Heartbeat</td>
   </tr>
   <tr>
   <td>2</td>
   <td>Heartbeat and REMRSSI</td>
   </tr>
   <tr>
   <td>3</td>
   <td>Heartbeat and AUTO</td>
   </tr>
   </tbody>
   </table>

Failsafe Diagnosis in Logs or GCS
=================================

GCSs will often display text indicating the type of failsafe encountered, such as "Failsafe Short event on: type=1/reason=3". Type and Reason can be determined using the table below:


.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>TYPE</th>
   <th>MEANING</th>
   </tr>
   <tr>
   <td>0</td>
   <td>None</td>
   </tr>
   <tr>
   <td>1</td>
   <td>Short Failsafe</td>
   </tr>
   <tr>
   <td>2</td>
   <td>Long Failsafe</td>
   </tr>
   <tr>
   <td>3</td>
   <td>GCS Failsafe</td>
   </tr>
   </tbody>
   </table>
   
.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>REASON</th>
   <th>MEANING</th>
   </tr>
   <tr>
   <td>0</td>
   <td>Unknown</td>
   </tr>
   <tr>
   <td>1</td>
   <td>RC Command</td>
   </tr>
   <tr>
   <td>2</td>
   <td>GCS Command</td>
   </tr>
   <tr>
   <td>3</td>
   <td>Radio Failsafe</td>
   </tr>
   <tr>
   <td>4</td>
   <td>Battery Failsafe</td>
   </tr>
   <tr>
   <td>5</td>
   <td>GCS Failsafe</td>
   </tr>
   <tr>
   <td>6</td>
   <td>EKF Failsafe</td>
   </tr>
   <tr>
   <td>7</td>
   <td>GPS Glitch</td>
   </tr>
   <tr>
   <td>10</td>
   <td>Fence Breached</td>
   </tr>
   <tr>
   <td>11</td>
   <td>Terrain</td>
   </tr>
   <tr>
   <td>19</td>
   <td>Crash</td>
   </tr>
   <tr>
   <td>25+</td>
   <td>General unspecific</td>
   </tr>
   </tbody>
   </table>


Independent Watchdog
====================

See :ref:`common-watchdog` for details.
