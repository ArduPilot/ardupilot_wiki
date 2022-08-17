.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp"]

Add Analog Fuel Level Sensors
=============================

See https://github.com/ArduPilot/ardupilot_wiki/pull/4413

On :ref:`common-rangefinder-landingpage`
========================================

Add link to :ref:`common-teraranger-neo`

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

On :ref:`common-auxiliary-functions` page
=========================================

Add

+----------------------+----------------------------+----------+---------+---------+
| **RCx_OPTION value** | **Feature Description**    |**Copter**|**Plane**|**Rover**|
+----------------------+----------------------------+----------+---------+---------+
|        162           | In-Flight FFT AutoSetup    |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        163           | Mount Lock                 |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        164           | Pause Streaming Logging    |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Option</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>In-Flight FFT AutoSetup</strong></td>
   <td>

Allows automatic setup of in-flight FFT notch parameters. Set :ref:`FFT_ENABLE<FFT_ENABLE>` =1, takeoff with switch low, hover 30 seconds with switch high,switch low, and land and notch parameters will have been configured.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Mount Lock</strong></td>
   <td>

If high, locks the heading of the mount to earth-frame, otherwise, yaw is heading locked to vehicle heading. If pilot controlled positioning is active, the pilot's inputs changes the heading target appropriately in whatever frame is selected. Without this switch, its vehicle heading.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Pause Streaming Logging</strong></td>
   <td>

If high, will not log streaming type log messages (sensors,attitudes,ekf,etc.) to allow logging only when desired for autopilots with limited logging capabilites (ie no SD card). Events, mode changes, etc. are still logged. Logging is unaffected if switch is low.

.. raw:: html

   </td>
   </tr>
   </table>

In :ref:`common-imu-fft`
========================

add:

.. note:: Setting up the FFT parameters can be done automatically using the ``RCx_OPTION`` auxiliary function "162" on a transmitter switch. Set the function to a switch on the transmitter. Hover the vehicel, switch it on (high) for 30 seconds, switch back low and land. The parameters will have been setup and switch function removed.

In :ref:`common-downloading-and-analyzing-data-logs-in-mission-planner`
=======================================================================

add:


.. note:: Logging of the continuously streaming log messages, such as attitude, sensors, etc. can be paused by using the ``RCx_OPTION`` auxiliary function "164" on a transmitter channel. Switching this channel high will pause these messages, but not events, mode changes, warnings, etc. This allows autopilots with limited logging capabilites (ie using Block logging to chip memory and no SD card) to log only when desired during the flight, as during tuning phases or determination of TECs parameters, etc.

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

- On :ref:`quadplane-auto-mode` page under "AUTO VTOL Takeoff" add to end of section:

In order to prevent the landing gear from dragging due to GPS noise at the beginning of the takeoff, you can set :ref:`Q_NAVALT_MIN<Q_NAVALT_MIN>` to a non zero value which will force the roll and pitch to be zero until that altitude is reached. At which point, navigation (ie loiter in position during the takeoff climb) will become active.

[/site]

