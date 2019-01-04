.. _common-diagnosing-problems-using-logs:

==============================
Diagnosing problems using Logs
==============================

This page show how to diagnose the six most common problems affecting Copter in particular but to some extent Plane and Rover as well.

[copywiki destination="copter,plane,rover,planner"]

Log Types (Dataflash vs tlogs)
==============================

There are two ways to record your flight data. With some exceptions, the
two methods record very similar data but in different ways:

-  :ref:`Dataflash logs <common-downloading-and-analyzing-data-logs-in-mission-planner>` are recorded on the flight controller (often to the SD card) so they must be downloaded from the flight controller after a flight
-  :ref:`Telemetry logs <planner:common-mission-planner-telemetry-logs>` (also known as "tlogs") are recorded by the ground station (i.e. Mission Planner) on the local PC when the autopilot is connected via a :ref:`telemetry link <common-telemetry-landingpage>`

If you are not yet familiar with the basics of these log files, first review the introductory pages to understand where these logs are stored and how you can download and view the information held within them.

.. _common-diagnosing-problems-using-logs_mechanical_failures:

Mechanical Failures
===================

Common mechanical failures include a motor or ESC failure (`including ESC sync failures <https://www.youtube.com/watch?v=hBUBbeyLe0Q>`__), the propeller breaking or coming off, etc.  These appear in the log as a sudden divergence in the desired roll and pitch vs the vehicle's actual roll and pitch.  This divergence is visible by graphing the ATT message's DesRoll vs Roll, DesPitch vs Pitch and to a lesser extent DesYaw vs Yaw.

.. image:: ../../../images/DiagnosingWithLogs_RollInVsRoll.png
    :target: ../_images/DiagnosingWithLogs_RollInVsRoll.png

In the example above the vehicle's actual roll ("Roll") closely follows the desired roll ("DesRoll") for the first part of the log but then suddenly diverges.  The flight controller wanted the roll to remain level ("Roll" = 0) but it was unable to likely meaning there was a mechanical failure.  This is very different from a software failure in which the flight controller freaked out and for some strange reason suddenly wanted the copter up-side-down because in such cases the DesRoll would be also be crazy and actual Roll would follow.

Tlogs contain the same data.  Compare NAV_CONTROLLER_OUTPUT's nav_roll (desired roll) and nav_pitch (desired pitch) to ATTITUDE.roll (actual roll) and pitch (actual pitch).

.. _common-diagnosing-problems-using-logs_vibrations:

Vibrations
==========

High vibrations can cause the Copter's accelerometer based altitude and horizontal position estimates to drift far off from reality which leads to problems with altitude hold (the vehicle may rocket into the sky) or position control in modes like Loiter, PosHold, Auto, etc.

As covered on the :ref:`Measuring Vibration <common-measuring-vibration>` page, vibrations are best viewed by graphing the **VIBE** message's **VibeX**, **VibeY** and **VibeZ** values.  Vibration levels below 30m/s/s are normally acceptable. Levels above 30m/s/s may have problems and levels above 60m/s/s nearly always have problems with position or altitude hold.

The below graph shows acceptable vibration levels which are consistently below 30m/s/s

.. image:: ../../../images/mp_vibe_dataflash_msg.png
    :target: ../_images/mp_vibe_dataflash_msg.png

The graph below is from a vehicle that had position estimation problems due to high vibrations

.. image:: ../../../images/mp_measuring_vibration_bad_vibes.png
    :target: ../_images/mp_measuring_vibration_bad_vibes.png

Tlog's VIBRATION **vibration_x**, **vibration_y** and **vibration_z** can also be used and show the same information as is stored to the dataflash log

.. image:: ../../../images/DiagnosingWithLogs_VibesTlog.png
    :target: ../_images/DiagnosingWithLogs_VibesTlog.png

.. _common-diagnosing-problems-using-logs_compass_interference:

Compass interference
====================

Interference from the power distribution board, motors, battery, ESCs and other electrical devices near the flight controller can throw off the compass heading which can lead to circling (aka "toilet bowling") or even the copter flying off in completely the wrong direction.  Graphing the tlog's mag_field (found under "CUSTOM") and throttle (found under VFR_HUD) is the easiest way to quickly see the amount of interference.

The graph below shows an acceptable amount of magnetic interference with mag_field fluctuations of only 10% to 20% when the throttle is raised.  Below 30% interference is acceptable.  Between 30% ~ 60% is in the grey zone where it might be OK (some users are OK, some are not) and really bad magnetic interference will show up as jumps of over 60% when the throttle is raised.

.. image:: ../../../images/DiagnosingWithLogs_MagFieldVsThr.png
    :target: ../_images/DiagnosingWithLogs_MagFieldVsThr.png

Extra Notes:

-  The length of the mag_field can be anywhere from 120 ~ 550 depending upon where in the world the vehicle is but it is normally around 330
-  The magnetic interference as a percentage of the total mag field is also displayed at the end of the compassmot set-up procedure.  Search for "CompassMot" on the :ref:`Advanced Compass Setup page <common-compass-setup-advanced>` page to learn more about compassmot
-  Dataflash log's COMPASS message hold the compass's raw x, y and z axis values (called MagX, MagY, MagZ) which are equivalent to the tlog's RAW_IMU xmag, ymag and zmag fields.  It is possible to calculate the mag-field length by first loading the dataflash log file into excel, filtering by the COMPASS message and then calculating the mag-field using the formula mag_field = sqrt(MagX^2, MagY^2, MagZ^2).  Note that the COMPASS message is not enabled by default in the dataflash logs because it runs at 50hz and does affect CPU performance a bit.
-  The image above it shows a short spike at the beginning of the graph but this can be ignored because it is before the throttle is raised so it is probably just as the user plugged in some other electrical device

.. _common-diagnosing-problems-using-logs_gps_glitches:

GPS glitches
============

When in autonomous modes (Loiter, RTL, Auto, etc) position errors from the GPS can cause the vehicle to think that it is suddenly in the wrong place and lead to aggressive flying to correct the perceived error.  These "glitches" show up in both the tlogs and dataflash logs as an decrease in the number of satellites visible and an increase in the `hdop <https://en.wikipedia.org/wiki/Dilution_of_precision_%28GPS%29>`__.

Graph the Dataflash log's GPS message's "HDop" and "NSats" values.  Hdop values below 1.5 are very good, values over 2.0 could indicate the GPS positions are not good.  The number of satellites falling below 12 is also bad.  A significant change in these two values often accompanies a GPS position change.

.. image:: ../../../images/DisagnosingWithLogs_GPShdopAndNumSats_DF.png
    :target: ../_images/DisagnosingWithLogs_GPShdopAndNumSats_DF.png

If using tlogs graph the GPS_RAW_IT group's "eph" and "satellites_visible" values.  Hdop values below 150 are good, values over 200 could indicate a bad position.

.. image:: ../../../images/DisagnosingWithLogs_GPShdopAndNumSats.png
    :target: ../_images/DisagnosingWithLogs_GPShdopAndNumSats.png

See the :ref:`EKF failsafe <copter:ekf-inav-failsafe>` wiki page for more details on how the vehicle may switch to non-autonomous modes in the case of very bad GPS glitches

Power Problems (BrownOuts, etc)
===============================

:ref:`Power Modules <common-powermodule-landingpage>` provide a reliable power supply to the flight controller but brown-outs do still occasionally occur.  They can normally be recognised by the logs suddenly ending while the vehicle is still in the air (i.e. barometer or EKF altitude is still reporting the vehicle's altitude is well above zero).

Try graphing the:

-  Dataflash log's CTUN Alt (altitude above home) or BAlt (Barometer altitude)
-  Dataflash log's GPS Alt (altitude above sea level)
-  Tlog's VFR_HUD alt (the combined accelerometer + barometer altitude estimate)
-  Tlog's GLOBAL_POSITION relative_alt (altitude above home)

.. image:: ../../../images/DiagnosingWithLogs_BrownOut.png
    :target: ../_images/DiagnosingWithLogs_BrownOut.png

Changes in the board voltage can also be a sign of a power problem.  Variations of 0.10 to 0.15 volts are normal.  Beyond that could be a sign that other devices sharing the power source with the flight controller are causing ripples in the power supply that could lead to a brown-out.  The board voltage can be graphed using:

-  Dataflash POWR message's VCC
-  Tlog HWSTATUS's Vcc

In the image directly below shows the board voltage sinking by 0.15V when the throttle is raised.  This is generally not a good thing but because it's only 0.15V it's probably OK. The 2nd graph below (a dataflash graph from a different user's log) shows a very stable voltage with ripples less than 0.1V.

.. image:: ../../../images/DiagnosingWithLogs_BoardVccVsThr.png
    :target: ../_images/DiagnosingWithLogs_BoardVccVsThr.png

.. image:: ../../../images/DiagnosingWithLogs_DataFlashBoardVcc.png
    :target: ../_images/DiagnosingWithLogs_DataFlashBoardVcc.png

Unexpected ERRORS including Failsafes
=====================================

When unexpected behaviour from the flight controller occurs (especially when the user complains that the copter no longer responded to their input) it is often caused by one of the :ref:`failsafes <copter:failsafe-landing-page>` being triggered.

The easiest way to find these is to look in the dataflash logs and filter the first column by "ERR".  If using the Mission Planner, the errors will also appear in red markers at the top of the graphing area.

.. image:: ../../../images/DiagnosingWithLogs_FilterForErrors.png
    :target: ../_images/DiagnosingWithLogs_FilterForErrors.png

The Subsys (aka Sub-system) gives the area that generated the error and the ECode (aka Error Code) tells you what the error was specifically.  The list of subsystems and error codes can be found at the bottom of the Copter `defines.h file <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/defines.h#L339>`__.

Sub Systems / Error Codes

1: Main (never used)

2: Radio

ECode 1: "Late Frame" which means the APM's onboard ppm encoder did not
provide an update for at least 2 seconds

ECode 0: error resolved which means the ppm encoder started providing
data again

3: Compass

ECode 1: the compass failed to initialise (likely a hardware issue)

ECode 2: failure while trying to read a single value from the compass
(probably a hardware issue)

ECode 0: above errors resolved

4: Optical flow

Ecode 1: failed to initialise (likely a hardware issue)

5: Throttle failsafe

ECode 1: throttle dropped below FS_THR_VALUE meaning likely loss of
contact between RX/TX

ECode 0: above error resolve meaning RX/TX contact likely restored

6: Battery failsafe

ECode 1: battery voltage dropped below LOW_VOLT or total battery
capacity used exceeded BATT_CAPACITY

7: GPS failsafe

ECode 1: GPS lock lost for at least 5 seconds

ECode 0: GPS lock restored

8: GCS (Ground station) failsafe

ECode 1: updates from ground station joystick lost for at least 5
seconds

ECode 0: updates from ground station restored

9: Fence

ECode 1: altitude fence breached

ECode 2: circular fence breached

ECode 3: both altitude and circular fences breached

ECode 0: vehicle is back within the fences

10: Flight Mode

ECode 0 ~ 22: the vehicle was unable to enter the desired flight mode (the mapping of code to flight mode can be found here in `Copter's defines.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/defines.h#L34>`__)

11: GPS

ECode 2: GPS Glitch

ECode 0: GPS Glitch cleared

12: Crash Check

ECode 1: Crash detected

13: Flip

ECode 2: Flip abandoned (because of 2 second timeout)

14: AutoTune

ECode 2: Bad Gains (failed to determine proper gains)

15: Parachute

ECode 2: Too low to deploy parachute

16: EKF/InertialNav Check

ECode 2: Bad Variance

ECode 0: Bad Variance cleared

17: EKF Failsafe

ECode 2: EKF Failsafe triggered

18: Baro glitch

ECode 2: Baro glitch

ECode 0: Baro glitch cleared
