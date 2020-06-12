.. _loiter-mode:

===========
Loiter Mode
===========

Loiter Mode automatically attempts to maintain the current location, heading and altitude. The pilot may fly the copter in Loiter mode as if it were in a more manual flight mode but when the sticks are released, the vehicle will slow to a stop and hold position.

A good GPS lock, :ref:`low magnetic interference on the compass <common-diagnosing-problems-using-logs_compass_interference>` and :ref:`low vibrations <common-diagnosing-problems-using-logs_vibrations>` are all important in achieving good loiter performance.

..  youtube:: yVAnBQkNJdY?start=261
    :width: 100%

Controls
========

The pilot can control the copter's position with the control sticks.

-  Horizontal location can be adjusted with the the Roll and Pitch
   control sticks with the default maximum horizontal speed being 5m/s
   (see Tuning section below on how to adjust this).  When the pilot
   releases the sticks the copter will slow to a stop.
-  Altitude can be controlled with the Throttle control stick just as in
   :ref:`AltHold mode <altholdmode_controls>`
-  The heading can be set with the Yaw control stick

The vehicle can be armed in Loiter mode but only once the GPS has 3D lock and the HDOP has dropped below 2.0.  :ref:`More details on LED patterns here <common-leds-pixhawk>`.

.. _loiter-mode_tuning:

Tuning
======

.. image:: ../images/Loiter_Tuning.png
    :target: ../_images/Loiter_Tuning.png

Loiter mode incorporates the altitude controller from AltHold mode. 
Details for tuning :ref:`AltHold are on this wiki page <altholdmode_tuning>`.

Loiter Parameters
-----------------

- :ref:`LOIT_SPEED <LOIT_SPEED>` : max horizontal speed in cm/s (i.e. 1250 = 12.5m/s)
- :ref:`LOIT_ACC_MAX <LOIT_ACC_MAX>` : max acceleration in cm/s/s.  Higher values cause the copter to accelerate and stop more quickly
- :ref:`LOIT_ANG_MAX <LOIT_ANG_MAX>` : max lean angle in centi-degrees (i.e. 3000 = 30deg).  By default this value is zero which causes the :ref:`ANGLE_MAX <ANGLE_MAX>` parameter's value to be used
- :ref:`LOIT_BRK_ACCEL <LOIT_BRK_ACCEL>`: max acceleration in cm/s/s while braking (i.e. pilot has moved sticks to center).  Higher values will stop the vehicle more quickly
- :ref:`LOIT_BRK_DELAY <LOIT_BRK_DELAY>`: the delay in seconds before braking starts once the pilot has centered the sticks
- :ref:`LOIT_BRK_JERK <LOIT_BRK_JERK>`: max change in acceleration in cm/s/s/s while braking.  Higher numbers will make the vehicle reach the maximum braking angle more quickly, lower numbers will cause smoother braking
- :ref:`PSC_POSXY_P <PSC_POSXY_P>` : (shown as "Position XY (Dist to Speed)" at the top right of the screen shot above) converts the horizontal position error (i.e difference between the desired position and the actual position) to a desired speed towards the target position.  **It is generally not required to adjust this**
- :ref:`PSC_VELXY_P <PSC_VELXY_P>` (shown as "Velocity XY (Vel to Accel)") converts the desired speed towards the target to a desired acceleration.  The resulting desired acceleration becomes a lean angle which is then passed to the same angular controller used by :ref:`Stabilize mode <stabilize-mode>`.  **It is generally not required to adjust this**


Common Problems
===============

#. The vehicle `circles (aka "toiletbowls") <https://www.youtube.com/watch?v=a-3G9ZvXHhk>`__.  This
   is normally caused by a compass problem the most likely being
   :ref:`magnetic interference <common-diagnosing-problems-using-logs_compass_interference>`
   from the power cables under the autopilot.  Running
   :ref:`compassmot <common-compass-setup-advanced_compassmot_compensation_for_interference_from_the_power_wires_escs_and_motors>`
   or purchasing a :ref:`GPS+compass module <common-installing-3dr-ublox-gps-compass-module>` normal
   resolves this.  Other possibilities include bad compass offsets set
   during the :ref:`compass calibration process <common-compass-calibration-in-mission-planner>`.
#. The vehicle takes off in the wrong direction as soon as loiter is engaged.  The cause is the same as #2 except that the compass error is greater than 90deg.  Please try the suggestions above to resolve this.


Verifying Loiter performance with dataflash logs
================================================

Viewing the loiter's horizontal performance is best done by :ref:`downloading a dataflash log <common-downloading-and-analyzing-data-logs-in-mission-planner>` from your flight, then open it with the mission planner and graph the NTUN message’s DesVelX vs VelX and DesVelY vs VelY.  In a good performing copter the actual velocities will track the desired velocities as shown below.  X = latitude (so positive = moving North, negative = South), Y = longitude (positive = East, negative = West).

.. image:: ../images/Loiter_TuningCheck.png
    :target: ../_images/Loiter_TuningCheck.png

Checking altitude hold performance is the same as for :ref:`AltHold <altholdmode_verifying_althold_performance_with_dataflash_logs>` mode.

