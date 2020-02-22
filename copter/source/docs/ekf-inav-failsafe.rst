.. _ekf-inav-failsafe:

============
EKF Failsafe
============

The EKF failsafe monitors the health of EKF (the position and attitude estimation system) to catch problems with the vehicle's position estimate (often caused by GPS glitches or compass errors) and prevent "flyaways".

.. note::

   Starting in Copter 3.3 the EKF failsafe replaces the :ref:`GPS Failsafe <archived-gps-failsafe>`. 

When will it trigger?
=====================

The EKF failsafe will trigger when any two of the EKF "variances" for compass, position or velocity are higher than the :ref:`FS_EKF_THRESH <FS_EKF_THRESH>` parameter value for 1 second.

These "variances" come from the EKF itself and are numbers indicating the EKF's confidence in its estimates.  The values are between 0 and 1 with 0 meaning the estimate is very trustwrothy and 1.0 is very untrustworthy.

The EKF calculates these "variances" by comparing the results from multiple sensors.  So for example, if the GPS position suddenly jumps but the accelerometers do not show a sudden acceleration, the EKF variance for position would climb (i.e become less trustworthy)

The variances can be viewed in real-time on the ground station.  If using Mission Planner click on the "EKF" label on the HUD.

.. image:: ../images/ekf-failsafe-variance-viewer.png
    :target: ../_images/ekf-failsafe-variance-viewer.png
    :width: 450px

What will happen when the failsafe triggers?
============================================

- The autopilot's `LED will flash red-yellow or blue-yellow and the tone-alarm will sound <https://www.youtube.com/watch?v=j-CMLrAwlco&feature=player_detailpage#t=60>`__
- "EKF variance" will appear on the ground station's HUD if telemetry is connected
- In manual flight modes that do not require GPS (i.e. Stabilize, Acro, AltHold) nothing further will happen but the pilot will be unable to switch into autonomous flight modes (Loiter, PosHold, RTL, Guided, Auto) until the failure clears
- In autonomous modes that require GPS (i.e. Loiter, PosHold, RTL, Guided, Auto, etc) the :ref:`FS_EKF_ACTION <FS_EKF_ACTION>` controls the behaviour.  By default this is "1" meaning the vehicle will switch to :ref:`Land <land-mode>` mode.  This is a "pilot controlled" land meaning the pilot will have control of the roll and pitch angle but the vehicle will descend at the :ref:`LAND_SPEED <LAND_SPEED>`.  It will land and finally disarm its motors
- An EKF failsafe error will be written to the dataflash logs

After an EKF failsafe occurs, the pilot can re-take control (using the flight mode switch) in a manual flight mode such as :ref:`AltHold <altholdmode>` to bring the vehicle home.

Adjusting the Sensitivity of the failsafe
=========================================

The :ref:`FS_EKF_THRESH <FS_EKF_THRESH>` parameter can be adjusted to control the sensitivity of the failsafe

- Set :ref:`FS_EKF_THRESH <FS_EKF_THRESH>` = 0 to disable the EKF failsafe
- Increase :ref:`FS_EKF_THRESH <FS_EKF_THRESH>` to values between 0.8 and 1.0 to reduce the chance of an EKF failsafe.  The downside of increasing this parameter value is that during a flyaway caused by a bad compass or GPS glitch, the vehicle will fly further away before the vehicle is automatically switched to LAND mode
- Decrease :ref:`FS_EKF_THRESH <FS_EKF_THRESH>` to values as low as 0.6 to increase the chance of an EKF failsafe triggering quickly.  The downside of lowering this value is the EKF failsafe could trigger a LAND during aggressive maneuvers

Dataflash Log example
=====================

The EKF's innovations can be viewed by graphing a dataflash log's NKF4.SP (position innovation), NKF4.SV (velocity innovation) and NKF4.SM (compass innovation) values

The graph below show the EKF's innovations for position (green), velocity (red) and compass (yellow) during an actual EKF failsafe event.  During this incident external interference (probably from a high-powered radio tower nearby) caused the GPS to report inaccurate positions and velocities.  The vehicle switches to Land mode soon after both velocity and position innovations climb over the :ref:`FS_EKF_THRESH<FS_EKF_THRESH>` value of 0.8

.. image:: ../images/ekf-failsafe-example-log.png
    :target: ../_images/ekf-failsafe-example-log.png
    :width: 450px

Video
=====

..  youtube:: zJbephAEFWQ
    :width: 100%

.. toctree::
    :maxdepth: 1

    GPS Failsafe and Glitch Protection <gps-failsafe-glitch-protection>
