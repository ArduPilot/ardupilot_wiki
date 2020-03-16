.. _common-compass-calibration-in-mission-planner:

===================
Compass Calibration
===================

This article explains how to perform basic compass calibration. It assumes that you have at least one compass, either internally or externally in the system, and it has been enabled. See :ref:`Advanced Compass Setup <common-compass-setup-advanced>` for more information and to setup other compass related features.

.. tip::

   It is not necessary to recalibrate the compass when the vehicle is flown at a new location because ArduPilot includes a "world magnetic model" which allows converting the location's magnetic North to true North without recalibrating if using the option "Obtain declination automatically".  In addition the location's "inclination" is calibrated at startup and then again soon after takeoff.
   
.. note::

   Compass calibration cannot be performed while vehicle is armed.


Calibration first steps
=======================


.. warning:: Do not calibrate the compasses near any metallic or magnetic field producing object (computers, cell phones, metal desks, power supplies, etc.) or incorrect calibration will occur.


- Under **Initial Setup \| Mandatory Hardware** select **Compass**.

   .. figure:: ../../../images/MissionPlanner_CompassCalibration_MainScreen.png
      :target: ../_images/MissionPlanner_CompassCalibration_MainScreen.png

      Mission Planner: Compass Calibration

   You normally shouldn't need to change any of the "General Compass Settings" or compass specific values (i.e. "Compass #1" section), but you might want to confirm that the \ **Enable compasses** and **Obtain declination automatically** boxes are checked.

   You may wish to disable any internal compasses if you are consistently seeing the "inconsistent compasses" pre-arm message often and you are sure that the external compass is calibrated and the orientation is correct.

.. _onboard_calibration:

Onboard Calibration
===================

Copter-3.4, Plane 3.4.0, and Rover 3.0.0 (and higher) support "Onboard Calibration" meaning that the calibration routine runs on the autopilot.  This method is more accurate than the older "Offboard Calibration" (aka "Live Calibration") which runs on the ground station because in addition to offsets, scaling is also calculated.

.. note:: A big advantage of using Onboard Calibration is that the orientation of external compasses is automatically determined during the process, as well as verification of the orientation of onboard compasses. Calibration could fail if the onboard compasses are not oriented as set by the :ref:`autopilot board orientation<AHRS_ORIENTATION>` parameter.

   .. figure:: ../../../images/CompassCalibration_Onboard.png
      :target: ../_images/CompassCalibration_Onboard.png

To perform the onboard calibration:

- click the "Onboard Mag Calibration" section's "Start" button
- if your autopilot has a buzzer attached you should hear a single tone followed by short beep once per second
- hold the vehicle in the air and rotate it so that each side (front, back, left, right, top and bottom) points down towards the earth for a few seconds in turn. Consider a full 360-degree turn with each turn pointing a different direction of the vehicle to the ground. It will result in 6 full turns plus possibly some additional time and turns to confirm the calibration or retry if it initially does not pass.

   .. figure:: ../../../images/accel-calib-positions-e1376083327116.jpg
      :target: ../_images/accel-calib-positions-e1376083327116.jpg

- as the vehicle is rotated the green bars should extend further and further to the right until the calibration completes
- upon successful completion three rising tones will be emitted and a "Please reboot the autopilot" window will appear and you will need to reboot the autopilot before it is possible to arm the vehicle.

.. note: In Copter 3.6.0, Plane 3.9.0, and Rover 3.5.0 and later versions, the compass orientation is also automatically determined upon successful completion of Onboard Calibration.

If calibration fails:

- you will hear an "unhappy" failure tone, the green bars may reset to the left, and the calibration routine may restart (depending upon the ground station). Mission Planner will automatically retry, so continue to rotate the vehicle as instructed above.
- if a compass is not calibrating, consider moving to a different area away from magnetic disturbances, and remove electronics from your pockets.
- if, after multiple attempts, the compass has not passed the calibration, Press the "Cancel" button and change the "Fitness" drop-down to a more relaxed setting and try again.
- if compass calibration still fails it may help to raise :ref:`COMPASS_OFFS_MAX <COMPASS_OFFS_MAX>` from 850 to 2000 or even 3000
- finally, if a single compass is not calibrating and you trust the others, disable it.

Onboard Calibration using Stick Gestures (no GCS)
=================================================
Copter-3.4, Plane 3.4.0, and Rover 3.0.0 (and higher) support "Onboard Calibration using RC Controller stick gestures" meaning that the calibration routine runs on the autopilot with no GCS.  This method is more accurate than the older "Offboard Calibration" (aka "Live Calibration") which runs on the ground station because in addition to offsets, scaling is also calculated.

- requires RC calibration first
- to start compass calibration hold throttle stick full up and full right yaw for 2 seconds
- if your autopilot has a buzzer attached you should hear a single tone followed by short beep once per second
- hold the vehicle in the air and rotate it so that each side (front, back, left, right, top and bottom) points down towards the earth for a few seconds in turn

   .. figure:: ../../../images/accel-calib-positions-e1376083327116.jpg
      :target: ../_images/accel-calib-positions-e1376083327116.jpg

- upon successful completion three rising tones will be emitted and you will need to reboot the autopilot before it is possible to arm the vehicle.

If calibration fails:

- you will hear a failure tone and the calibration routine will restart
- to cancel calibration at anytime hold throttle stick full up and full left yaw for 2 seconds
- if, after multiple attempts, you are unable to calibrate the compass, Cancel with stick and use normal Onboard Calibration from GCS above

Offboard Calibration (aka "Live Calibration")
=============================================

.. note:: This method is not recommended if you are using a recent version of ArudPilot that supports the Onboard Calibration described above.

Older versions of ArduPilot rely on the ground station to calculate the compass offsets.  To use this older method:

- Click the **Live Calibration** button.

   A window should pop-up showing you the state of the live calibration.

   .. figure:: ../../../images/MissionPlanner_CompassCalibration_LiveCalibrationScreen.png
      :target: ../_images/MissionPlanner_CompassCalibration_LiveCalibrationScreen.png

      Mission Planner: Live Compass Calibration

- The aim is to rotate the vehicle so that the coloured trail hits each of the white dots.  One way to do this is to hold the vehicle in the air and rotate it slowly so that each side (front, back, left, right, top and bottom) points down towards the earth for a few seconds in turn.

   .. figure:: ../../../images/accel-calib-positions-e1376083327116.jpg
      :target: ../_images/accel-calib-positions-e1376083327116.jpg

      Compass Calibration Positions (shown for Copter, but true for all vehicles)

- The calibration will automatically complete when it has data for all the positions. At this point, another window will pop up telling you that it is saving the newly calculated offsets. These are displayed on the main screen below each associated compass.

   .. note::

      In Copter-3.2.1 and later offsets are considered acceptable
         provided their combined "length" is less than 600 (i.e.
         *sqrt(offset_x^2+offset_y^2+offset_Z^2) < 600*). Prior to Copter
         3.2.1 the recommendation was that the absolute value of each offset
         be less than 150 (i.e. *-150 < offset < 150*).

Additional information
======================

More information about compass configuration can be found in :ref:`Advanced Compass Setup <common-compass-setup-advanced>`. This includes instructions for how to set up additional compasses, :ref:`automatic setting of offsets<automatic-compass-offset-calibration>`, non-standard compass alignments, :ref:`compassmot <copter:common-compass-setup-advanced_compassmot_compensation_for_interference_from_the_power_wires_escs_and_motors>`, etc.

General discussion on magnetic interference and ways to reduce it can be
found in :ref:`Magnetic Interference <common-magnetic-interference>`.

Video demonstration
===================

Video demonstrations of compass calibration.

..  youtube:: CD8EhVDfgnI
    :width: 100%

..  youtube:: DmsueBS0J3E
    :width: 100%

[copywiki destination="copter,plane,rover,planner"]
