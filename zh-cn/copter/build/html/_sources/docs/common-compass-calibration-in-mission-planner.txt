.. _common-compass-calibration-in-mission-planner:

======================================
Compass Calibration in Mission Planner
======================================

This article explains how to perform basic compass calibration.

.. note::

   This article assumes that you're the most common configuration — a
   Pixhawk, PX4 or APM2.6 board with the compass mounted with the the arrow
   facing toward the front of the vehicle and in the same direction as the
   arrow on the autopilot. If you're using a different configuration see
   :ref:`Advanced Compass Setup <common-compass-setup-advanced>`.



Calibration steps
=================

#. Under **Initial Setup \| Mandatory Hardware** select **Compass**.
#. Select your autopilot configuration to automatically enter the most
   important configuration information for your board:

   -  For Pixhawk and PX4, select the button **Pixhawk/PX4**.
   -  For APM 2.6, select **APM with External Compass**.

   .. figure:: ../../../images/MissionPlanner_CompassCalibration_MainScreen.png
      :target: ../_images/MissionPlanner_CompassCalibration_MainScreen.png

      Mission Planner: Compass Calibration

   You shouldn't need to do any more configuration, but you might want
   to confirm that the \ **Enable compasses**, **Obtain declination
   automatically** and **Automatically learn offsets** check boxes are
   checked.

#. Click the **Live Calibration** button.

   A window should pop-up showing you the state of the live calibration.

   .. figure:: ../../../images/MissionPlanner_CompassCalibration_LiveCalibrationScreen.png
      :target: ../_images/MissionPlanner_CompassCalibration_LiveCalibrationScreen.png

      Mission Planner: Live Compass Calibration

#. Hold the vehicle in the air and rotate it slowly so that each side
   (front, back, left, right, top and bottom) points down towards the
   earth for a few seconds in turn.

   .. figure:: ../../../images/accel-calib-positions-e1376083327116.jpg
      :target: ../_images/accel-calib-positions-e1376083327116.jpg

      Compass Calibration Positions (shown for Copter, but true for all vehicles)

#. The calibration will automatically complete when it has data for all
   the positions. At this point, another window will pop up telling you
   that it is saving the newly calculated offsets. These are displayed
   on the main screen below each associated compass.

   .. note::

      In Copter-3.2.1 and later offsets are considered acceptable
         provided their combined "length" is less than 600 (i.e.
         *sqrt(offset_x^2+offset_y^2+offset_Z^2) < 600*). Prior to Copter
         3.2.1 the recommendation was that the absolute value of each offset
         be less than 150 (i.e. *-150 < offset < 150*).

Additional information
======================

More information about compass configuration can be found in :ref:`Advanced Compass Setup <common-compass-setup-advanced>`. This includes
instructions for how to set up additional compasses, non-standard
compass alignments,

:ref:`compassmot <common-compass-setup-advanced_compassmot_compensation_for_interference_from_the_power_wires_escs_and_motors>`,

etc.

General discussion on magnetic interference and ways to reduce it can be
found in :ref:`Magnetic Interference <common-magnetic-interference>`.

Video demonstration
===================

Video demonstration of compass calibration.

..  youtube:: DmsueBS0J3E
    :width: 100%

