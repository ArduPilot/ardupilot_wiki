.. _common-accelerometer-calibration:

=========================
Accelerometer Calibration
=========================

[copywiki destination="copter,plane,rover,planner"]

This article shows how to perform basic accelerometer calibration (using
*Mission Planner*).

.. attention::

   Accelerometer calibration is mandatory in ArduPilot.

.. important::

   Accelerometer calibration cannot be performed while vehicle is armed.

Calibration steps
=================

.. warning:: If the board is mounted in a non-standard orientation (i.e. arrow is not pointing forward) then please ensure the :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>`  is properly set before doing the accelerometer calibration.

- Under **Setup \| Mandatory Hardware**, select **Accel Calibration** from the left-side menu.

   .. figure:: ../../../images/mp_accelerometer_calibration.png
      :target: ../_images/mp_accelerometer_calibration.png

      Mission Planner: Calibrate Acceleration

- Click **Calibrate Accel** to start the calibration.

   *Mission Planner* will prompt you to place the vehicle each
   calibration position. Press any key to indicate that the autopilot is
   in position and then proceed to the next orientation.

   The calibration positions are: level, on right side, left side, nose
   down, nose up and on its back.

   .. figure:: ../../../images/accel-calib-positions-e1376083327116.jpg
      :target: ../_images/accel-calib-positions-e1376083327116.jpg

      Accelerometer Calibration Positions (Copter)

   It is important that the vehicle is kept still immediately after pressing the key for each step. This is more important than getting the angle exactly right, ie. left being 90deg to horizontal, etc. Except for the first "LEVEL", the positions can be within 20 degs of being exact. Being still in each position as you press the key is much more important.

   -  You may need to calibrate the board before it is mounted if the size/shape of the vehicle makes this difficult.
   -  The level position is the most important to get right as this will be the attitude that your controller considers level while flying.

[site wiki="plane"]
   .. tip:: For planes, the "level" angle for pitch is important for steady altitude flight. See :ref:`tuning-cruise` for more details.

 [/site]

- Proceed through the required positions, using the **Click when Done** button once each position is reached and held still.

- When you have completed the calibration process, Mission Planner will display "Calibration Successful!" as shown below.

   .. figure:: ../../../images/mp_calibration_successful.png
      :target: ../_images/mp_calibration_successful.png

      Mission Planner: Calibration Successful

.. note:: If your autopilot has a built-in IMU heater, then it is recommended that the :ref:`common-imutempcal` also be done. Some autopilots have this calibration done at the factory in which case, this calibration need not be done.


Video demonstration (Copter)
============================

Video demonstration of accelerometer calibration. This is for an older
version of Copter/Mission Planner, but is useful as an example of how
you might hold a Copter.

.. vimeo:: 56224615
   :width: 400
   :height: 400

