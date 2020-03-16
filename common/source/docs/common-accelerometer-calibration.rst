.. _common-accelerometer-calibration:

=========================
Accelerometer Calibration
=========================

[copywiki destination="copter,plane,rover,planner"]

This article shows how to perform basic accelerometer calibration (using
*Mission Planner*).

.. note::

   Accelerometer calibration is mandatory in ArduPilot.

.. note::

   Accelerometer calibration cannot be performed while vehicle is armed

Calibration steps
=================

.. warning:: If the board is mounted in a non-standard orientation (i.e. arrow is not pointing forward) then please ensure the :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>`  is properly set before doing the accelerometer calibration.

#. Under **Initial Setup \| Mandatory Hardware**, select **Accel
   Calibration** from the left-side menu.

   .. figure:: ../../../images/mp_accelerometer_calibration.png
      :target: ../_images/mp_accelerometer_calibration.png

      Mission Planner: Calibrate Acceleration

#. Click **Calibrate Accel** to start the calibration.

   *Mission Planner* will prompt you to place the vehicle each
   calibration position. Press any key to indicate that the autopilot is
   in position and then proceed to the next orientation.

   The calibration positions are: level, on right side, left side, nose
   down, nose up and on its back.

   .. figure:: ../../../images/accel-calib-positions-e1376083327116.jpg
      :target: ../_images/accel-calib-positions-e1376083327116.jpg

      Accelerometer Calibration Positions (Copter)

[/site]

   .. tip::

         -  It is important that the vehicle is kept still immediately after
            pressing the key for each step.
         -  You may need to calibrate the board before it is mounted if the
            size/shape of the vehicle makes this difficult.
         -  The level position is the most important to get right as this will
            be the attitude that your controller considers level while flying.

#. Proceed through the required positions (it is not necessary to use
   the **Click When Done** button).

   .. figure:: ../../../images/mp_accel_calibration_press_any_key.jpg
      :target: ../_images/mp_accel_calibration_press_any_key.jpg

      Mission Planner: Accel Calibration - press any key

#. When you've completed the calibration process, Mission Planner will
   display "Calibration Successful!" as shown below.

   .. figure:: ../../../images/mp_calibration_successful.jpg
      :target: ../_images/mp_calibration_successful.jpg

      Mission Planner: Calibration Successful


Video demonstration (Copter)
============================

Video demonstration of accelerometer calibration. This is for an older
version of Copter/Mission Planner, but is useful as an example of how
you might hold a Copter.

.. vimeo:: 56224615
   :width: 400
   :height: 400

[/site]
