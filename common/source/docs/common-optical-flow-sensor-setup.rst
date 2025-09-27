.. _common-optical-flow-sensor-setup:

[copywiki destination="copter,plane,rover"]

=====================================
Optical Flow Sensor Testing and Setup
=====================================

This page describes how to test and calibrate an optical flow sensor.  These instructions should normally be followed after performing the setup described on the individual sensor setup pages.

Testing the sensor
==================

With the sensor connected to the autopilot, connect to the autopilot with the Mission Planner and open the Flight Data screen's Status tab.  If the sensor is operating you should see non-zero opt_m_x, opt_m_y and opt_qua values.

.. image:: ../../../images/PX4Flow_CheckForData_MP.png
    :target: ../_images/PX4Flow_CheckForData_MP.png

If the vehicle does not appear on the GCS map, please check the following:

- Check the rangefinder is operating correctly (see below)
- Use the Ground Station to set the EKF origin.  If using Mission Planner, right mouse button click on the map, select "Set Home here", and choose, "set EKF origin here"

Once the vehicle appears on the map, move the vehicle horizontally and confirm the vehicle icon on the map moves accordingly

[site wiki="plane,copter"]
Rangefinder Check
=================

Check the rangefinder's reported distance is between the :ref:`RNGFND1_MIN <RNGFND1_MIN>` and :ref:`RNGFND1_MAX <RNGFND1_MAX>` parameter values when on the ground and when held up

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
    :width: 250px

You may also download an onboard log and check the ``EKF5.meaRng`` message to confirm

#. There is continuous range measurement
#. It outputs a range on the ground that is within 10cm of the expected value
   (remember that measured range will increase when you roll or pitch
   the vehicle because the laser is then measuring on a slant)

[/site]

Inflight Calibration
====================

..  youtube:: Crx97v1bwWo
    :width: 100%

If the vehicle has a GPS, the inflight calibration procedures is the easiest way to get a good calibration:

- Set :ref:`RCx_OPTION <RC8_OPTION>` = 158 (Optflow Calibration) to allow starting the calibration from an :ref:`auxiliary switch <common-auxiliary-functions>`
- Setup the EKF3 to use GPS (the default)

  - :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 3 (GPS)
  - :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` = 1 (Baro)
  - :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 3 (GPS)
  - :ref:`EK3_SRC1_VELZ <EK3_SRC1_VELZ>` = 3 (GPS)
  - :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 1 (Compass)
  - :ref:`EK3_SRC_OPTIONS <EK3_SRC_OPTIONS>` = 0 (Disable FuseAllVelocities)

- Fly the vehicle in Loiter mode to at least 10m (higher is better but stay within the limits of the rangefinder)
- Pull the auxiliary switch high to start the calibration
- Use the roll and pitch sticks to rock the vehicle back and forth in both roll and pitch
- Check the GCS "Messages" tab for output like below confirming the calibration is complete

::

   FlowCal: Started
   FlowCal: x:0% y:0%
   FlowCal: x:66% y:6%
   FlowCal: x:100% y:74%
   FlowCal: samples collected
   FlowCal: scalarx:0.976 fit: 0.10   <-- lower "fit" values are better
   FlowCal: scalary:0.858 fit: 0.04
   FlowCal: FLOW_FXSCALER=30.00000, FLOW_FYSCALER=171.0000

- Land the vehicle and setup the EKF3 to use OpticalFlow

  - :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 0 (None)
  - :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 5 (Optical Flow)
  - :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` = 1 (Baro)
  - :ref:`EK3_SRC1_VELZ <EK3_SRC1_VELZ>` = 0 (None)
  - :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 1 (Compass)
  - :ref:`EK3_SRC_OPTIONS <EK3_SRC_OPTIONS>` = 0 (Disable FuseAllVelocities)

- Fly the vehicle again to check performance

An alternative method which avoids the need to land and change EKF3 parameters between calibration and testing is to setup :ref:`GPS/Non-GPS transitions <common-non-gps-to-gps>` so the pilot can manually change between GPS and Optical Flow inflight.  The full parameter list is below assuming the pilot will engage the calibration using RC input 8 (a 2-position switch) and switch between GPS and Optical flow using RC input 9 (a 3-position switch)

  - :ref:`RC8_OPTION <RC8_OPTION>` = 158 (Optflow Calibration)
  - :ref:`RC9_OPTION <RC9_OPTION>` = 90 (EKF Pos Source) low is GPS, middle is OpticalFlow, high is unused
  - :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 3 (GPS)
  - :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` = 1 (Baro)
  - :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 3 (GPS)
  - :ref:`EK3_SRC1_VELZ <EK3_SRC1_VELZ>` = 3 (GPS)
  - :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 1 (Compass)
  - :ref:`EK3_SRC2_POSXY <EK3_SRC1_POSXY>` = 0 (None)
  - :ref:`EK3_SRC2_VELXY <EK3_SRC1_VELXY>` = 5 (Optical Flow)
  - :ref:`EK3_SRC2_POSZ <EK3_SRC1_POSZ>` = 1 (Baro)
  - :ref:`EK3_SRC2_VELZ <EK3_SRC1_VELZ>` = 0 (None)
  - :ref:`EK3_SRC2_YAW <EK3_SRC1_YAW>` = 1 (Compass)
  - :ref:`EK3_SRC_OPTIONS <EK3_SRC_OPTIONS>` = 0 (Disable FuseAllVelocities)

Log based Calibration
=====================

To calibrate the sensor using Log file data please follow these steps:

#. Connect to your autopilot and ensure that logging while disarmed is enabled by setting :ref:`LOG_DISARMED <copter:LOG_DISARMED>` to 1
[site wiki="rover"]
#. Ensure the :ref:`FLOW_HGT_OVR <rover:FLOW_HGT_OVR>` parameter is set to the height of the sensor above ground
[/site]

#. Find a location with a textured surface and good lighting (natural light or strong incandescent)
   [site wiki="plane,copter"]
#. Remove Propellers (safety first)
#. Power on the vehicle and hold level away from your body and at eye level
#. Rotate about the vehicle in roll through a range from -15 to +15
   degrees in about a second and then back again. Repeat this 5 to 10
   times. By closing one eye you will be able to keep the centre of the
   sensor stationary against the background while you do the rotation.
   [/site]
   [site wiki="rover"]
#. Power on the vehicle
#. Rotate the vehicle (keeping it as close to the ground as practical)
   in roll through a range from -15 to +15
   degrees in about a second and then back again. Repeat this 5 to 10
   times. By closing one eye you will be able to keep the centre of the
   sensor stationary against the background while you do the rotation.
   [/site]
#. Repeat  about the vehicle pitch axis.
#. Download the data flash logs and plot the ``OF.flowX``, ``OF.bodyX``
   and ``IMU.GyrX`` data. It should look something like this:

   .. image:: ../../../images/OF-roll-calibration.png
       :target: ../_images/OF-roll-calibration.png
    
#. If ``OF.flowX`` is larger or smaller than ``OF.bodyX``, then it can
   be adjusted by changing the ``FLOW_FXSCALER`` parameter
#. IF ``OF.bodyX`` is uncorrelated or opposite sign to ``IMU.GyrX``,
   the :ref:`FLOW_ORIENT_YAW <copter:FLOW_ORIENT_YAW>` parameter is probably set incorrectly or
   you do not have the flow sensor pointing downwards
#. Plot the ``OF.flowY``, ``OF.bodyYband``, ``IMU.GyrY`` data. It should
   look something like this:

   .. image:: ../../../images/OF-pitch-calibration.png
       :target: ../_images/OF-pitch-calibration.png
   
#. If ``OF.flowY`` is larger or smaller than ``OF.bodyY``, then it can
   be adjusted by changing the ``FLOW_FYSCALER`` parameter
#. IF ``OF.bodyY`` is uncorrelated or opposite sign to ``IMU.GyrY``,
   the :ref:`FLOW_ORIENT_YAW <copter:FLOW_ORIENT_YAW>` parameter is probably set incorrectly or
   you do not have the flow sensor pointing downwards

First Flight
============

#. Set :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 0 (we don't want the optical flow being used by the EKF at this stage) 
#. Perform a short test flight hovering in STABILIZE or AltHold for copter, or QSTABILIZE or QHOVER for QuadPlane, at small lean angles at heights ranging from 50cm to 3m with 
#. Download the flash log and plot the following in mission planner
#. EKF5.meaRng should correlate with the change in vehicle height
#. ``OF.flowX`` and ``OF.flowY`` should be varying
#. ``OF.bodyX`` and ``OF.bodyY`` should be consistent with IMU.GyrX and IMU.GyrY

Second Flight
=============

.. warning::

   You will need at least 15m of clear space around the vehicle to do this flight safely.
   If the optical flow velocity estimates are bad, you will have little warning and the copter may lean to its maximum lean angle very quickly.

#. Set :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 5 and :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 0 to make the EKF ignore GPS and use the flow sensor
#. Ensure you have a loiter and hover mode available on you transmitter.
#. Set "EKF Origin" on Ground Control Station map. In Mission Planner, right click, select "Set Home here", and choose to set "set EKF origin here".
#. Take-off in loiter and bring the Copter/QuadPlane to about 1m height
#. If the vehicle starts to accelerate away or there is erratic pitch or roll
   movement, then switch to hover and land. You will need to
   download the log file and share it on `the forums <https://discuss.ardupilot.org/c/arducopter>`__ to understand why.
#. If it holds location then congratulations, you have succeeded and can
   now start experimenting with height changes and moving it around in
   the loiter mode

[/site]

Setup for Normal Operation
==========================

#. Set :ref:`EK3_SRC_OPTIONS <EK3_SRC_OPTIONS>` = 0 to disable FuseAllVelocities
#. Set :ref:`EK3_FLOW_DELAY <EK3_FLOW_DELAY>` depending on your optical flow sensor
#. Set :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 3 (Primary horizontal position from GPS, set this to 0 to only use the optical flow sensor)
#. Set :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 5 (Primary horizontal velocity from OpticalFlow)
#. Set :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` = 1 (Primary vertical position from barometer)
#. Set :ref:`EK3_SRC1_VELZ <EK3_SRC1_VELZ>` = 0 (No primary vertical velocity sensor)
#. Set :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 1 (Primary yaw/heading from compass)

- Alternatively, GPS can work with OpticalFlow using EKF source switching.
- See :ref:`GPS / Non-GPS Transitions <common-non-gps-to-gps>` for information on EKF source switching.
[site wiki="plane,copter"]
.. note:: When Copters have an optical flow sensor enabled (along with a rangefinder) and it is specified as the only horizontal position source (e.g. ``EK3_SRCx_VELXY``=OpticalFlow and ``EK3_SRCx_POSXY``=None) and the vehicle is flying in a pilot controlled mode requiring a position estimate (ie Loiter or PosHold) the vehicle will not climb above the rangefinder's maximum altitude specified in ``RNGFNDx_MAX``. This is a safety mechanism because otherwise the EKF failsafe would trigger as the vehicle flew out of rangefinder range.

Example Video (Copter-3.4)
==========================

..  youtube:: Bzgey8iR69Q
    :width: 100%

[/site]