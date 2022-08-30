.. _common-optical-flow-sensor-setup:

[copywiki destination="copter,plane"]

=====================================
Optical Flow Sensor Testing and Setup
=====================================

Testing the sensor
==================

With the sensor connected to the autopilot, connect to the autopilot with the Mission Planner and open the Flight Data screen's Status tab.  If the sensor is operating you should see non-zero opt_m_x, opt_m_y and opt_qua values.

.. image:: ../../../images/PX4Flow_CheckForData_MP.png
    :target: ../_images/PX4Flow_CheckForData_MP.png

Calibrating the sensor
======================
#. Connect to your autopilot and ensure that logging while disarmed is enabled by setting :ref:`LOG_DISARMED <copter:LOG_DISARMED>` to 1
#. Find a location with a textured surface and good lighting (natural light or strong incandescent)
#. Remove Propellers (safety first)
#. Power on the vehicle and hold level away from your body and at eye level
#. Rotate about the vehicle in roll through a range from -15 to +15
   degrees in about a second and then back again. Repeat this 5 to 10
   times. By closing one eye you will be able to keep the centre of the
   sensor stationary against the background while you do the rotation.
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

Range Sensor Check
==================

Check the ``EKF5.meaRng`` message in the flashlog from your flow
sensor calibration test. Check the following:

#. There is continuous range measurement.
#. It outputs a range on the ground that is 10cm of the expected value
   (remember that measured range will increase when you roll or pitch
   the vehicle because the laser is then measuring on a slant)

Pre-Arm checks
==============

To allow arming and taking off in Loiter without a GPS the GPS arming
check should be turned off as shown below.  Unchecking "All" and "GPS"
and leave all other options checked.

.. image:: ../../../images/OptFlow_ArmingChecks.png
    :target: ../_images/OptFlow_ArmingChecks.png

Because optical flow requires good sonar/range finder data when the
optical flow is enabled, an additional pre-arm check is enforced.

**While the vehicle is disarmed you should lift the vehicle straight up
to at least 50cm but no higher than 2m** (if the rangefinder sees a
distance of over 2m you will need to restart the autopilot).

The error message when arming fails this check is "PreArm: check range
finder"

This check can be disabled by unchecking the "Parameter/Sonar" arming
check.

First Flight
============

#. For EKF2, set :ref:`EK2_GPS_TYPE <EK2_GPS_TYPE>` = 0; for EKF3, set :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 0 (we don't want the optical flow being used by the EKF at this stage) 
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

#. For EKF2, set :ref:`EK2_GPS_TYPE <EK2_GPS_TYPE>` = 3; for EKF3, set :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 5 and :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 0 to make the EKF ignore GPS and use the flow sensor
#. Ensure you have a loiter and hover mode available on you transmitter.
#. Set "EKF Origin" on Ground Control Station map. In Mission Planner, right click, select "Set Home here", and choose to set "set EKF origin here".
#. Take-off in loiter and bring the Copter/QuadPlane to about 1m height
#. If the vehicle starts to accelerate away or there is erratic pitch or roll
   movement, then switch to hover and land. You will need to
   download the log file and share it on `the forums <https://discuss.ardupilot.org/c/arducopter>`__ to understand why.
#. If it holds location then congratulations, you have succeeded and can
   now start experimenting with height changes and moving it around in
   the loiter mode

Setup for Regular Flight
========================

For EKF2:
---------

#. Verify that :ref:`EK2_ENABLE <EK2_ENABLE>` = 1, enabling EKF2
#. Set :ref:`EK2_FLOW_DELAY <EK2_FLOW_DELAY>` depending on your optical flow sensor
#. To only use the optical flow sensor and not use the GPS, set :ref:`EK2_GPS_TYPE <EK2_GPS_TYPE>` = 3; to use the GPS with the optical flow sensor, set this to 0.

For EKF3:
---------
.. note::

   EKF3 is enabled and used by default in ArduPilot firmware 4.1 and higher

#. Verify that :ref:`EK3_ENABLE <EK3_ENABLE>` = 1, enabling EKF3
#. Set :ref:`AHRS_EKF_TYPE <AHRS_EKF_TYPE>` = 3 to use EKF3
#. Set :ref:`EK3_SRC_OPTIONS <EK3_SRC_OPTIONS>` = 0 to disable FuseAllVelocities
#. Set :ref:`EK3_FLOW_DELAY <EK3_FLOW_DELAY>` depending on your optical flow sensor
#. Set :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 3 (Primary horizontal position from GPS, set this to 0 to only use the optical flow sensor)
#. Set :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 5 (Primary horizontal velocity from OpticalFlow)
#. Set :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` = 1 (Primary vertical position from barometer)
#. Set :ref:`EK3_SRC1_VELZ <EK3_SRC1_VELZ>` = 0 (No primary vertical velocity sensor)
#. Set :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 1 (Primary yaw/heading from compass)

- Alternatively, GPS can work with OpticalFlow using EKF source switching.
- See :ref:`GPS / Non-GPS Transitions <common-non-gps-to-gps>` for information on EKF source switching.

Example Video (Copter-3.4)
==========================

..  youtube:: Bzgey8iR69Q
    :width: 100%

---------------------------------

Inflight Calibration
====================

Copter-4.2.0 includes an inflight calibration procedure:

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

.. note::

   To use the inflight calibration EKF3 must be enabled.  This is the default for ArduPilot 4.1 and higher

..  youtube:: Crx97v1bwWo
    :width: 100%
