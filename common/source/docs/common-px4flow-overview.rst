.. _common-px4flow-overview:

==========================================
PX4FLOW Optical Flow Camera Board Overview
==========================================

This article describes the `PX4FLOW (Optical Flow) Sensor <http://store.3drobotics.com/products/px4flow>`__.

.. warning::

   The PX4FLOW is supported as an experimental feature starting
   from Copter 3.3. It is not supported in Plane or Rover.

Overview
========

The `PX4FLOW (Optical Flow) Sensor <http://store.3drobotics.com/products/px4flow>`__ is a
specialized high resolution downward pointing camera module that uses
the ground texture and visible features and a rangefinder to determine
aircraft ground velocity. Although the sensor has plus a built-in
Maxbotix LZ-EZ4 sonar to measure height, this has not been reliable
enough over a range of surfaces in testing, so its readings are not used
and instead a separate :ref:`Range Finder <common-rangefinder-landingpage>`
such as the :ref:`Lidar Lite <common-rangefinder-lidarlite>`
should also be attached to the vehicle.

The built-in 3 axis gyro enables automatic compensation for vehicle tilt
and heading relative to the ground.  The PX4-FLOW incorporates the same
powerful Cortex M4F Microcontroller as is used in the PX4-FMU.

More information on this sensor including how to upgrade the sensor's
firmware using QGroundControl can be found on the `ETH's PX4FLOW wiki page <http://pixhawk.org/modules/px4flow>`__.

.. warning::

   To use this flow sensor to perform optical flow loiter as seen
   in the following video, you will need to purchase a separate range
   finder. Our testing has been performed using the `Pulsed Light unit <http://pulsedlight3d.com/>`__ 

..  youtube:: LP8kl4hGfMw
    :width: 100%

Upgrade the PX4Flow sensor's firmware
=====================================

The PX4Flow firmware must be updated prior to connecting to the Pixhawk:

#. Download and unzip the `PX4Flow-KLT firmware <http://firmware.ardupilot.org/downloads/wiki/advanced_user_tools/px4flow-klt-06Dec2014.zip>`__
   (`source code here <https://github.com/priseborough/px4flow/tree/klt_flow>`__)
#. `Download, install and start QGround Control <http://qgroundcontrol.org/downloads>`__
#. Select *PX4 Autopilot* from the splash screen

   .. image:: ../../../images/PX4Flow_FirmUpgrade1.png
       :target: ../_images/PX4Flow_FirmUpgrade1.png
       
#. Press the **Config** button to open the *Firmware Upgrade* screen

   .. figure:: ../../../images/PX4Flow_FirmUpgrade2.png
      :target: ../_images/PX4Flow_FirmUpgrade2.png

      QGroundControl: Firmware Upgrade Screen

#. Click **Advanced**, then set Port to "<Automatic>", Board to "PX4FLOW
   v1.1"
#. Push the **Select File** button and find the **.px4** file downloaded
   from step #1
#. Push the "Flash" firmware and then push the small reset button on the
   edge of the px4flow sensor near the sonar.

Connect to the Pixhawk
======================

.. image:: ../../../images/OptFlow_Pixhawk.jpg
    :target: ../_images/OptFlow_Pixhawk.jpg

The sensor should be connected to the Pixhawk's the 4-pin I2C port.  In
most cases an `I2C splitter <http://store.jdrones.com/Pixhawk_I2C_splitter_p/dstpx4i2c01.htm>`__
should be used to allow other I2C devices (like the external RGB LED and
GPS/Compass module's compass) to the same port.

Mounting to the Frame
=====================

Mount the flow sensor pointing straight down with the X axis forwards
and the Y axis to the right. The :ref:`FLOW_ORIENT_YAW <copter:FLOW_ORIENT_YAW>`
parameter can be used to account for other yaw orientations. It is
important that the flow sensor be mounted where it does not experience
angular angular vibration that could blur the image.

.. note::

   The default mounting orientation is different to that shown on
   the \ `ETH PX4FLOW wiki <http://pixhawk.org/modules/px4flow>`__. If you
   mount the board as shown in the \ `ETH PX4FLOW wiki <http://pixhawk.org/modules/px4flow>`__, you will need to set
   ``FLOW_ORIENT_YAW`` to -9000.

Setup through the Mission Planner
=================================

.. image:: ../../../images/OptFlow_MPSetup.png
    :target: ../_images/OptFlow_MPSetup.png

The sensor can be enabled by connecting with the Mission Planner and
then on the **Initial Setup \| Optional Hardware \| Optical Flow** page
check the **Enable** checkbox.  Alternatively the ``FLOW_ENABLE``
parameter should be set to "1" through the full parameters list.  The
sensor will be initialised once the Pixhawk board is rebooted.

Testing the sensor
==================

The raw data from the sensor will eventually be visible on the Mission
Planner's Flight Data screen's Status tab (`issue raised here <https://github.com/diydrones/MissionPlanner/issues/745>`__). 
Until then you can perform the following test in a well lit room. Note
that some artificial lighting has strobing that can reduce the sensor
performance. Natural light or a high power incandescent light source is
best.

.. warning::

   Follow the instructions on the \ `ETH PX4FLOW wiki page <http://pixhawk.org/modules/px4flow>`__ on how to use
   QGroundControl to check and adjust the focus of your sensor. Do not
   assume it was in focus as shipped. You should be pointing it at a high
   contrast object about 3m away when you check the focus.

.. warning::

   Disconnect the sensor I2C connection from your flight
   computer before you connect it to via USB to your computer, Otherwise
   the sensor will be trying to power the flight computer and it may not
   have enough voltage to work properly.

#. Remove the lens cap from the sensor
#. We recommend you upgrade to \ `this modified PX4FLOW sensor firmware <http://firmware.ardupilot.org/downloads/wiki/advanced_user_tools/px4flow-klt-06Dec2014.zip>`__.
   using QGroundControl. This modified firmware uses a different method
   to determine the optical flow and provides readings over a wider
   range of light and distance, but has a smaller maximum flow
   speed compared to the standard firmware.
#. Reconnect the I2C connection your flight computer  and change the
   following parameters: \ ``FLOW_ENABLE = 1`` turns on use of the
   sensor and \ ``LOG_BITMASK = 131071`` turns on pre-arm logging and
   will help diagnose problems
#. Power up the vehicle (you can power up via the flight computer USB)
   to start logging and rotate the vehicle with the sensor pointing at
   the floor.
#. Download the dataflash logs and graph the OF message's data including
   "Qual", flowX, flowY, bodyX and bodyY values. The quality reading
   should be greater than 0, and you should see the flow and body rates
   changing.
#. Power up the vehicle using the battery and repeat steps 4. and 5. If
   you get no data, but works when the Pixhawk is powered via USB, then
   it is likely you have encountered a start-up problem with the
   PX4-Flow sensor and the APM+Pixhawk combination that is known to
   affect some sensors. Currently there is no solution to this other
   than to always power up via USB first before connecting the battery.
   Please notify the APM developers and PX4Flow supplier if this occurs.

Flow Sensor Calibration
=======================

#. Find a location with a textured surface and good lighting (natural 
   light or strong incandescent)
#. Remove Propellers (safety first)
#. Power on the vehicle and hold level away from your body and at eye level
#. Rotate about the vehicle in roll through a range from -15 to +15
   degrees in about a second and then back again. Repeat this 10
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
   the ``FLOW_ORIENT_YAW`` parameter is probably set incorrectly or
   you do not have the flow sensor pointing downwards
#. Plot the ``OF.flowY``, ``OF.bodyYband``, ``IMU.GyrY`` data. It should
   look something like this:

   .. image:: ../../../images/OF-pitch-calibration.png
       :target: ../_images/OF-pitch-calibration.png
   
#. If ``OF.flowY`` is larger or smaller than ``OF.bodyY``, then it can
   be adjusted by changing the ``FLOW_FYSCALER`` parameter
#. IF ``OF.bodyY`` is uncorrelated or opposite sign to ``IMU.GyrY``,
   the ``FLOW_ORIENT_YAW`` parameter is probably set incorrectly or
   you do not have the flow sensor pointing downwards

Range Sensor Check
==================

Check the EKF5.meaRng message in the flashlog from your flow
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
distance of over 2m you will need to restart the flight controller).

The error message when arming fails this check is "PreArm: check range
finder"

This check can be disabled by unchecking the "Parameter/Sonar" arming
check.

First Flight (Copter Only)
==========================

#. Perform a short test flight  hovering in stabilise at small tilt
   angles at heights ranging from 50cm to 3m with ``EKF_GPS_TYPE = 0``
   (we don't want the optical flow being used by the EKF at this stage)
#. Download the flash log and plot the following in mission planner
#. EKF5.meaRng should correlate with the change in vehicle height
#. ``OF.flowX`` and ``OF.flowY`` should be varying
#. ``OF.bodyX`` and ``OF.bodyY`` should be consistent with IMU.GyrX and
   IMU.GyrY

Second Flight (Copter only)
===========================

.. warning::

   You will need at least 15m of clear space around the vehicle to
   do this flight safely.

#. Set the EKF_GPS_TYPE parameter to 3 make the EKF ignore GPS and use

   .. warning::

      Do not switch from any non-gps mode, eg
         STABILIZE, ALT_HOLD or ACRO  to a GPS mode, eg LOITER when flying
         with EKF_GPS_TYPE set to 3. If the optical flow velocity estimates
         are bad, you will have no warning and the copter could go to maximum
         roll or pitch when you make the switch. This vulnerability will be
         fixed in subsequent SW releases.

#. Ensure you have LOITER mode and STABILIZE mode available on
   you transmitter.
#. Take-off in loiter and bring the copter to about 1m height

   .. warning::

      Do not take-off in STABILIZE and switch
         to LOITER

#. If it starts to accelerate away or there is erratic pitch or roll
   movement, then switch to stabilise and land. You will need to
   download the log file and share it on the forum to understand why.
#. If it holds location then congratulations, you have succeeded and can
   now start experimenting with height changes and moving it around in
   LOITER

