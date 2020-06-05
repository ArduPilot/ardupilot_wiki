.. _common-gps-for-yaw:

=================================
GPS for Yaw (aka Moving Baseline)
=================================

Two UBlox M9 or F9 GPSs can be used to estimate yaw which removes the need for a compass which may suffer from magnetic interference from the ground or the vehicle's motors and ESCs.  This works even if the GPSs do not have RTK fix.

GPSs from ArduPilot Partners that are known to work include:

- `CUAV C-RTK 9P <https://store.cuav.net/index.php?id_product=101&rewrite=c-rtk-9p&controller=product#/32-rtk-sky_end>`__
-  `mRobotics Location One and ZED-f9 GPSs <https://store.mrobotics.io/category-s/109.htm>`__

.. note::

   This feature is available in Copter 4.0.4 (and higher), Plane 4.0.6 (and higher) and Rover 4.1.0 (and higher)

Hardware Setup
--------------

- Two Ublox M9 or F9 GPSs should be place on the vehicle at least 30cm apart (horizontally)
- The 2nd GPS should be connected to a serial/telem port on the autopilot.  These instructions assume Serial4/Telem4 is used but any serial port should work

Configuration
-------------

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 5 ("GPS") assuming the 2nd GPS is connected to serial port 4
- :ref:`GPS_TYPE2 <GPS_TYPE2>` = 2 ("UBlox")
- Set the :ref:`GPS_POS1_X <GPS_POS1_X>`/Y/Z and :ref:`GPS_POS2_X <GPS_POS2_X>`/Y/Z parameters for the GPSs (see :ref:`Sensor Position Offset are here <common-sensor-offset-compensation>`)
- :ref:`AHRS_EKF_TYPE <AHRS_EKF_TYPE>` = 3 (to use EKF3)
- :ref:`EK2_ENABLE <EK2_ENABLE>` = 0 (to disable EKF2)
- :ref:`EK3_ENABLE <EK3_ENABLE>` = 1 (to enable EKF3)
- :ref:`EK3_MAG_CAL <EK3_MAG_CAL>` = 5 ("Use external yaw sensor") or 6 ("External yaw sensor with compass fallback")

You may optionally wish to enable :ref:`GPS Blending <common-gps-blending>`

Testing
-------

In a location with good GPS reception point the vehicle at a landmark some distance away and then check the heading on the ground station matches.  Rotate the vehicle and ensure the heading on the ground station updates correctly.

If the heading is reversed, then the GPS_POS_xxx parameters have probably been set incorrectly.
