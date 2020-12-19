.. _common-gps-for-yaw:

=================================
GPS for Yaw (aka Moving Baseline)
=================================

Two UBlox F9 GPS modules can be used to estimate yaw which removes the
need for a compass which may suffer from magnetic interference from
the ground or the vehicle's motors and ESCs.  This works even if the
GPSs do not have RTK fix.

GPSs from ArduPilot Partners that are known to work include:

- :ref:`CUAV C-RTK 9P <common-cuav-c-rtk-9p-gps>`
- :ref:`Holybro H-RTK F9P GNSS <common-holybro-rtk-f9p>`
- `mRobotics ZED-F9 GPS <https://store.mrobotics.io/category-s/109.htm>`__

.. note::

   This feature is available in Copter 4.0.4 (and higher), Plane 4.0.6 (and higher) and Rover 4.1.0 (and higher); **Ublox F9p must run firmware version 1.13 or higher and constellations configured**. See :ref:`common-gps-ublox-firmware-update`.


Hardware Setup
--------------

- Two Ublox F9 GPSs should be place on the vehicle at least 30cm apart (horizontally)
- The 2nd GPS should be connected to a serial/telem port on the
  autopilot.  These instructions assume Serial4/Telem4 is used but any
  serial port should work
- both GPS modules must be connected to ArduPilot via their UART1 connectors

Configuration
-------------

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 5 ("GPS") assuming the 2nd GPS is connected to serial port 4
- :ref:`GPS_TYPE <GPS_TYPE>` = 17 ("UBlox moving baseline base")
- :ref:`GPS_TYPE2 <GPS_TYPE2>` = 18 ("UBlox moving baseline rover")
- Set the :ref:`GPS_POS1_X <GPS_POS1_X>`/Y/Z and :ref:`GPS_POS2_X <GPS_POS2_X>`/Y/Z parameters for the GPSs (see :ref:`Sensor Position Offset are here <common-sensor-offset-compensation>`)
- :ref:`AHRS_EKF_TYPE <AHRS_EKF_TYPE>` = 3 (to use EKF3)
- :ref:`EK2_ENABLE <EK2_ENABLE>` = 0 (to disable EKF2)
- :ref:`EK3_ENABLE <EK3_ENABLE>` = 1 (to enable EKF3)
- :ref:`EK3_MAG_CAL <EK3_MAG_CAL>` = 5 ("Use external yaw sensor") or 6 ("External yaw sensor with compass fallback")
- :ref:`GPS_AUTO_SWITCH <GPS_AUTO_SWITCH>` = 1

The above configuration assumes that you want the RTCMv3 data between
the two GPS modules to go via the flight controller board. You may instead
install a cross-over UART cable between the two UART2 connectors on
the two GPS modules. If you do that then you can set
:ref:`GPS_DRV_OPTIONS <GPS_DRV_OPTIONS>` = 1 which tells the u-blox
GPS driver to configure the two GPS modules to send RTCMv2 data
over UART2.

Testing
-------

In a location with good GPS reception point the vehicle at a landmark
some distance away and then check the heading on the ground station
matches.  Rotate the vehicle and ensure the heading on the ground
station updates correctly.

If the heading is reversed, then the GPS_POS_xxx parameters have probably been set incorrectly.

Note that it can take some time for the two GPS modules to get a
sufficiently good fix for yaw to work. The ArduPilot GPS driver
validates that the fix is good enough in several ways:

 - that the rover GPS module is in fix type 6 (fixed RTK)
 - that the reported distance between the two modules matches the
   distance given by the GPS position parameters within 20%
 - that the reported heights of the two GPS modules match the attitude
   of the vehicles is within 20% of the distance between the two GPS
   modules

Firmware Versions
-----------------

The 4.0 ArduPilot firmware versions don't support the EK3_MAG_CAL=6
choice. That choice will be in the 4.1.x releases. Using EK3_MAG_CAL=6
allows the EKF to learn a set of compass offsets while flying which
allows your compass to be a backup for if your GPS yaw fails in
flight.

Video Demo
----------

.. youtube:: NjaIKyrInpg

