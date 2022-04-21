.. _common-gps-for-yaw:

=================================
GPS for Yaw (aka Moving Baseline)
=================================

Two UBlox F9 GPS modules can be used to estimate yaw which removes the
need for a compass which may suffer from magnetic interference from
the ground or the vehicle's motors and ESCs.  This works even if the
GPSs do not have RTK fix.

GPSs from ArduPilot Partners that are known to work include:

- :ref:`common-gps-ardusimple`
- :ref:`common-cuav-c-rtk-9p-gps`
- `CUAV C-RTK 9Ps GPS <https://store.cuav.net/shop/c-rtk-9ps/>`__
- :ref:`Holybro H-RTK F9P GNSS <common-holybro-rtk-f9p>`
- `mRobotics ZED-F9 GPS <https://store.mrobotics.io/category-s/109.htm>`__
- :ref:`common-synerex-mdu-2000`
- :ref:`common-piksi-multi-rtk-receiver`
- :ref:`common-gps-septentrio`

.. note::

   This feature is available in Copter 4.0.4 (and higher), Plane 4.0.6 (and higher) and Rover 4.1.0 (and higher); **Ublox F9p must run firmware version 1.13 or higher and constellations configured**. See :ref:`common-gps-ublox-firmware-update`.


Hardware Setup
--------------

- Two Ublox F9 GPSs should be place on the vehicle at least 30cm apart (horizontally)
- The 2nd GPS should be connected to a serial/telem port on the
  autopilot.  These instructions assume Serial4/Telem4 is used but any
  serial port should work
- Serial GPS modules must be connected to ArduPilot via their UART1 connectors, DroneCAN modules via CAN, or interconnected per their manufacturer instructions.

Configuration
-------------

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 5 ("GPS") assuming the 2nd GPS is connected to serial port 4
- :ref:`GPS_TYPE <GPS_TYPE>` = 17 ("UBlox moving baseline base") or 22 (DroneCAN-MovingBaseline-Base), as appropriate.
- :ref:`GPS_TYPE2 <GPS_TYPE2>` = 18 ("UBlox moving baseline rover") or 23 (DroneCAN-MovingBaseline-Rover), as appropriate.
- Set the :ref:`GPS_POS1_X <GPS_POS1_X>`/Y/Z and :ref:`GPS_POS2_X <GPS_POS2_X>`/Y/Z parameters for the GPSs (see :ref:`Sensor Position Offset are here <common-sensor-offset-compensation>`)
- :ref:`GPS_AUTO_SWITCH <GPS_AUTO_SWITCH>` = 1
- :ref:`AHRS_EKF_TYPE <AHRS_EKF_TYPE>` = 3 (to use EKF3)
- :ref:`EK2_ENABLE <EK2_ENABLE>` = 0 (to disable EKF2)
- :ref:`EK3_ENABLE <EK3_ENABLE>` = 1 (to enable EKF3)

If using 4.0

- :ref:`EK3_MAG_CAL <EK3_MAG_CAL>` = 5 ("Use external yaw sensor")

If using 4.1 (or higher)

- :ref:`EK3_MAG_CAL <EK3_MAG_CAL>` is not used for this feature so it can be left at its default value ("0" for Plane, "3" for Copter, "2" for Rover)
- :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 2 ("GPS") or 3 ("GPS with Compass Fallback")

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

Video Demo
----------

.. youtube:: NjaIKyrInpg


Using Moving Baseline Yaw to Reject Magnetic Disturbances
---------------------------------------------------------

.. youtube:: MmnfHUYLTeQ



