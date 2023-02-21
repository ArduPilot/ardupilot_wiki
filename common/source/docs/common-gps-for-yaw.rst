.. _common-gps-for-yaw:

=================================
GPS for Yaw (aka Moving Baseline)
=================================

Two UBlox F9 GPS modules can be used to estimate yaw which removes the
need for a compass which may suffer from magnetic interference from
the ground or the vehicle's motors and ESCs.  This works even if the
GPSs do not have RTK fix (RTCM data from a fixed RTK station or NTRIP server).

Also, recently, single unit GPSes which utilize Internal Moving Baseline (see below) are becoming available, like the Blicube RTK GPS.

GPSes from ArduPilot Partners that are known to work are shown on the :ref:`common-positioning-landing-page`

.. note::

 **Ublox F9P modules must run firmware version 1.13 or higher and constellations configured**. See :ref:`common-gps-ublox-firmware-update`.


Hardware Setup
==============

- Two Ublox F9 GPSs should be placed on the vehicle at least 30cm apart (horizontally)
- The 1st GPS and 2nd GPS should be connected to a serial/telem ports on the
  autopilot.  These instructions assume Serial3 and Serial4 are used but any
  serial ports should work as long as the first port using protocol 5 is connected to one of the GPS.
- Serial GPS modules must be connected to ArduPilot via their UART1 connectors, DroneCAN modules via CAN (if the DroneCAN modules support Moving Baseline), or interconnected per their manufacturer instructions.

Configuration
=============

Serial GPS
----------

- :ref:`SERIAL3_PROTOCOL<SERIAL3_PROTOCOL>` = 5 ("GPS") assuming the 1st GPS is connected to SERIAL3.
- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 5 ("GPS") assuming the 2nd GPS is connected to serial port 4
- :ref:`GPS_TYPE <GPS_TYPE>` = 17 ("UBlox moving baseline base") 
- :ref:`GPS_TYPE2 <GPS_TYPE2>` = 18 ("UBlox moving baseline rover")
- :ref:`GPS_AUTO_CONFIG<GPS_AUTO_CONFIG>` = 1 (AutoConfig Serial)
- Set the :ref:`GPS_POS1_X <GPS_POS1_X>`/Y/Z and :ref:`GPS_POS2_X <GPS_POS2_X>`/Y/Z parameters for the GPSs (see :ref:`Sensor Position Offset are here <common-sensor-offset-compensation>`). You must establish the relative positions of each GPS location on the vehicle with respect the vehicle's motion.

DroneCAN GPS
------------

If DroneCAN GPS are used, then configure the CAN/DroneCAN ports as explained in :ref:`common-uavcan-setup-advanced` and instead of setting up the SERIAL port protocols above, make sure that no SERIAL ports are setup with GPS protocol ("5"). Also be sure that the two DroneCAN GPS are on the same physical CAN bus from the autopilot. This usually requires that a CAN bus splitter be used. Then set these parameters:

- :ref:`GPS_TYPE <GPS_TYPE>` = 22 ("DroneCAN moving baseline base")
- :ref:`GPS_TYPE2 <GPS_TYPE2>` = 23 ("DroneCAN moving baseline rover")
- :ref:`GPS_AUTO_CONFIG<GPS_AUTO_CONFIG>` = 2 (AutoConfig DroneCAN)
- :ref:`GPS1_CAN_OVRIDE<GPS1_CAN_OVRIDE>` (Base NODEID) and :ref:`GPS2_CAN_OVRIDE<GPS2_CAN_OVRIDE>` (Rover NODEID) determine which physical DroneCAN GPS is used for GPS1 and GPS2. These are automatically populated at boot from the detected addresses, which are also shown in :ref:`GPS_CAN_NODEID1<GPS_CAN_NODEID1>` and :ref:`GPS_CAN_NODEID2<GPS_CAN_NODEID2>`, but can be overriden, if needed. You will need to determine which physical CAN GPS is assigned as GPS1 and GSP2 in order to setup the position offsets (see :ref:`Sensor Position Offset are here <common-sensor-offset-compensation>`)

For either Serial or DroneCAN GPS also set:

- Set the :ref:`GPS_POS1_X <GPS_POS1_X>`/Y/Z and :ref:`GPS_POS2_X <GPS_POS2_X>`/Y/Z parameters for the GPSs (see :ref:`Sensor Position Offset are here <common-sensor-offset-compensation>`). You must establish the relative positions of each GPS location on the vehicle with respect the vehicle's motion.
- :ref:`GPS_AUTO_SWITCH <GPS_AUTO_SWITCH>` = 1
- :ref:`AHRS_EKF_TYPE <AHRS_EKF_TYPE>` = 3 (to use EKF3)
- :ref:`EK2_ENABLE <EK2_ENABLE>` = 0 (to disable EKF2)
- :ref:`EK3_ENABLE <EK3_ENABLE>` = 1 (to enable EKF3)
- :ref:`EK3_MAG_CAL <EK3_MAG_CAL>` is not used for this feature so it can be left at its default value ("0" for Plane, "3" for Copter, "2" for Rover)
- :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 2 ("GPS") or 3 ("GPS with Compass Fallback") if a compass(es) is also in the system

The above configurations assumes that you want the RTCMv3 data between
the two GPS modules to go via the autopilot board.

.. note:: You may instead install a cross-over UART cable between the two UART2 connectors on the two GPS modules (Assuming they are serial rather than DroneCAN). If you do that then you can set :ref:`GPS_DRV_OPTIONS <GPS_DRV_OPTIONS>` = 1 which tells the u-blox GPS driver to configure the two GPS modules to send RTCMv2 data over UART2. On DroneCAN modules, a second CAN port is provided to allow this direct connection for RTCMv2 data. In that case, set :ref:`GPS_DRV_OPTIONS <GPS_DRV_OPTIONS>` = 8.

.. note:: should be possible to mix and match a Serial GPS and a DroneCAN GPS, but this configuration has not been tested as yet.

.. note:: Do not use :ref:`GPS_AUTO_SWITCH<GPS_AUTO_SWITCH>` = 2 (Blend) when using Moving Baseline configurations.

Internal Moving Baseline Systems
================================

Some vehicle GPS provide GPS for Yaw utilizing a completely internal dual gps unit and managing the inter gps communication totally internally, rather than having ArduPilot pass data between the GPSes. An example of this system is the `Blicube GRTK <https://wiki.blicube.com/grtk/>`__.

This only requires that the GPS be attached to a SERIALx port using ``SERIALx_PROTOCOL`` = 5 (GPS)
 and setting only:

- :ref:`GPS_TYPE<GPS_TYPE>` = 5 (NMEA)
- :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 2 ("GPS") or 3 ("GPS with Compass Fallback") if a compass(es) is also in the system.


Testing
=======

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



