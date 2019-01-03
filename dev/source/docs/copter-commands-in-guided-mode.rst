.. _copter-commands-in-guided-mode:

==============================
Copter Commands in Guided Mode
==============================

This article lists the MAVLink commands that Copter accepts.  Normally these commands are sent by a ground station or :ref:`Companion Computers <companion-computers>` often running `DroneKit <http://dronekit.io/>`__.

.. note::

   The Copter code which processes these commands can be found `here in GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/GCS_Mavlink.cpp#L676>`__

Movement commands
=================

These commands can be used to control the vehicle's position, velocity or attitude while in Guided Mode

- :ref:`SET_POSITION_TARGET_LOCAL_NED <copter-commands-in-guided-mode_set_position_target_local_ned>`
- :ref:`SET_POSITION_TARGET_GLOBAL_INT <copter-commands-in-guided-mode_set_position_target_global_int>`
- :ref:`SET_ATTITUDE_TARGET <copter-commands-in-guided-mode_set_attitude_target>` (supported in Guided and Guided_NoGPS modes)

MAV_CMDs
=========

These MAV_CMDs can be processed if packaged within a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ message.

- :ref:`MAV_CMD_COMPONENT_ARM_DISARM <copter:mav_cmd_component_arm_disarm>`
- :ref:`MAV_CMD_CONDITION_YAW <copter:mav_cmd_condition_yaw>`
- :ref:`MAV_CMD_DO_CHANGE_SPEED <copter:mav_cmd_do_change_speed>`
- :ref:`MAV_CMD_DO_DIGICAM_CONFIGURE <copter:mav_cmd_do_digicam_configure>`
- :ref:`MAV_CMD_DO_DIGICAM_CONTROL <copter:mav_cmd_do_digicam_control>`
- :ref:`MAV_CMD_DO_FENCE_ENABLE <copter:mav_cmd_do_fence_enable>`
- `MAV_CMD_DO_FLIGHTTERMINATION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION>`__ - disarms motors immediately (Copter falls!).
- :ref:`MAV_CMD_DO_GRIPPER <copter:mav_cmd_do_gripper>`
- `MAV_CMD_DO_MOTOR_TEST <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_MOTOR_TEST>`__
- :ref:`MAV_CMD_DO_PARACHUTE <copter:mav_cmd_do_parachute>`
- :ref:`MAV_CMD_DO_REPEAT_SERVO <copter:mav_cmd_do_repeat_servo>`
- :ref:`MAV_CMD_DO_REPEAT_RELAY <copter:mav_cmd_do_repeat_relay>`
- `MAV_CMD_DO_SEND_BANNER <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SEND_BANNER>`__
- :ref:`MAV_CMD_DO_SET_HOME <copter:mav_cmd_do_set_home>`
- :ref:`MAV_CMD_DO_SET_RELAY <copter:mav_cmd_do_set_relay>`
- :ref:`MAV_CMD_DO_SET_ROI <copter:mav_cmd_do_set_roi>`
- :ref:`MAV_CMD_DO_SET_SERVO <copter:mav_cmd_do_set_servo>`
- `MAV_CMD_DO_MOUNT_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE>`__
- :ref:`MAV_CMD_DO_MOUNT_CONTROL <copter:mav_cmd_do_mount_control>`
- `MAV_CMD_GET_HOME_POSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_GET_HOME_POSITION>`__
- :ref:`MAV_CMD_MISSION_START <copter:mav_cmd_mission_start>`
- :ref:`MAV_CMD_NAV_TAKEOFF <copter:mav_cmd_nav_takeoff>`
- :ref:`MAV_CMD_NAV_LOITER_UNLIM <copter:mav_cmd_nav_loiter_unlim>`
- :ref:`MAV_CMD_NAV_RETURN_TO_LAUNCH <copter:mav_cmd_nav_return_to_launch>`
- :ref:`MAV_CMD_NAV_LAND <copter:mav_cmd_nav_land>`
- `MAV_CMD_PREFLIGHT_CALIBRATION <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_CALIBRATION>`__
- `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS>`__
- `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN>`__
- `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES>`__
- `MAV_CMD_START_RX_PAIR <https://mavlink.io/en/messages/common.html#MAV_CMD_START_RX_PAIR>`__ - starts receiver pairing
- `MAV_CMD_DO_START_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_START_MAG_CAL>`__
- `MAV_CMD_DO_ACCEPT_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_ACCEPT_MAG_CAL>`__
- `MAV_CMD_DO_CANCEL_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_CANCEL_MAG_CAL>`__

Other commands
==============

Below are other commands that will be handled by Copter

- `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`__
- `AUTOPILOT_VERSION_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#AUTOPILOT_VERSION_REQUEST>`__
- `COMMAND_ACK <https://mavlink.io/en/messages/common.html#COMMAND_ACK>`__
- `GIMBAL_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_REPORT>`__
- `GPS_INJECT_DATA <https://mavlink.io/en/messages/common.html#GPS_INJECT_DATA>`__
- `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`__
- `HIL_STATE <https://mavlink.io/en/messages/common.html#HIL_STATE>`__
- `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`__
- `LED_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#LED_CONTROL>`__
- `LOG_ERASE <https://mavlink.io/en/messages/common.html#LOG_ERASE>`__
- `LOG_REQUEST_DATA <https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA>`__
- `LOG_REQUEST_END <https://mavlink.io/en/messages/common.html#LOG_REQUEST_END>`__
- `LOG_REQUEST_LIST <https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST>`__
- `MISSION_CLEAR_ALL <https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL>`__
- `MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`__
- `MISSION_ITEM <https://mavlink.io/en/messages/common.html#MISSION_ITEM>`__
- `MISSION_REQUEST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST>`__
- `MISSION_REQUEST_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST>`__
- `MISSION_SET_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT>`__
- `MISSION_WRITE_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST>`__
- `PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`__
- `PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`__
- `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`__
- `RADIO <https://mavlink.io/en/messages/ardupilotmega.html#RADIO>`__
- `RADIO_STATUS <https://mavlink.io/en/messages/common.html#RADIO_STATUS>`__
- `RALLY_FETCH_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_FETCH_POINT>`__
- `RALLY_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_POINT>`__
- `RC_CHANNELS_OVERRIDE <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`__
- `REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`__
- `REMOTE_LOG_BLOCK_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_BLOCK_STATUS>`__
- `SERIAL_CONTROL <https://mavlink.io/en/messages/common.html#SERIAL_CONTROL>`__
- :ref:`SET_HOME_POSITION <copter-commands-in-guided-mode_set_home_position>`
- `SET_MODE <https://mavlink.io/en/messages/common.html#SET_MODE>`__
- `TERRAIN_DATA <https://mavlink.io/en/messages/common.html#TERRAIN_DATA>`__
- `TERRAIN_CHECK <https://mavlink.io/en/messages/common.html#TERRAIN_CHECK>`__

Movement Command Details
========================

This section contains details of MAVLink commands to move the vehicle

.. _copter-commands-in-guided-mode_set_position_target_local_ned:

SET_POSITION_TARGET_LOCAL_NED
-----------------------------

Set the vehicle's target position, velocity, heading or turn rate.  The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>time_boot_ms</strong></td>
   <td>
   Sender's system time in milliseconds since boot
   </td>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>System ID of vehicle</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>Component ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>coordinate_frame</strong></td>
   <td>Valid options are listed below</td>
   </tr>
   <tr>
   <td><strong>type_mask</strong></td>
   <td>

Bitmask to indicate which fields should be **ignored** by the vehicle (see POSITION_TARGET_TYPEMASK enum)

bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

When providing Pos or Vel all 3 axis must be provided

- Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
- Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
- Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)

Acceleration not supported
   
.. raw:: html
   
   </td>
   </tr>
   <tr>
   <td><strong>x</strong></td>
   <td>X Position in meters (positive is forward or North)</td>
   </tr>
   <tr>
   <td><strong>y</strong></td>
   <td>Y Position in meters (positive is right or East)</td>
   </tr>
   <tr>
   <td><strong>z</strong></td>
   <td>Z Position in meters (positive is down)</td>
   </tr>
   <tr>
   <td><strong>vx</strong></td>
   <td>X velocity in m/s (positive is forward or North)</td>
   </tr>
   <tr>
   <td><strong>vy</strong></td>
   <td>Y velocity in m/s (positive is right or East)</td>
   </tr>
   <tr>
   <td><strong>vz</strong></td>
   <td>Z velocity in m/s (positive is down)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afx</td>
   <td>X acceleration not supported</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afy</td>
   <td>Y acceleration not supported</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afz</td>
   <td>Z acceleration not supported</td>
   </tr>
   <tr>
   <td><strong>yaw</strong></td>
   <td>yaw or heading in radians (0 is forward or North)</td>
   </tr>
   <tr>
   <td><strong>yaw_rate</strong></td>
   <td>yaw rate in rad/s</td>
   </tr>
   </tbody>
   </table>

The ``co-ordinate frame`` field takes the following values:

+--------------------------------------+--------------------------------------+
| Frame                                | Description                          |
+======================================+======================================+
| ``MAV_FRAME_LOCAL_NED``              | Positions are relative to the        |
|                                      | vehicle's EKF Origin in NED frame    |
|                                      |                                      |
|                                      | I.e x=1,y=2,z=3 is 1m North, 2m East |
|                                      | and 3m Down from the origin          |
|                                      |                                      |
|                                      | The **EKF origin** is the vehicle's  |
|                                      | location when it first achieved a    |
|                                      | good position estimate               |
|                                      |                                      |
|                                      | Velocity are in NED frame            |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_LOCAL_OFFSET_NED``       | Positions are relative to the        |
|                                      | vehicle's current position           |
|                                      |                                      |
|                                      | I.e. x=1,y=2,z=3 is 1m North,        |
|                                      | 2m East and 3m below the current     |
|                                      | position.                            |
|                                      |                                      |
|                                      | Velocity are in NED frame.           |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_BODY_OFFSET_NED``        | Positions are relative to the        |
|                                      | vehicle's current position and       |
|                                      | heading                              |
|                                      |                                      |
|                                      | I.e x=1,y=2,z=3 is 1m forward,       |
|                                      | 2m right and 3m Down from the current|
|                                      | position                             |
|                                      |                                      |
|                                      | Velocities are relative to           |
|                                      | the current vehicle heading. Use     |
|                                      | this to specify the speed forward,   |
|                                      | right and down (or the opposite if   |
|                                      | you use negative values).            |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_BODY_NED``               | Positions are relative to the        |
|                                      | EKF Origin in NED frame              |
|                                      |                                      |
|                                      | I.e x=1,y=2,z=3 is 1m North, 2m East |
|                                      | and 3m Down from the origin          |
|                                      |                                      |
|                                      | Velocities are relative to           |
|                                      | the current vehicle heading. Use     |
|                                      | this to specify the speed forward,   |
|                                      | right and down (or the opposite if   |
|                                      | you use negative values).            |
+--------------------------------------+--------------------------------------+

.. tip::

   In frames, ``_OFFSET_`` means "relative to vehicle position" while ``_LOCAL_`` is "relative to home position" (these have no impact on *velocity* directions). ``_BODY_`` means that velocity components are relative to the heading of the vehicle rather than the NED frame.

.. note::

   If sending velocity commands, they should be resent every second (the vehicle will stop after 3 seconds if no command is received)
   
.. _copter-commands-in-guided-mode_set_position_target_global_int:

SET_POSITION_TARGET_GLOBAL_INT
------------------------------

Set the vehicle's target position (in WGS84 coordinates), velocity, heading or turn rate.  This is similar to the SET_POSITION_TARGET_LOCAL_NED message (see above) except positions are provided as latitude and longitude values and altitudes can be above sea-level, relative to home or relative to terrain.

The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`__

**Command parameters**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>time_boot_ms</strong></td>
   <td>
   Sender's system time in milliseconds since boot
   </td>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>System ID of vehicle</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>Component ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>coordinate_frame</strong></td>
   <td>

Valid options are:

- MAV_FRAME_GLOBAL_INT : alt is meters above sea level
- MAV_FRAME_GLOBAL_RELATIVE_ALT: alt is meters above home
- MAV_FRAME_GLOBAL_RELATIVE_ALT_INT: alt is meters above home
- MAV_FRAME_GLOBAL_RELATIVE_TERRAIN_ALT: alt is meters above terrain
- MAV_FRAME_GLOBAL_RELATIVE_TERRAIN_ALT_INT: alt is meters above terrain

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>type_mask</strong></td>
   <td>

Bitmask to indicate which fields should be **ignored** by the vehicle (see POSITION_TARGET_TYPEMASK enum)

bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

When providing Pos or Vel all 3 axis must be provided

- Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
- Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
- Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)

Acceleration not supported

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>lat_int</strong></td>
   <td>Latitude * 1e7</td>
   </tr>
   <tr>
   <td><strong>lon_int</strong></td>
   <td>Longitude * 1e7</td>
   </tr>
   <tr>
   <td><strong>alt</strong></td>
   <td>Alt in meters above sea level, home or terrain (see coordinate_frame field)</td>
   </tr>
   <tr>
   <td><strong>vx</strong></td>
   <td>X velocity in m/s (positive is North)</td>
   </tr>
   <tr>
   <td><strong>vy</strong></td>
   <td>Y velocity in m/s (positive is East)</td>
   </tr>
   <tr>
   <td><strong>vz</strong></td>
   <td>Z velocity in m/s (positive is down)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afx</td>
   <td>X acceleration not supported
   </td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afy</td>
   <td>Y acceleration not supported</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afz</td>
   <td>Z acceleration not supported</td>
   </tr>
   <tr>
   <td><strong>yaw</strong></td>
   <td>yaw or heading in radians (0 is forward or North)</td>
   </tr>
   <tr>
   <td><strong>yaw_rate</strong></td>
   <td>yaw rate in rad/s</td>
   </tr>
   </tbody>
   </table>

.. note::

   Velocity commands should be resent every second (the vehicle will stop after 3 seconds if no command is received)

.. _copter-commands-in-guided-mode_set_attitude_target:

SET_ATTITUDE_TARGET
-------------------

Set the vehicle's target attitude and climb rate.  This message is accepted in :ref:`Guided <copter:ac2_guidedmode>` or Guided_NoGPS (this is the only message accepted by Guided_NoGPS).  The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`__

**Command parameters**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>time_boot_ms</strong></td>
   <td>uint32_t</td>
   <td>Sender's system time in milliseconds since boot</td>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>uint8_t</td>
   <td>System ID of vehicle</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>int8_t</td>
   <td>Component ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>type_mask</strong></td>
   <td>int8_t</td>
   <td>

Bitmask to indicate which fields should be **ignored** by the vehicle

bit1:body roll rate, bit2:body pitch rate, bit3:body yaw rate, bit7:throttle, bit8:attitude

Should always be 0b00000111 / 0x07 / 7 (decimal)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>q</strong></td>
   <td>float[4]</td>
   <td>
   Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})
   <br>
   Note that zero-rotation causes vehicle to rotate towards North.
   </td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>body_roll_rate</strong></td>
   <td>float</td>
   <td>Body roll rate not supported</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>body_pitch_rate</strong></td>
   <td>float</td>
   <td>Body pitch rate not supported</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>body_yaw_rate</strong></td>
   <td>float</td>
   <td>Body yaw rate not supported</td>
   </tr>
   <tr>
   <td><strong>thrust</strong></td>
   <td>float</td>
   <td>Climb rate where 0.5 = no climb, 0 = WPNAV_SPEED_DN, 1 = WPNAV_SPEED_UP
   </td>
   </tr>
   </tbody>
   </table>

.. _copter-commands-in-guided-mode_set_home_position:

SET_HOME_POSITION
-------------------

The position the system will return to and land on. The position is set
automatically by the system during the takeoff if it has not been
explicitly set by the operator before or after.

The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_HOME_POSITION>`__

**Command parameters**

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>uint8_t</td>
   <td>System ID</td>
   </tr>
   <tr>
   <td><strong>latitude</strong></td>
   <td>int32_t</td>
   <td>Latitude * 1e7</td>
   </tr>
   <tr>
   <td><strong>longitude</strong></td>
   <td>int32_t</td>
   <td>Longitude * 1e7</td>
   </tr>
   <tr>
   <td><strong>altitude</strong></td>
   <td>int32_t</td>
   <td>Altitude above sea level in millimeters (i.e. meters * 1000)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>x</strong></td>
   <td>float</td>
   <td>Local X position of this position in the local coordinate frame.</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>y</strong></td>
   <td>float</td>
   <td>Local Y position of this position in the local coordinate frame</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>z</strong></td>
   <td>float</td>
   <td>Local Z position of this position in the local coordinate frame</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>q</strong></td>
   <td>float[4]</td>
   <td>World to surface normal and heading transformation of the takeoff
   position. Used to indicate the heading and slope of the ground.
   </td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>approach_x</strong></td>
   <td>float</td>
   <td>
   Local X position of the end of the approach vector. Multicopters should
   set this position based on their takeoff path. Grass-landing fixed wing
   aircraft should set it the same way as multicopters. Runway-landing
   fixed wing aircraft should set it to the opposite direction of the
   takeoff, assuming the takeoff happened from the threshold / touchdown
   zone.
   </td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>approach_y</strong></td>
   <td>float</td>
   <td>
   Local Y position of the end of the approach vector. Multicopters should
   set this position based on their takeoff path. Grass-landing fixed wing
   aircraft should set it the same way as multicopters. Runway-landing
   fixed wing aircraft should set it to the opposite direction of the
   takeoff, assuming the takeoff happened from the threshold / touchdown
   zone.

   </td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>approach_z</strong></td>
   <td>float</td>
   <td>
   Local Z position of the end of the approach vector. Multicopters should
   set this position based on their takeoff path. Grass-landing fixed wing
   aircraft should set it the same way as multicopters. Runway-landing
   fixed wing aircraft should set it to the opposite direction of the
   takeoff, assuming the takeoff happened from the threshold / touchdown zone.
   </td>
   </tr>
   </tbody>
   </table>
