.. _copter-commands-in-guided-mode:

==============================
Copter Commands in Guided Mode
==============================

This article lists the MAVLink commands that affect the movement of a Copter.  Normally these commands are sent by a ground station or :ref:`Companion Computers <companion-computers>` often running `DroneKit <http://dronekit.io/>`__.

.. note::

   The Copter code which processes these commands can be found `here in GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/GCS_Mavlink.cpp#L683>`__

Movement Commands
=================

These commands can be used to control the vehicle's position, velocity or attitude while in :ref:`Guided Mode <copter:ac2_guidedmode>`

- :ref:`SET_POSITION_TARGET_LOCAL_NED <copter-commands-in-guided-mode_set_position_target_local_ned>`
- :ref:`SET_POSITION_TARGET_GLOBAL_INT <copter-commands-in-guided-mode_set_position_target_global_int>`
- :ref:`SET_ATTITUDE_TARGET <copter-commands-in-guided-mode_set_attitude_target>` (supported in Guided and Guided_NoGPS modes)

MAV_CMDs
=========

These MAV_CMDs can be processed if packaged within a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ message.

- :ref:`MAV_CMD_CONDITION_YAW <copter:mav_cmd_condition_yaw>`
- :ref:`MAV_CMD_DO_CHANGE_SPEED <copter:mav_cmd_do_change_speed>`
- `MAV_CMD_DO_FLIGHTTERMINATION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION>`__ - disarms motors immediately (Copter falls!).
- :ref:`MAV_CMD_DO_PARACHUTE <copter:mav_cmd_do_parachute>`
- :ref:`MAV_CMD_DO_SET_ROI <copter:mav_cmd_do_set_roi>`
- :ref:`MAV_CMD_NAV_TAKEOFF <copter:mav_cmd_nav_takeoff>`
- :ref:`MAV_CMD_NAV_LOITER_UNLIM <copter:mav_cmd_nav_loiter_unlim>`
- :ref:`MAV_CMD_NAV_RETURN_TO_LAUNCH <copter:mav_cmd_nav_return_to_launch>`
- :ref:`MAV_CMD_NAV_LAND <copter:mav_cmd_nav_land>`
- `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN>`__

These MAV_CMDs can be processed if packaged within a `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`__ message

- `MAV_CMD_DO_REPOSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION>`__

Movement Command Details
========================

This section contains details of MAVLink commands to move the vehicle

.. _copter-commands-in-guided-mode_set_position_target_local_ned:

SET_POSITION_TARGET_LOCAL_NED
-----------------------------

Set the vehicle's target position (as an offset in NED from the EKF origin), velocity, acceleration, heading or turn rate.  The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__

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

When providing Pos, Vel and/or Accel all 3 axis must be provided.  At least one of Pos, Vel and Accel must be provided (e.g. providing Yaw or YawRate alone is not supported)

- Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
- Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
- Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
- Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
- Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
- Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
- Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)
   
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
   <tr>
   <td><strong>afx</strong></td>
   <td>X acceleration in m/s/s (positive is forward or North)</td>
   </tr>
   <tr>
   <td><strong>afy</strong></td>
   <td>Y acceleration in m/s/s (positive is right or East)</td>
   </tr>
   <tr>
   <td><strong>afz</strong></td>
   <td>Z acceleration in m/s/s (positive is down)</td>
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

The ``coordinate_frame`` field takes the following values:

+--------------------------------------+--------------------------------------+
| Frame                                | Description                          |
+======================================+======================================+
| ``MAV_FRAME_LOCAL_NED`` (1)          | Positions are relative to the        |
|                                      | vehicle's EKF Origin in NED frame    |
|                                      |                                      |
|                                      | I.e x=1,y=2,z=3 is 1m North, 2m East |
|                                      | and 3m Down from the origin          |
|                                      |                                      |
|                                      | The **EKF origin** is the vehicle's  |
|                                      | location when it first achieved a    |
|                                      | good position estimate               |
|                                      |                                      |
|                                      | Velocity and Acceleration are in     |
|                                      | NED frame                            |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_LOCAL_OFFSET_NED`` (7)   | Positions are relative to the        |
|                                      | vehicle's current position           |
|                                      |                                      |
|                                      | I.e. x=1,y=2,z=3 is 1m North,        |
|                                      | 2m East and 3m below the current     |
|                                      | position.                            |
|                                      |                                      |
|                                      | Velocity and Acceleration are in     |
|                                      | NED frame                            |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_BODY_NED`` (8)           | Positions are relative to the        |
|                                      | EKF Origin in NED frame              |
|                                      |                                      |
|                                      | I.e x=1,y=2,z=3 is 1m North, 2m East |
|                                      | and 3m Down from the origin          |
|                                      |                                      |
|                                      | Velocity and Acceleration are        |
|                                      | relative to the current vehicle      |
|                                      | heading. Use this to specify the     |
|                                      | speed forward, right and down (or the|
|                                      | opposite if you use negative values).|
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_BODY_OFFSET_NED`` (9)    | Positions are relative to the        |
|                                      | vehicle's current position and       |
|                                      | heading                              |
|                                      |                                      |
|                                      | I.e x=1,y=2,z=3 is 1m forward,       |
|                                      | 2m right and 3m Down from the current|
|                                      | position                             |
|                                      |                                      |
|                                      | Velocity and Acceleration are        |
|                                      | relative to the current vehicle      |
|                                      | heading. Use this to specify the     |
|                                      | speed forward, right and down (or the|
|                                      | opposite if you use negative values).|
|                                      |                                      |
|                                      | Specify yaw rate of zero to stop     |
|                                      | vehicle yaw from changing            |
+--------------------------------------+--------------------------------------+

.. tip::

   In frames, ``_OFFSET_`` means "relative to vehicle position" while ``_LOCAL_`` is "relative to home position" (these have no impact on *velocity* directions). ``_BODY_`` means that velocity components are relative to the heading of the vehicle rather than the NED frame.

.. note::

   If sending velocity or acceleration commands, they should be re-sent every second (the vehicle will stop after 3 seconds if no command is received)

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- module load message
- GUIDED
- arm throttle
- takeoff 10

+----------------------------------------------------------------------------------+-----------------------------------------------------+
| Example MAVProxy/SITL Command                                                    | Description                                         |
+==================================================================================+=====================================================+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 3576 100 0 -10 0 0 0 0 0 0 0 0`` | fly to 100m North and 10m *above* of the EKF origin |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 7 3576 10 0 0 0 0 0 0 0 0 0 0``    | fly 10m North of the current position               |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3576 10 0 0 0 0 0 0 0 0 0 0``    | fly 10m forward of the current position             |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 3527 0 0 0 1 0 0 0 0 0 0 0``     | fly North at 1m/s                                   |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3527 0 0 0 1 0 0 0 0 0 0 0``     | fly forward at 1m/s                                 |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 1479 0 0 0 0 1 0 0 0 0 0 0``     | fly right at 1m/s with yaw rate of zero             |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 3135 0 0 0 0 0 0 1 0 0 0 0``     | accelerate North at 1m/s                            |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3135 0 0 0 0 0 0 1 0 0 0 0``     | accelerate forward at 1m/s                          |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 1087 0 0 0 0 0 0 0 1 0 0 0``     | accelerate right at 1m/s with yaw rate of zero      |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 2503 0 0 0 0 0 0 0 0 0 0.7854 0``| turn to North-East (Yaw target + velocity of zero)  |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 2503 0 0 0 0 0 0 0 0 0 0.7854 0``| turn 45deg to right (Yaw target + velocity of zero) |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 1479 0 0 0 0 0 0 0 0 0 0 0.174`` | rotate clock-wise at 10deg/sec (velocity of zero)   |
+----------------------------------------------------------------------------------+-----------------------------------------------------+

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

- MAV_FRAME_GLOBAL (0): alt is in meters above sea level
- MAV_FRAME_GLOBAL_INT (5): alt is in meters above sea level
- MAV_FRAME_GLOBAL_RELATIVE_ALT (3): alt is in meters above home
- MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (6): alt is in meters above home
- MAV_FRAME_GLOBAL_TERRAIN_ALT (10): alt is in meters above terrain
- MAV_FRAME_GLOBAL_TERRAIN_ALT_INT (11): alt is in meters above terrain

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>type_mask</strong></td>
   <td>

Bitmask to indicate which fields should be **ignored** by the vehicle (see POSITION_TARGET_TYPEMASK enum)

bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

When providing Pos, Vel and/or Accel all 3 axis must be provided.  At least one of Pos, Vel and Accel must be provided (e.g. providing Yaw or YawRate alone is not supported)

- Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
- Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
- Use Acceleration : 0b110000111000 / 0x0C38 / 3128 (decimal)
- Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
- Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
- Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
- Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)

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
   <tr>
   <td><strong>afx</strong></td>
   <td>X acceleration in m/s/s (positive is North)</td>
   </td>
   </tr>
   <tr>
   <td><strong>afy</strong></td>
   <td>Y acceleration in m/s/s (positive is East)</td>
   </tr>
   <tr>
   <td><strong>afz</strong></td>
   <td>Z acceleration in m/s/s (positive is Down)</td>
   </tr>
   <tr>
   <td><strong>yaw</strong></td>
   <td>yaw or heading in radians (0 is forward)</td>
   </tr>
   <tr>
   <td><strong>yaw_rate</strong></td>
   <td>yaw rate in rad/s</td>
   </tr>
   </tbody>
   </table>

.. note::

   If sending velocity or acceleration commands, they should be re-sent every second (the vehicle will stop after 3 seconds if no command is received)

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- module load message
- GUIDED
- arm throttle
- takeoff 10

+---------------------------------------------------------------------------------------------------+----------------------------------------------------------+
| Example MAVProxy/SITL Command                                                                     | Description                                              |
+===================================================================================================+==========================================================+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 6 3576 -353621474 1491651746 10 0 0 0 0 0 0 0 0``  | fly to lat,lon of -35.36,149.16 and 10m above home       |
+---------------------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 5 3576 -353621474 1491651746 600 0 0 0 0 0 0 0 0`` | fly to lat,lon of -35.36,149.16 and 600m above sea level |
+---------------------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 11 3576 -353621474 1491651746 10 0 0 0 0 0 0 0 0`` | fly to lat,lon of -35.36,149.16 and 10m above terrain    |
+---------------------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 6 3527 0 0 0 1 0 0 0 0 0 0 0``                     | fly North at 1m/s                                        |
+---------------------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 6 3135 0 0 0 0 0 0 1 0 0 0 0``                     | accelerate North at 1m/s                                 |
+---------------------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 6 2503 0 0 0 0 0 0 0 0 0 0.7854 0``                | turn to North-East (Yaw target + velocity of zero)       |
+---------------------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 6 1479 0 0 0 0 0 0 0 0 0 0 0.174``                 | rotate clock-wise at 10deg/sec (velocity of zero)        |
+---------------------------------------------------------------------------------------------------+----------------------------------------------------------+

.. _copter-commands-in-guided-mode_set_attitude_target:

SET_ATTITUDE_TARGET
-------------------

Set the vehicle's target attitude and climb rate or thrust.  This message is accepted in :ref:`Guided <copter:ac2_guidedmode>` or Guided_NoGPS (this is the only message accepted by Guided_NoGPS).  The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`__

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
   <td>

If GUID_OPTIONS = 0: climb rate where 0.5=no climb, 0=descend at WPNAV_SPEED_DN, 1=climb at WPNAV_SPEED_UP
If GUID_OPTIONS = 8: thrust from 0 to 1

.. raw:: html

   </td>
   </tr>
   </tbody>
   </table>

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- GUIDED
- arm throttle
- takeoff 10

+------------------------------------------+--------------------------------------------------------------+
| Example MAVProxy/SITL Command            | Description                                                  |
+==========================================+==============================================================+
| ``attitude 1 0 0 0 0.5``                 | hold level attitude with zero climb rate (if GUID_OPTIONS=0) |
|                                          | OR hold level attitude and 50% throttle (if GUID_OPTIONS=8)  |
+------------------------------------------+--------------------------------------------------------------+
| ``attitude 1 0 0 0 1.0``                 | climb at WPNAV_SPEED_UP (if GUID_OPTIONS=0) OR               |
|                                          | climb at 100% throttle (if GUID_OPTIONS=8)                   |
+------------------------------------------+--------------------------------------------------------------+
| ``attitude 1 0 0 0 0.0``                 | descend at WPNAV_SPEED_DN (if GUID_OPTIONS=0) OR             |
|                                          | descend at 0% throttle (if GUID_OPTIONS=8)                   |
+------------------------------------------+--------------------------------------------------------------+
| ``attitude 0.9961947 0.0871557 0 0 0.5`` | roll at 10deg with zero climb rate (if GUID_OPTIONS=0) OR    |
|                                          | roll at 10deg and 50% throttle (if GUID_OPTIONS=8)           |
+------------------------------------------+--------------------------------------------------------------+

