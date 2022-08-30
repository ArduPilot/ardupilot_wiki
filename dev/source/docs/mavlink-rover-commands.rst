.. _mavlink-rover-commands:

=============================
Rover Commands in Guided Mode
=============================

This page explains how MAVLink can be used by a ground station or companion computer to control the motion of a Rover or Boat while in :ref:`Guided mode <rover:guided-mode>`.

Movement commands
=================

These commands can be used to control the vehicle's position, velocity or attitude while in Guided Mode

- :ref:`SET_POSITION_TARGET_LOCAL_NED <mavlink-rover-commands_set_position_target_local_ned>`
- :ref:`SET_POSITION_TARGET_GLOBAL_INT <mavlink-rover-commands_set_position_target_global_int>`
- :ref:`SET_ATTITUDE_TARGET <mavlink-rover-commands_set_attitude_target>` (supported in Guided and Guided_NoGPS modes)


Movement Command Details
========================

This section contains details of MAVLink commands to move the vehicle

.. _mavlink-rover-commands_set_position_target_local_ned:

SET_POSITION_TARGET_LOCAL_NED
-----------------------------

Set the vehicle's target position (as an offset in NED from the EKF origin), velocity, heading or turn rate.  The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>time_boot_ms</strong></td>
   <td>Sender's system time in milliseconds since boot</td>
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

When providing position or velocity both X and Y axis must be provided.  At least one of position, velocity, yaw or yaw rate must be provided.  If position is provided velocity, yaw and yaw rate are ignored. Acceleration is not supported.

- Use Position : 0b110111111100 / 0x0DFC / 3580 (decimal)
- Use Velocity : 0b110111100111 / 0x0DE7 / 3559 (decimal)
- Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
- Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)
- Use Vel+Yaw : 0b100111100111 / 0x09E7 / 2535 (decimal)
- Use Vel+Yaw Rate : 0b010111100111 / 0x05E7 / 1511 (decimal)
   
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
   <tr style="color: #c0c0c0">
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
   <tr style="color: #c0c0c0">
   <td><strong>vz</strong></td>
   <td>Z velocity in m/s (positive is down)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>afx</strong></td>
   <td>X acceleration in m/s/s (positive is forward or North)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>afy</strong></td>
   <td>Y acceleration in m/s/s (positive is right or East)</td>
   </tr>
   <tr style="color: #c0c0c0">
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
|                                      | Velocity is in NED frame             |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_LOCAL_OFFSET_NED`` (7)   | Positions are relative to the        |
|                                      | vehicle's current position           |
|                                      |                                      |
|                                      | I.e. x=1,y=2,z=3 is 1m North,        |
|                                      | 2m East and 3m below the current     |
|                                      | position.                            |
|                                      |                                      |
|                                      | Velocity is in NED frame             |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_BODY_NED`` (8)           | Positions are relative to the        |
|                                      | EKF Origin in NED frame              |
|                                      |                                      |
|                                      | I.e x=1,y=2,z=3 is 1m North, 2m East |
|                                      | and 3m Down from the origin          |
|                                      |                                      |
|                                      | Velocity are relative to the         |
|                                      | vehicle's current heading.  Use this |
|                                      | to specify the speed forward or      |
|                                      | backwards (if you use negative       |
|                                      | values)                              |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_BODY_OFFSET_NED`` (9)    | Positions are relative to the        |
|                                      | vehicle's current position and       |
|                                      | heading                              |
|                                      |                                      |
|                                      | I.e x=1,y=2,z=3 is 1m forward,       |
|                                      | 2m right and 3m Down from the current|
|                                      | position                             |
|                                      |                                      |
|                                      | Velocity are relative to the         |
|                                      | vehicle's current heading.  Use this |
|                                      | to specify the speed forward or      |
|                                      | backwards (if you use negative       |
|                                      | values)                              |
+--------------------------------------+--------------------------------------+

.. tip::

   In frames, ``_OFFSET_`` means "relative to vehicle position" while ``_LOCAL_`` is "relative to home position" (these have no impact on *velocity* directions). ``_BODY_`` means that velocity components are relative to the heading of the vehicle rather than the NED frame.

.. note::

   If sending velocity commands, they should be re-sent every second (the vehicle will stop after 3 seconds if no command is received)

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- module load message
- GUIDED
- arm throttle

+----------------------------------------------------------------------------------+-----------------------------------------------------+
| Example MAVProxy/SITL Command                                                    | Description                                         |
+==================================================================================+=====================================================+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 3580 100 0 0 0 0 0 0 0 0 0 0``   | move to 100m North of the EKF origin                |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 7 3580 10 0 0 0 0 0 0 0 0 0 0``    | move 10m North of the current position              |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3580 10 0 0 0 0 0 0 0 0 0 0``    | move 10m forward of the current position            |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 3559 0 0 0 1 0 0 0 0 0 0 0``     | move North at 1m/s                                  |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3559 0 0 0 1 0 0 0 0 0 0 0``     | move forward at 1m/s                                |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 2535 0 0 0 0 0 0 0 0 0 0.7854 0``| turn to North-East (Yaw target + velocity of zero)  |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 2535 0 0 0 0 0 0 0 0 0 0.7854 0``| turn 45deg to right (Yaw target + velocity of zero) |
+----------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 1511 0 0 0 0 0 0 0 0 0 0 0.174`` | rotate clock-wise at 10deg/sec (velocity of zero)   |
+----------------------------------------------------------------------------------+-----------------------------------------------------+

.. _mavlink-rover-commands_set_position_target_global_int:

SET_POSITION_TARGET_GLOBAL_INT
------------------------------

Set the vehicle's target position (in WGS84 coordinates), velocity, heading or turn rate.  This is similar to the SET_POSITION_TARGET_LOCAL_NED message (see above) except positions are provided as latitude and longitude values.

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
   <td>Sender's system time in milliseconds since boot</td>
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

Valid options are below but are treated the same:

- MAV_FRAME_GLOBAL (0): alt is meters above sea level
- MAV_FRAME_GLOBAL_INT (5): alt is meters above sea level
- MAV_FRAME_GLOBAL_RELATIVE_ALT (3): alt is meters above home
- MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (6): alt is meters above home
- MAV_FRAME_GLOBAL_TERRAIN_ALT (10): alt is meters above terrain
- MAV_FRAME_GLOBAL_TERRAIN_ALT_INT (11): alt is meters above terrain

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>type_mask</strong></td>
   <td>

Bitmask to indicate which fields should be **ignored** by the vehicle (see POSITION_TARGET_TYPEMASK enum)

bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

When providing position or velocity both X and Y axis must be provided.  At least one of position, velocity, yaw or yaw rate must be provided.  If position is provided velocity, yaw and yaw rate are ignored. Acceleration is not supported.

- Use Position : 0b110111111100 / 0x0DFC / 3580 (decimal)
- Use Velocity : 0b110111100111 / 0x0DE7 / 3559 (decimal)
- Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
- Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)
- Use Vel+Yaw : 0b100111100111 / 0x09E7 / 2535 (decimal)
- Use Vel+Yaw Rate : 0b010111100111 / 0x05E7 / 1511 (decimal)

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
   <tr style="color: #c0c0c0">
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
   <tr style="color: #c0c0c0">
   <td><strong>vz</strong></td>
   <td>Z velocity in m/s (positive is down)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>afx</strong></td>
   <td>X acceleration in m/s/s (positive is North)</td>
   </td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>afy</strong></td>
   <td>Y acceleration in m/s/s (positive is East)</td>
   </tr>
   <tr style="color: #c0c0c0">
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

   If sending velocity commands, they should be re-sent every second (the vehicle will stop after 3 seconds if no command is received)

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- module load message
- GUIDED
- arm throttle

+-------------------------------------------------------------------------------------------------+-----------------------------------------------------+
| Example MAVProxy/SITL Command                                                                   | Description                                         |
+=================================================================================================+=====================================================+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 3 3580 -353621474 1491651746 0 0 0 0 0 0 0 0 0`` | move to lat,lon of -35.36,149.16                    |
+-------------------------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 3 3559 0 0 0 1 0 0 0 0 0 0 0``                   | move North at 1m/s                                  |
+-------------------------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 3 2535 0 0 0 0 0 0 0 0 0 0.7854 0``              | turn to North-East (Yaw target + velocity of zero)  |
+-------------------------------------------------------------------------------------------------+-----------------------------------------------------+
| ``message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 3 1511 0 0 0 0 0 0 0 0 0 0 0.174``               | rotate clock-wise at 10deg/sec (velocity of zero)   |
+-------------------------------------------------------------------------------------------------+-----------------------------------------------------+

.. _mavlink-rover-commands_set_attitude_target:

SET_ATTITUDE_TARGET
-------------------

Set the vehicle's target heading and speed.  The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`__

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

- Use Yaw Rate + Throttle : 0b10100011 / 0xA3 / 163 (decimal)
- Use Attitude + Throttle:  0b00100111 / 0x27 / 39 (decimal)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>q</strong></td>
   <td>float[4]</td>
   <td>
   Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})
   <br>
   Note that zero-rotation causes vehicle to point North.
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
   <td>Body yaw rate in radians</td>
   </tr>
   <tr>
   <td><strong>thrust</strong></td>
   <td>float</td>
   <td>

0=throttle 0%, +1=forward at WP_SPEED, -1=backwards at WP_SPEED

.. raw:: html

   </td>
   </tr>
   </tbody>
   </table>

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- GUIDED
- arm throttle

+------------------------------------------+--------------------------------------------------+
| Example MAVProxy/SITL Command            | Description                                      |
+==========================================+==================================================+
| ``attitude 1 0 0 0 1``                   | face North, move forward at WP_SPEED             |
+------------------------------------------+--------------------------------------------------+
| ``attitude 1 0 0 0 -1``                  | face North, move in reverse at WP_SPEED          |
+------------------------------------------+--------------------------------------------------+
| ``attitude 0.9238795 0 0 0.3826834 0.5`` | face North-East, move forward at 1/2 of WP_SPEED |
+------------------------------------------+--------------------------------------------------+

