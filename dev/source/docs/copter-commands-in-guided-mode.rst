.. _copter-commands-in-guided-mode:

==============================
Copter Commands in Guided Mode
==============================

This article lists the commands that are handled by Copter in GUIDED
mode (for example, when writing GCS or Companion Computer apps in
`DroneKit <http://dronekit.io/>`__). Except where explicitly stated,
most of these can also be called in other modes too.

.. note::

   The list is inferred from Copter's
   `GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/GCS_Mavlink.cpp#L967>`__
   for AC3.3.

Movement commands
=================

These commands can only be called in GUIDED Mode. They are used for
position and velocity control of the vehicle.

:ref:`SET_POSITION_TARGET_LOCAL_NED <copter-commands-in-guided-mode_set_position_target_local_ned>`

:ref:`SET_POSITION_TARGET_GLOBAL_INT <copter-commands-in-guided-mode_set_position_target_global_int>`

:ref:`SET_ATTITUDE_TARGET (for Guided_NoGPS mode) <copter-commands-in-guided-mode_set_attitude_target>`

MAV_CMDs
=========

These MAV_CMDs can be processed if packaged within a
`COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ message.

:ref:`MAV_CMD_NAV_TAKEOFF <copter:mav_cmd_nav_takeoff>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_NAV_LOITER_UNLIM <copter:mav_cmd_nav_loiter_unlim>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_NAV_RETURN_TO_LAUNCH <copter:mav_cmd_nav_return_to_launch>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_NAV_LAND <copter:mav_cmd_nav_land>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_CONDITION_YAW <copter:mav_cmd_condition_yaw>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_CHANGE_SPEED <copter:mav_cmd_do_change_speed>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_SET_HOME <copter:mav_cmd_do_set_home>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_SET_ROI <copter:mav_cmd_do_set_roi>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_MISSION_START <copter:mav_cmd_mission_start>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_COMPONENT_ARM_DISARM <copter:mav_cmd_component_arm_disarm>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_SET_SERVO <copter:mav_cmd_do_set_servo>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_REPEAT_SERVO <copter:mav_cmd_do_repeat_servo>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_SET_RELAY <copter:mav_cmd_do_set_relay>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_REPEAT_RELAY <copter:mav_cmd_do_repeat_relay>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_FENCE_ENABLE <copter:mav_cmd_do_fence_enable>`
(Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_PARACHUTE <copter:mav_cmd_do_parachute>`
(If parachute enabled) (Copter 3.2.1 or earlier)

:ref:`MAV_CMD_DO_GRIPPER <copter:mav_cmd_do_gripper>`
(If gripper enabled) (Copter 3.2.1 or earlier)

`MAV_CMD_START_RX_PAIR <https://mavlink.io/en/messages/common.html#MAV_CMD_START_RX_PAIR>`__
(Copter 3.3) Starts receiver pairing

`MAV_CMD_PREFLIGHT_CALIBRATION <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_CALIBRATION>`__
(Copter 3.3)

`MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS>`__
(Copter 3.3)

`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN>`__
(Copter 3.3)

`MAV_CMD_DO_MOTOR_TEST <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_MOTOR_TEST>`__
(Copter 3.3)

`MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES>`__
(Copter 3.3)

`MAV_CMD_GET_HOME_POSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_GET_HOME_POSITION>`__
(Copter 3.3)

`MAV_CMD_DO_START_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_START_MAG_CAL>`__
(Master - not in Copter 3.3)

`MAV_CMD_DO_ACCEPT_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_ACCEPT_MAG_CAL>`__
(Master - not in Copter 3.3)

`MAV_CMD_DO_CANCEL_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_CANCEL_MAG_CAL>`__
(Master - not in Copter 3.3)

`MAV_CMD_DO_FLIGHTTERMINATION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION>`__
(Copter 3.3) Disarms motors immediately (Copter falls!).

MAV_CMD_DO_SEND_BANNER - No link available (?)

These MAV_CMD commands can be sent as their own message type (not
inside `:ref:`COMMAND_LONG``): `MAV_CMD_DO_DIGICAM_CONFIGURE <copter:mav_cmd_do_digicam_configure>`

:ref:`MAV_CMD_DO_DIGICAM_CONTROL <copter:mav_cmd_do_digicam_control>`

`MAV_CMD_DO_MOUNT_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE>`__

:ref:`MAV_CMD_DO_MOUNT_CONTROL <copter:mav_cmd_do_mount_control>`

Other commands
==============

Below are other (non-MAV_CMD) commands that will be handled by Copter
in GUIDED mode.

.. note::

   Most of these commands are not relevant to DroneKit-Python apps or
   are already provided through the API.

`HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`__

`SET_MODE <https://mavlink.io/en/messages/common.html#SET_MODE>`__

`PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`__

`PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`__

`PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`__

`MISSION_WRITE_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST>`__

`MISSION_ITEM <https://mavlink.io/en/messages/common.html#MISSION_ITEM>`__

`MISSION_REQUEST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST>`__

`MISSION_SET_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT>`__

`MISSION_REQUEST_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST>`__

`MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`__

`MISSION_CLEAR_ALL <https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL>`__

`REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`__

`GIMBAL_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_REPORT>`__

`RC_CHANNELS_OVERRIDE <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`__

`COMMAND_ACK <https://mavlink.io/en/messages/common.html#COMMAND_ACK>`__

`HIL_STATE <https://mavlink.io/en/messages/common.html#HIL_STATE>`__

`RADIO <https://mavlink.io/en/messages/ardupilotmega.html#RADIO>`__

`RADIO_STATUS <https://mavlink.io/en/messages/common.html#RADIO_STATUS>`__

`LOG_REQUEST_DATA <https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA>`__

`LOG_ERASE <https://mavlink.io/en/messages/common.html#LOG_ERASE>`__

`LOG_REQUEST_LIST <https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST>`__

`LOG_REQUEST_END <https://mavlink.io/en/messages/common.html#LOG_REQUEST_END>`__

`SERIAL_CONTROL <https://mavlink.io/en/messages/common.html#SERIAL_CONTROL>`__

`GPS_INJECT_DATA <https://mavlink.io/en/messages/common.html#GPS_INJECT_DATA>`__

`TERRAIN_DATA <https://mavlink.io/en/messages/common.html#TERRAIN_DATA>`__

`TERRAIN_CHECK <https://mavlink.io/en/messages/common.html#TERRAIN_CHECK>`__

`RALLY_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_POINT>`__

`RALLY_FETCH_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_FETCH_POINT>`__

`AUTOPILOT_VERSION_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#AUTOPILOT_VERSION_REQUEST>`__

`LED_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#LED_CONTROL>`__

`ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`__

`REMOTE_LOG_BLOCK_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_BLOCK_STATUS>`__

`LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`__
(Planned for Copter 3.4)

:ref:`SET_HOME_POSITION <copter-commands-in-guided-mode_set_home_position>` (Master branch - not in
Copter 3.3)

Command definitions
===================

This section contains information about some immediate commands
supported by Copter (Mission Commands are documented in :ref:`MAVLink Mission Command Messages (MAV_CMD) <copter:common-mavlink-mission-command-messages-mav_cmd>`).

.. note::

   Editors: It may make sense to merge the immediate command
   information for Copter/Plane/Rover as done for :ref:`Mission Commands <planner:common-mavlink-mission-command-messages-mav_cmd>`
   when we have a few more

.. _copter-commands-in-guided-mode_set_position_target_local_ned:

SET_POSITION_TARGET_LOCAL_NED
-----------------------------

Set vehicle position or velocity setpoint in local frame.

.. note::

   Starting in Copter 3.3, velocity commands should be resent every
   second (the vehicle will stop after a few seconds if no command is
   received). Prior to Copter 3.3 the command was persistent, and would
   only be interrupted when the next movement command was received.

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
   Timestamp in milliseconds since system boot. The rationale for the
   timestamp in the setpoint is to allow the system to compensate for the
   transport delay of the setpoint. This allows the system to compensate
   processing latency.
   </td>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>System ID</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>Component ID</td>
   </tr>
   <tr>
   <td><strong>coordinate_frame</strong></td>
   <td>Valid options are listed below</td>
   </tr>
   <tr>
   <td><strong>type_mask</strong></td>
   <td>

Bitmask to indicate which dimensions should be ignored by the vehicle (a
value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
the setpoint dimensions should be ignored). Mapping: bit 1: x, bit 2: y,
bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9:

.. note::

   At
   time of writing you **must** enable all three bits for the position
   (0b0000111111111000) OR all three bits for the velocity
   (0b0000111111000111). Setting just one bit of the position/velocity or
   mixing the bits is not supported. The **acceleration**, **yaw**,
   **yaw_rate** are present in the `protocol definition <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__
   but are not supported by ArduPilot.

   
.. raw:: html
   
   </td>
   </tr>
   <tr>
   <td><strong>x</strong></td>
   <td>X Position in specified NED frame in meters</td>
   </tr>
   <tr>
   <td><strong>y</strong></td>
   <td>y Position in specified NED frame in meters</td>
   </tr>
   <tr>
   <td><strong>z</strong></td>
   <td>Z Position in specified NED frame in meters (note, altitude is negative in NED)</td>
   </tr>
   <tr>
   <td><strong>vx</strong></td>
   <td>X velocity in specified NED frame in meter/s</td>
   </tr>
   <tr>
   <td><strong>vy</strong></td>
   <td>Y velocity in NED frame in meter/s</td>
   </tr>
   <tr>
   <td><strong>vz</strong></td>
   <td>Z velocity in NED frame in meter/s</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afx</td>
   <td>X acceleration or force (if bit 10 of type_mask is set) in specified NED frame in meter/s^2 or N</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afy</td>
   <td>Y acceleration or force (if bit 10 of type_mask is set) in specified NED frame in meter/s^2 or N</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afz</td>
   <td>Z acceleration or force (if bit 10 of type_mask is set) in specified NED frame in meter/s^2 or N</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>yaw</td>
   <td>yaw setpoint in rad</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>yaw_rate</td>
   <td>yaw rate setpoint in rad/s</td>
   </tr>
   </tbody>
   </table>

.. note::

   The ``co-ordinate frame`` information below applies from AC3.3.
   Prior to AC3.3, the field is ignored and all values are relative to the
   ``MAV_FRAME_LOCAL_NED`` frame.

The ``co-ordinate frame`` field takes the following values:

+--------------------------------------+--------------------------------------+
| Frame                                | Description                          |
+======================================+======================================+
| ``MAV_FRAME_LOCAL_NED``              | Positions are relative to the        |
|                                      | vehicle's *home position* in the     |
|                                      | North, East, Down (NED) frame. Use   |
|                                      | this to specify a position x metres  |
|                                      | north, y metres east and (-) z       |
|                                      | metres above the home position.      |
|                                      |                                      |
|                                      | Velocity directions are in the       |
|                                      | North, East, Down (NED) frame.       |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_LOCAL_OFFSET_NED``       | Positions are relative to the        |
|                                      | current vehicle position in the      |
|                                      | North, East, Down (NED) frame. Use   |
|                                      | this to specify a position x metres  |
|                                      | north, y metres east and (-) z       |
|                                      | metres of the current vehicle        |
|                                      | position.                            |
|                                      |                                      |
|                                      | Velocity directions are in the       |
|                                      | North, East, Down (NED) frame.       |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_BODY_OFFSET_NED``        | Positions are relative to the        |
|                                      | current vehicle position in a frame  |
|                                      | based on the vehicle's current       |
|                                      | heading. Use this to specify a       |
|                                      | position x metres forward from the   |
|                                      | current vehicle position, y metres   |
|                                      | to the right, and z metres down      |
|                                      | (forward, right and down are         |
|                                      | "positive" values).                  |
|                                      |                                      |
|                                      |                                      |
|                                      | Velocity directions are relative to  |
|                                      | the current vehicle heading. Use     |
|                                      | this to specify the speed forward,   |
|                                      | right and down (or the opposite if   |
|                                      | you use negative values).            |
+--------------------------------------+--------------------------------------+
| ``MAV_FRAME_BODY_NED``               | Positions are relative to the        |
|                                      | vehicle's *home position* in the     |
|                                      | North, East, Down (NED) frame. Use   |
|                                      | this to specify a position x metres  |
|                                      | north, y metres east and (-) z       |
|                                      | metres above the home position.      |
|                                      |                                      |
|                                      |                                      |
|                                      | Velocity directions are relative to  |
|                                      | the current vehicle heading. Use     |
|                                      | this to specify the speed forward,   |
|                                      | right and down (or the opposite if   |
|                                      | you use negative values).            |
+--------------------------------------+--------------------------------------+

.. tip::

   In frames, ``_OFFSET_`` means "relative to vehicle position" while
   ``_LOCAL_`` is "relative to home position" (these have no impact on
   *velocity* directions). ``_BODY_`` means that velocity components are
   relative to the heading of the vehicle rather than the NED frame.

   
.. _copter-commands-in-guided-mode_set_position_target_global_int:

SET_POSITION_TARGET_GLOBAL_INT
------------------------------

Set vehicle position, velocity and acceleration setpoint in the WGS84
coordinate system.

.. note::

   Starting in Copter 3.3, velocity commands should be resent every
   second (the vehicle will stop after a few seconds if no command is
   received). Prior to Copter 3.3 the command was persistent, and would
   only be interrupted when the next movement command was received.

The protocol definition is here:
`SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`__.

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
   Timestamp in milliseconds since system boot. The rationale for the
   timestamp in the setpoint is to allow the system to compensate for the
   transport delay of the setpoint. This allows the system to compensate
   processing latency.
   </td>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>System ID</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>Component ID</td>
   </tr>
   <tr>
   <td><strong>coordinate_frame</strong></td>
   <td>
   Valid options are: MAV_FRAME_GLOBAL_INT,
   MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

   </td>
   </tr>
   <tr>
   <td><strong>type_mask</strong></td>
   <td>

Bitmask to indicate which dimensions should be ignored by the vehicle (a
value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
the setpoint dimensions should be ignored). Mapping: bit 1: x, bit 2: y,
bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9:

.. note::

   At
   time of writing you **must** enable all three bits for the position
   (0b0000111111111000) OR all three bits for the velocity
   (0b0000111111000111). Setting just one bit of the position/velocity or
   mixing the bits is not supported. The **acceleration**, **yaw**,
   **yaw_rate** are present in the `protocol definition <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__
   but are not supported by ArduPilot.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>lat_int</strong></td>
   <td>X Position in WGS84 frame in 1e7 \* meters</td>
   </tr>
   <tr>
   <td><strong>lon_int</strong></td>
   <td>Y Position in WGS84 frame in 1e7 \* meters</td>
   </tr>
   <tr>
   <td><strong>alt</strong></td>
   <td>Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT</td>
   </tr>
   <tr>
   <td><strong>vx</strong></td>
   <td>X velocity in MAV_FRAME_LOCAL_NED frame in meter/s</td>
   </tr>
   <tr>
   <td><strong>vy</strong></td>
   <td>Y velocity in MAV_FRAME_LOCAL_NED frame in meter/s</td>
   </tr>
   <tr>
   <td><strong>vz</strong></td>
   <td>Z velocity in MAV_FRAME_LOCAL_NED frame in meter/s</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afx</td>
   <td>X acceleration or force (if bit 10 of type_mask is set) in specified
   MAV_FRAME_LOCAL_NED frame in meter/s^2 or N
   </td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afy</td>
   <td>Y acceleration or force (if bit 10 of type_mask is set) in specified MAV_FRAME_LOCAL_NED frame in meter/s^2 or N</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>afz</td>
   <td>Z acceleration or force (if bit 10 of type_mask is set) in specified MAV_FRAME_LOCAL_NED frame in meter/s^2 or N</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>yaw</td>
   <td>yaw setpoint in rad</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>yaw_rate</td>
   <td>yaw rate setpoint in rad/s</td>
   </tr>
   </tbody>
   </table>

   
.. _copter-commands-in-guided-mode_set_home_position:

SET_HOME_POSITION
-------------------

The position the system will return to and land on. The position is set
automatically by the system during the takeoff if it has not been
explicitly set by the operator before or after.

.. note::

   Not in Copter 3.3 (currently in master)

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
   <td>Latitude (WGS84), in degrees \* 1E7</td>
   </tr>
   <tr>
   <td><strong>longitude</strong></td>
   <td>int32_t</td>
   <td>Longitude (WGS84), in degrees \* 1E7</td>
   </tr>
   <tr>
   <td><strong>altitude</strong></td>
   <td>int32_t</td>
   <td>Altitude (AMSL), in meters \* 1000 (positive for up)</td>
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

The protocol definition for this command is here:
`SET_HOME_POSITION <https://mavlink.io/en/messages/common.html#SET_HOME_POSITION>`__


.. _copter-commands-in-guided-mode_set_attitude_target:

SET_ATTITUDE_TARGET
-------------------

Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).

.. note::

   Only available in Copter-3.4 (or higher)

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
   <td>Timestamp in milliseconds since system boot. Used to avoid duplicate commands. 0 to ignore.</td>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>uint8_t</td>
   <td>System ID</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>int8_t</td>
   <td>Component ID</td>
   </tr>
   <tr>
   <td><strong>type_mask</strong></td>
   <td>int8_t</td>
   <td>
   Mappings: If any of these bits are set, the corresponding input should be ignored: 
   (LSB is bit 1)
   bit 1: body roll rate, 
   bit 2: body pitch rate, 
   bit 3: body yaw rate. 
   bit 4-bit 6: reserved, 
   bit 7: throttle
   bit 8: attitude
   <br>
   Currently, throttle and attitude must be set to 0, i.e. not ignored
   </td>
   </tr>
   <tr>
   <td><strong>q</strong></td>
   <td>float[4]</td>
   <td>
   Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})
   <br>
   Note that zero-rotation causes vehicle to rotate towards 0 yaw.
   </td>
   </tr>
   <tr>
   <td><strong>body_roll_rate</strong></td>
   <td>float</td>
   <td>Body roll rate in radians per second</td>
   </tr>
   <tr>
   <td><strong>body_pitch_rate</strong></td>
   <td>float</td>
   <td>Body pitch rate in radians per second</td>
   </tr>
   <tr>
   <td><strong>body_yaw_rate</strong></td>
   <td>float</td>
   <td>Body yaw rate in radians per second</td>
   </tr>
   <tr>
   <td><strong>thrust</strong></td>
   <td>float</td>
   <td>Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
   </td>
   </tr>
   </tbody>
   </table>

The protocol definition for this command is here:
`SET_ATTITUDE_TARGET <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`__
