.. _plane-commands-in-guided-mode:

=============================
Plane Commands in Guided Mode
=============================

This article lists the commands that are handled by Plane in GUIDED mode
(for example, when writing GCS or Companion Computer apps in
`DroneKit <http://dronekit.io/>`__). Except where explicitly stated,
most of these can also be called in other modes too.

.. note::

   The list is inferred from Plane's
   `GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/GCS_Mavlink.cpp>`__

Movement commands
=================

For movement, Plane uses:

:ref:`MAV_CMD_NAV_WAYPOINT <plane:mav_cmd_nav_waypoint>`
message encoded with the "current" parameter set to "2" to indicate that
it is a guided mode "goto" message.

MAV_CMDs
=========

These MAV_CMDs can be processed if packaged within a
`COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ message.

:ref:`MAV_CMD_NAV_LOITER_UNLIM <plane:mav_cmd_nav_loiter_unlim>`

:ref:`MAV_CMD_NAV_RETURN_TO_LAUNCH <plane:mav_cmd_nav_return_to_launch>`

:ref:`MAV_CMD_DO_SET_ROI <plane:mav_cmd_do_set_roi>`

MAV_CMD_MISSION_START 

MAV_CMD_COMPONENT_ARM_DISARM

.. todo::

    MAV_CMD_MISSION_START and MAV_CMD_COMPONENT_ARM_DISARM not implemented as
    auto commands (or at least not when I did the master doc. Need to
    confirm that they aren't AUTO commands, that they are guided commands,
    and if so then document either here or in the command list.

:ref:`MAV_CMD_DO_SET_SERVO <plane:mav_cmd_do_set_servo>`

:ref:`MAV_CMD_DO_REPEAT_SERVO <plane:mav_cmd_do_repeat_servo>`

:ref:`MAV_CMD_DO_SET_RELAY <plane:mav_cmd_do_set_relay>`

:ref:`MAV_CMD_DO_REPEAT_RELAY <plane:mav_cmd_do_repeat_relay>`

:ref:`MAV_CMD_DO_FENCE_ENABLE <plane:mav_cmd_do_fence_enable>`

:ref:`MAV_CMD_DO_SET_HOME <plane:mav_cmd_do_set_home>`

MAV_CMD_START_RX_PAIR

MAV_CMD_PREFLIGHT_CALIBRATION

MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS

MAV_CMD_DO_SET_MODE

MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

MAV_CMD_DO_LAND_START

MAV_CMD_DO_GO_AROUND

MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES

:ref:`MAV_CMD_DO_AUTOTUNE_ENABLE <plane:mav_cmd_do_autotune_enable>`

These MAV_CMD commands can be sent as their own message type (not
inside `:ref:`COMMAND_LONG``): `MAV_CMD_DO_DIGICAM_CONFIGURE <plane:mav_cmd_do_digicam_configure>`

:ref:`MAV_CMD_DO_DIGICAM_CONTROL <plane:mav_cmd_do_digicam_control>`

MAV_CMD_DO_MOUNT_CONFIGURE

:ref:`MAV_CMD_DO_MOUNT_CONTROL <plane:mav_cmd_do_mount_control>`

Other commands
==============

Below are other (non-MAV_CMD) commands that will be handled by Plane in
GUIDED mode.

.. note::

   Most of these commands are not relevant to DroneKit-Python apps or
   are already provided through the API.

`SET_MODE <https://mavlink.io/en/messages/common.html#SET_MODE>`__

`MISSION_REQUEST_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST>`__

`MISSION_REQUEST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST>`__

MISSION_ACK:

`PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`__

`PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`__

`MISSION_CLEAR_ALL <https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL>`__

`MISSION_SET_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT>`__

`MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`__

`MISSION_WRITE_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST>`__

`MISSION_ITEM <https://mavlink.io/en/messages/common.html#MISSION_ITEM>`__

MAVLINK_MSG_ID_FENCE_POINT

MAVLINK_MSG_ID_FENCE_FETCH_POINT

RALLY_POINT

RALLY_FETCH_POINT

`PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`__

GIMBAL_REPORT

`RC_CHANNELS_OVERRIDE <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`__

`HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`__

`HIL_STATE <https://mavlink.io/en/messages/common.html#HIL_STATE>`__

RADIO

`RADIO_STATUS <https://mavlink.io/en/messages/common.html#RADIO_STATUS>`__

`LOG_REQUEST_DATA <https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA>`__

`LOG_ERASE <https://mavlink.io/en/messages/common.html#LOG_ERASE>`__

`LOG_REQUEST_LIST <https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST>`__

`LOG_REQUEST_END <https://mavlink.io/en/messages/common.html#LOG_REQUEST_END>`__

`SERIAL_CONTROL <https://mavlink.io/en/messages/common.html#SERIAL_CONTROL>`__

`GPS_INJECT_DATA <https://mavlink.io/en/messages/common.html#GPS_INJECT_DATA>`__

`TERRAIN_DATA <https://mavlink.io/en/messages/common.html#TERRAIN_DATA>`__

`TERRAIN_CHECK <https://mavlink.io/en/messages/common.html#TERRAIN_CHECK>`__

AUTOPILOT_VERSION_REQUEST

`REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`__
