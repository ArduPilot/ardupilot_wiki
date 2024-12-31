.. _plane-commands-in-guided-mode:

=============================
Plane Commands in Guided Mode
=============================

This article lists the MAVLink commands that affect the movement of a Plane or QuadPlane.  Normally these commands are sent by a ground station or :ref:`Companion Computers <companion-computers>` often running `DroneKit <http://dronekit.io/>`__.

.. note::

   The code which processes these commands can be found `here in GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/GCS_Mavlink.cpp>`__

.. _movement_commands:
Movement commands
=================

To command a plane to move to a particular lat, lon, alt send a :ref:`MAV_CMD_NAV_WAYPOINT <plane:mav_cmd_nav_waypoint>` (16) within a `MISSION_ITEM_INT <https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT>`__ with the following fields set as follows:

- "current" = 2 to indicate that it is a guided mode "goto" message
- "frame" = 0 or 3 for alt-above-sea-level, 6 for alt-above-home or 11 for alt-above-terrain
- "x" = longitude * 1e7
- "y" = latitude * 1e7
- "z" = altitude in meters

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands:

- module load message
- create an Auto mission ("wp editor") with a NAV_TAKEOFF command with Alt of 100m
- arm throttle
- Auto (wait until vehicle begins circling)
- GUIDED (and enter one of the commands below)

+--------------------------------------------------------------------------------+----------------------------------------------------------+
| Example MAVProxy/SITL Command                                                  | Description                                              |
+================================================================================+==========================================================+
| ``message MISSION_ITEM_INT 0 0 0 0 16 2 0 0 0 0 0 -353621474 1491651746 700``  | fly to lat,lon of -35.36,149.16 and 700m above sea level |
+--------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message MISSION_ITEM_INT 0 0 0 6 16 2 0 0 0 0 0 -353621474 1491651746 100``  | fly to lat,lon of -35.36,149.16 and 100m above home      |
+--------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message MISSION_ITEM_INT 0 0 0 11 16 2 0 0 0 0 0 -353621474 1491651746 100`` | fly to lat,lon of -35.36,149.16 and 100m above terrain   |
+--------------------------------------------------------------------------------+----------------------------------------------------------+

MAV_CMDs
=========

These MAV_CMDs can be processed if packaged within a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ message

- :ref:`MAV_CMD_NAV_LOITER_UNLIM <plane:mav_cmd_nav_loiter_unlim>`
- :ref:`MAV_CMD_NAV_RETURN_TO_LAUNCH <plane:mav_cmd_nav_return_to_launch>`

These MAV_CMDs can be processed if packaged within a `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`__ message

- `MAV_CMD_DO_REPOSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION>`__
- `MAV_CMD_GUIDED_CHANGE_SPEED <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED>`__
- `MAV_CMD_GUIDED_CHANGE_ALTITUDE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_ALTITUDE>`__
- `MAV_CMD_GUIDED_CHANGE_HEADING <https://mavlink.io/en/messages/common.html#MAV_CMD_GUIDED_CHANGE_HEADING>`__

Position Targets
================

To command the altitude in any supported frame, use `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`__.
Ensure to set the `POSITION_TARGET_TYPEMASK <https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK>`__ bit 3.

To command the altitude in LOCAL frame, use `SET_POSITION_TARGET_LOCAL_NED <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__.
It is currently not necessary to bother with `POSITION_TARGET_TYPEMASK <https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK>`__.
The `MAV_FRAME <https://mavlink.io/en/messages/common.html#MAV_FRAME>`__ must be ``MAV_FRAME_LOCAL_OFFSET_NED``.

No other fields in position target messages are currently supported. Use :ref:`movement_commands` to send a lat lon alt target.

Attitude Targets
================

The low level attitude of the vehicle can be controlled with `SET_ATTITUDE_TARGET <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`__.

