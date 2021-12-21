.. _mavlink-mission-upload-download:

=========================
Mission Upload / Download
=========================

This page explains how MAVLink can be used to upload and download missions (executed in :ref:`Auto mode <copter:auto-mode>`) and perform some other mission related actions.  The user wiki page for :ref:`Mission Planning is here <copter:common-mission-planning>`.

Mission Related Messages
------------------------

- `MISSION_ACK <https://mavlink.io/en/messages/common.html#MISSION_ACK>`__
- `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`__
- `MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`__
- `MISSION_CLEAR_ALL <https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL>`__
- `MISSION_ITEM_INT <https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT>`__
- `MISSION_ITEM_REACHED <https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED>`__
- `MISSION_REQUEST_INT <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT>`__
- `MISSION_REQUEST_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST>`__
- `MISSION_REQUEST_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_PARTIAL_LIST>`__
- `MISSION_SET_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT>`__
- `MISSION_WRITE_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST>`__

Also the following MAV_CMDs may be sent within a COMMAND_LONG

- `MAV_CMD_MISSION_START <https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START>`__
- `MAV_CMD_DO_CHANGE_SPEED <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED>`__
