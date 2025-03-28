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
- `MAV_CMD_DO_PAUSE_CONTINUE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE>`__

Pausing a Mission with MAV_CMD_DO_PAUSE_CONTINUE
------------------------------------------------

A mission can be paused or resumed by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command, param1 and param2 fields set as specified for the `MAV_CMD_DO_PAUSE_CONTINUE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE>`__ command.

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
   <td><strong>target_component</strong></td>
   <td>uint8_t</td>
   <td>Component ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>command</strong></td>
   <td>uint16_t</td>
   <td>MAV_CMD_DO_PAUSE_CONTINUE=193</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>0:pause, 1:continue</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   </tbody>
   </table>

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter, "module load message"

+------------------------------------------------------+---------------------------+
| Example MAVProxy/SITL Command                        | Description               |
+======================================================+===========================+
| ``message COMMAND_LONG 0 0 193 0 0 0 0 0 0 0 0``     | pause mission             |
+------------------------------------------------------+---------------------------+
| ``message COMMAND_LONG 0 0 193 0 1 0 0 0 0 0 0``     | continue / resume mission |
+------------------------------------------------------+---------------------------+
