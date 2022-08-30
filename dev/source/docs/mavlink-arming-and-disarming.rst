.. _mavlink-arming-and-disarming:

====================
Arming and Disarming
====================

This page explains how MAVLink can be used by a ground station or companion computer to arm or disarm a vehicle.  The user wiki page for :ref:`arming and disarming is here <copter:arming_the_motors>`.

MAV_CMD_COMPONENT_ARM_DISARM within COMMAND_LONG
------------------------------------------------

Attempt to arm or disarm the vehicle by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command, param1 and param2 fields set as specified for the `MAV_CMD_COMPONENT_ARM_DISARM <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>`__ command.

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
   <td>MAV_CMD_COMPONENT_ARM_DISARM=400</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>0:disarm, 1:arm</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>0: arm-disarm unless prevented by safety checks, 21196: force arming or disarming</td>
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

+------------------------------------------------------+-----------------------------------------------------+
| Example MAVProxy/SITL Command                        | Description                                         |
+======================================================+=====================================================+
| ``message COMMAND_LONG 0 0 400 0 1 0 0 0 0 0 0``     | arm the vehicle (may fail because of arming checks) |
+------------------------------------------------------+-----------------------------------------------------+
| ``message COMMAND_LONG 0 0 400 0 1 21196 0 0 0 0 0`` | force arm the vehicle (try to bypass arming checks) |
+------------------------------------------------------+-----------------------------------------------------+
| ``message COMMAND_LONG 0 0 400 0 0 0 0 0 0 0 0``     | disarm the vehicle (may fail if not landed)         |
+------------------------------------------------------+-----------------------------------------------------+
| ``message COMMAND_LONG 0 0 400 0 0 21196 0 0 0 0 0`` | force disarm the vehicle even if flying             |
+------------------------------------------------------+-----------------------------------------------------+
