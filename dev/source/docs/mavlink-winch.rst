.. _mavlink-winch:

===============
Control a Winch
===============

This page explains how MAVLink can be used by a ground station or companion computer to control a winch.  The :ref:`user wiki pages for the winch is here <copter:common-daiwa-winch>`.

Control the winch with MAV_CMD_DO_WINCH
---------------------------------------

The servo's position can be set by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command, param1 and param2 fields set as specified for the `MAV_CMD_DO_WINCH <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_WINCH>`__ command.

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
   <td>System ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>uint8_t</td>
   <td>Component ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>command</strong></td>
   <td>uint16_t</td>
   <td>MAV_CMD_DO_WINCH=42600</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Winch instance number</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Action (0:Relax, 1:Length Control, 2:Rate Control)</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Length of line to release (negative to wind in)</td>
   </tr>
   <tr>
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>Release rate (negative to wind in)</td>
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

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter:

- setup a winch as described on the :ref:`Adding Simulated Devices page <adding_simulated_devices>`
- module load message
- module load graph
- graph SERVO_OUTPUT_RAW.servo9_raw

+------------------------------------------------------+------------------------------------------------+
| Example MAVProxy/SITL Command                        | Description                                    |
+======================================================+================================================+
| ``message COMMAND_LONG 0 0 42600 0 0 0 0 0 0 0 0``   | Relax winch so line can be pulled out manually |
+------------------------------------------------------+------------------------------------------------+
| ``message COMMAND_LONG 0 0 42600 0 0 1 5 0 0 0 0``   | Let out 5m of line                             |
+------------------------------------------------------+------------------------------------------------+
| ``message COMMAND_LONG 0 0 42600 0 0 2 0 0.5 0 0 0`` | Let out line at 0.5 m/s                        |
+------------------------------------------------------+------------------------------------------------+
