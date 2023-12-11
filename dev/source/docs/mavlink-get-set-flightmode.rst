.. _mavlink-get-set-flightmode:

======================
Get and Set FlightMode
======================

This page explains how MAVLink can be used by a ground station or companion computer to get or set the vehicle's flightmode.  The user wiki pages for flightmodes and links to the parameter with the flightmode's numbers are listed below

- :ref:`Copter flight modes <copter:flight-modes>`, see :ref:`FLTMODE1 <copter:FLTMODE1>` for flightmode numbers
- :ref:`Plane flight modes <plane:flight-modes>`, see :ref:`FLTMODE1 <plane:FLTMODE1>` for flightmode numbers
- :ref:`Rover flight modes <rover:rover-control-modes>`, see :ref:`MODE1 <rover:MODE1>` for flightmode numbers

The MAVLink enums for flight modes can be found `here <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml#L1007>`__

Get the Flightmode with HEARTBEAT
---------------------------------

The vehicle's current flight mode is sent once per second within the `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`__ message's custom_mode field.  The flightmode number varies by vehicle type (e.g. Copter, Plane, Rover, etc) so please refer to the links above to convert the custom_mode number to a human readable flightmode name.

Set the Flightmode with MAV_CMD_DO_SET_MODE
--------------------------------------------

Attempt to set the vehicle's flightmode by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param2 fields set as specified for the `MAV_CMD_DO_SET_MODE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>`__ command.

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
   <td>MAV_CMD_DO_SET_MODE=176</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>flightmode number (see FLTMODE1 links above)</td>
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
| ``message COMMAND_LONG 0 0 176 0 1 6 0 0 0 0 0``     | Copter: change to RTL mode (6)                      |
|                                                      | Plane: change to FBWB mode (6)                      |
|                                                      | Rover: change to Follow mode (6)                    |
+------------------------------------------------------+-----------------------------------------------------+
