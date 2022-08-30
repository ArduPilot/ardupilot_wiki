.. _mavlink-get-set-home-and-origin:

==============================
Setting Home and/or EKF origin
==============================

This page explains how MAVLink can be used by a ground station or companion computer to get or set the home or EKF origin.

Home vs EKF Origin
------------------

The vehicles "Home" position is the location (specified as a latitude, longitude and altitude above sea level) that the vehicle will return to in :ref:`RTL <copter:rtl-mode>` mode.  For most vehicles this location is set to the vehicle's current location each time the vehicle is armed but it may also be moved at any time.  Moving home can be useful in situations where the user wishes RTL mode to return the vehicle to a different location than it took off from.

The "EKF origin" is the location that the EKF (aka AHRS) uses for internal calculations.  This location is normally set to the vehicle's location soon after the GPS provides a good quality location.  Once set the EKF origin cannot be moved.  Users are not normally aware of the EKF origin's location unless they are using Non-GPS navigation on a vehicle with no GPS attached (without a GPS, the user must specify the EKF origin using the ground station).

Whenever the Home or EKF origin is updated the vehicle will send a `HOME_POSITION <https://mavlink.io/en/messages/common.html#HOME_POSITION>`__ or `GPS_GLOBAL_ORIGIN <https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN>`__ message (respectively) on all active mavlink channels.

The home will also be sent in response to a `MAV_CMD_GET_HOME_POSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_GET_HOME_POSITION>`__ sent within a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ or `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`__ message.

.. _mavlink-get-set-home-and-origin_set_home_position:

MAV_CMD_DO_SET_HOME within COMMAND_INT
--------------------------------------

Set the home location by sending a `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`__ with the command and parameter fields set as specified for the `MAV_CMD_DO_SET_HOME <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME>`__ command.

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
   <td><strong>frame</strong></td>
   <td>uint8_t</td>
   <td>MAV_FRAME_GLOBAL=0</td>
   </tr>
   <tr>
   <td><strong>command</strong></td>
   <td>uint16_t</td>
   <td>MAV_CMD_DO_SET_HOME=179</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>current</strong></td>
   <td>uint8_t</td>
   <td>0 (not used)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>autocontinue</strong></td>
   <td>uint8_t</td>
   <td>0 (not used)</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>1=use current location, 0=use specified location</td>
   </tr>
   <tr>
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
   <td><strong>param5</strong></td>
   <td>int32_t</td>
   <td>Latitude in degrees * 10^7</td>
   </tr>
   <td><strong>param6</strong></td>
   <td>int32_t</td>
   <td>Longitude in degrees * 10^7</td>
   </tr>
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>Altitude in meters</td>
   </tr>
   </tbody>
   </table>

**Examples**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter, "module load message"

+-------------------------------------------------------------------------+--------------------------------------------+
| Example MAVProxy/SITL Command                                           | Description                                |
+=========================================================================+============================================+
| ``message COMMAND_INT 0 0 0 179 0 0 1 0 0 0 0 0 0``                     | set home to the vehicle's current location |
+-------------------------------------------------------------------------+--------------------------------------------+
| ``message COMMAND_INT 0 0 0 179 0 0 0 0 0 0 -353630000 1491650000 575`` | set home to the specified location         |
+-------------------------------------------------------------------------+--------------------------------------------+

MAV_CMD_DO_SET_HOME within COMMAND_LONG
---------------------------------------

Set the home location by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and parameter fields set as specified for the `MAV_CMD_DO_SET_HOME <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME>`__ command.  Note that this method sets the home position with less accuracy than the COMMAND_INT method from above.

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
   <td>MAV_CMD_DO_SET_HOME=179</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>1=use current location, 0=use specified location</td>
   </tr>
   <tr>
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
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>Latitude in degrees</td>
   </tr>
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>Longitude in degrees</td>
   </tr>
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>Altitude in meters</td>
   </tr>
   </tbody>
   </table>

**Examples**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter, "module load message"

+-----------------------------------------------------------------------+--------------------------------------------+
| Example MAVProxy/SITL Command                                         | Description                                |
+=======================================================================+============================================+
| ``message COMMAND_LONG 0 0 179 0 1 0 0 0 0 0 0``                      | set home to the vehicle's current location |
+-----------------------------------------------------------------------+--------------------------------------------+
| ``message COMMAND_LONG 0 0 179 0 0 0 0 0 -35.363 149.165 575``        | set home to the specified location         |
+-----------------------------------------------------------------------+--------------------------------------------+

.. _mavlink-get-set-home-and-origin_set_gps_global_origin:

SET_GPS_GLOBAL_ORIGIN
---------------------

Sets the location used by the EKF/AHRS for internal calculations.  This location is normally automatically set soon after the GPS first returns a good location.  The operator may be required to set this manually if Non-GPS navigation is used.  Once set the EKF origin cannot be moved.

The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN>`__

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
   <td>Latitude * 1e7</td>
   </tr>
   <tr>
   <td><strong>longitude</strong></td>
   <td>int32_t</td>
   <td>Longitude * 1e7</td>
   </tr>
   <tr>
   <td><strong>altitude</strong></td>
   <td>int32_t</td>
   <td>Altitude above sea level in millimeters (i.e. meters * 1000)</td>
   </tr>
   <tr>
   <td><strong>time_usec</strong></td>
   <td>uint64_t</td>
   <td>Timestamp (UNIX Epoch time or time since system boot) in microseconds (us)</td>
   </tr>
   </tbody>
   </table>

**Example**

The example command below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- module load message
- param set EK3_SRC1_POSXY 0
- param set EK3_SRC1_VELXY 0
- param set EK3_SRC1_VELZ 0

+--------------------------------------------------------------------+---------------------------------------------------------------------+
| Example MAVProxy/SITL Command                                      | Description                                                         |
+====================================================================+=====================================================================+
| ``message SET_GPS_GLOBAL_ORIGIN 0 -353621474 1491651746 600000 0`` | set EKF origin to lat,lon of -35.36,149.16 and 600m above sea level |
+--------------------------------------------------------------------+---------------------------------------------------------------------+
