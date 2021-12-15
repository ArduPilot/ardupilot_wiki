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

.. _mavlink-get-set-home-and-origin_set_home_position:

SET_HOME_POSITION
-----------------

Sets the location that the vehicle will return to and land on when in RTL mode. The location is normally set automatically each time the vehicle is armed if it has not already been explicitly set by the operator.

The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_HOME_POSITION>`__

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

.. _mavlink-get-set-home-and-origin_set_gps_global_origin:

SET_GPS_GLOBAL_ORIGIN
---------------------

Sets the location used by the EKF/AHRS for internal calculations.  This location is normally automatically set soon after the GPS first returns a good location.  The operator may be required to set this manually if Non-GPS navigation is used.  Once set the EKF origin cannot be moved.

The message definition can be found `here <https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN>`__

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

The example command below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter, "module load message"

- module load message
- param set EK3_SRC1_POSXY 0
- param set EK3_SRC1_VELXY 0
- param set EK3_SRC1_VELZ 0

+-----------------------------------------------------------------+---------------------------------------------------------------------+
| Example MAVProxy/SITL Command                                   | Description                                                         |
+=================================================================+=====================================================================+
| ``message SET_GPS_GLOBAL_ORIGIN 0 -353621474 1491651746 600 0`` | set EKF origin to lat,lon of -35.36,149.16 and 600m above sea level |
+-----------------------------------------------------------------+---------------------------------------------------------------------+
