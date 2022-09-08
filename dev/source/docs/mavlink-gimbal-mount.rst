.. _mavlink-gimbal-mount:

===============================
Control a Gimbal / Camera Mount
===============================

This page explains how MAVLink can be used to control a gimbal (aka camera mount).  In particular this page describes how to

- use MAV_CMD_DO_MOUNT_CONTROL to set the gimbal's mode (aka mount mode)
- use MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW to move to a desired angle or at a desired rate
- use MAV_CMD_DO_SET_ROI_LOCATION to point at a Location
- use MAV_CMD_DO_SET_ROI_SYSID to point at another vehicle

The gimbal's attitude (in body-frame Quaternion form) can be monitored by decoding the `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`__ message.

.. note::

    ArduPilot's MAVLink interface for controlling gimbals was significantly upgraded for 4.3 compared with earlier versions. This page primarily discusses the interface for 4.3 (and higher) which aims to comply with the `MAVLink Gimbalv2 protocol <https://mavlink.io/en/services/gimbal_v2.html>`__

.. note::

    The :ref:`user wiki pages for gimbals is here <copter:common-cameras-and-gimbals>`.

Set the mode with MAV_CMD_DO_MOUNT_CONTROL
------------------------------------------

The gimbal/mount's mode can be set by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param7 fields set as specified for the `MAV_CMD_DO_MOUNT_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL>`__ command.
The supported modes are:

0. Retract : gimbal will move to a retracted position (see MNT1_RETRACT_X, Y, Z) or relax
1. Neutral : gimbal will move to a neutral position (see MNT1_NEUTRAL_X, Y, Z)
2. MAVLink Targeting : gimbal's angle or rate targets are controlled via mavlink messages (see MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW).  It is not necessary to specifically change to this mode, this will happen automatically if MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW is sent
3. RC Targeting : pilot controls the gimbal angle or rate using RC input (see MNT_RC_IN_PAN/ROLL/TILT and MNT_RC_RATE parameters)
4. GPS Point: gimbal points towards a Location (Lat, Lon, Alt) (see MAV_CMD_DO_SET_ROI).  It is not necessary to specifically change to this mode, this will happen automatically if MAV_CMD_DO_SET_ROI is sent
5. SysId Target: gimbal points towards another vehicle (see MAV_CMD_DO_SET_ROI_SYSID).  It is not necessary to specifically change to this mode, this will happen automatically if MAV_CMD_DO_SET_ROI_SYSID is sent
6. Home Location: gimbal points towards home

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
   <td>MAV_CMD_DO_MOUNT_CONTROL=205</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Pitch in degrees (or zero)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Roll in degrees (or zero)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Yaw in degrees (or zero)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>Altitude in meters (or zero)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>Longitude in degrees * 1E7 (or zero)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>Latitude in degrees * 1E7 (or zero)</td>
   </tr>
   <tr>
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>Mode (0=Retract, 1=Neutral, 2=Mavlink Targeting, 3=RC Targeting, 4=GPS Point, 5=SysId Target, 6=Home Location)</td>
   </tr>
   </tbody>
   </table>

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter

- module load message

+------------------------------------------------------+---------------------------------+
| Example MAVProxy/SITL Command                        | Description                     |
+======================================================+=================================+
| ``message COMMAND_LONG 0 0 205 0 0 0 0 0 0 0 0``     | Retract Gimbal                  |
+------------------------------------------------------+---------------------------------+
| ``message COMMAND_LONG 0 0 205 0 0 0 0 0 0 0 3``     | Switch to RC Targeting          |
|                                                      | (Pilot controls gimbal from RC) |
+------------------------------------------------------+---------------------------------+
| ``message COMMAND_LONG 0 0 205 0 0 0 0 0 0 0 5``     | Point gimbal at another vehicle |
|                                                      | see MAV_CMD_DO_SET_ROI_SYSID    |
+------------------------------------------------------+---------------------------------+
| ``message COMMAND_LONG 0 0 205 0 0 0 0 0 0 0 6``     | Point gimbal at home            |
+------------------------------------------------------+---------------------------------+

MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW to move to a desired angle or at a desired rate
----------------------------------------------------------------------------------

The gimbal's attitude can be changed to a desired pitch and yaw angle or changed at a desired rate by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the
command and param1 through param7 fields set as specified for the `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW>`__ command.

The gimbal's yaw behaviour as the vehicle rotates can also be controlled.  The two behaviour are:

- body-frame/follow means the gimbal's yaw will rotate with the vehicle
- earth-frame / lock means the gimbal's yaw will remain fixed and will not rotate with the vehicle

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
   <td>MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW=1000</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Pitch angle in deg (positive is up) or NaN if unused</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Yaw angle in deg (positive is clockwise) or NaN if unused</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Pitch rate in deg/s (positive is up) or NaN if unused</td>
   </tr>
   <tr>
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>Yaw rate in deg/s (positive is clockwise) or NaN if unused</td>
   </tr>
   <tr>
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>Flags (0=Yaw is body-frame/follow, 16=Yaw is earth-frame/lock)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr>
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>Gimbal device ID (0 is primary gimbal, 1 is 1st gimbal, 2 is 2nd gimbal)</td>
   </tr>
   </tbody>
   </table>

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter:

- module load message

+-----------------------------------------------------------------------------+-----------------------------------------------------------+
| Example MAVProxy/SITL Command                                               | Description                                               |
+=============================================================================+===========================================================+
| ``message COMMAND_LONG 0 0 1000 0 -20 90 float("NaN") float("NaN") 0 0 0``  | Pitch down 20deg, yaw right 90 deg, body-frame / follow   |
+-----------------------------------------------------------------------------+-----------------------------------------------------------+
| ``message COMMAND_LONG 0 0 1000 0 -20 90 float("NaN") float("NaN") 16 0 0`` | Pitch down 20deg, yaw East, earth-frame / lock            |
+-----------------------------------------------------------------------------+-----------------------------------------------------------+
| ``message COMMAND_LONG 0 0 1000 0 float("NaN") float("NaN") 5 0 0 0 0``     | Pitch down at 5deg/sec, yaw hold, body-frame / follow     |
+-----------------------------------------------------------------------------+-----------------------------------------------------------+
| ``message COMMAND_LONG 0 0 1000 0 float("NaN") float("NaN") 0 5 0 0 0``     | Pitch hold, yaw clockwise at 5deg/sec in body-frame       |
+-----------------------------------------------------------------------------+-----------------------------------------------------------+
| ``message COMMAND_LONG 0 0 1000 0 float("NaN") float("NaN") 0 5 16 0 0``    | Pitch hold, yaw clockwise at 5deg/sec in eartj-frame      |
+-----------------------------------------------------------------------------+-----------------------------------------------------------+

MAV_CMD_DO_SET_ROI_LOCATION to point at a Location
--------------------------------------------------

The gimbal can be pointed at a Location (Lat, Lon, Alt) by sending a `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`__ with the
command and param1 through param6 fields set as specified for the `MAV_CMD_DO_SET_ROI_LOCATION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION>`__ command.

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
   <td><strong>frame</strong></td>
   <td>uint8_t</td>
   <td>

Valid options are:

- MAV_FRAME_GLOBAL (0): alt is meters above sea level
- MAV_FRAME_GLOBAL_INT (5): alt is meters above sea level
- MAV_FRAME_GLOBAL_RELATIVE_ALT (3): alt is meters above home
- MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (6): alt is meters above home
- MAV_FRAME_GLOBAL_TERRAIN_ALT (10): alt is meters above terrain
- MAV_FRAME_GLOBAL_TERRAIN_ALT_INT (11): alt is meters above terrain

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>command</strong></td>
   <td>uint16_t</td>
   <td>MAV_CMD_DO_SET_ROI_LOCATION=195</td>
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
   <tr style="color: #c0c0c0">
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Gimbal device id (unused)</td>
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

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter:

- module load message

+---------------------------------------------------------------------------+------------------------------------------------------------------+
| Example MAVProxy/SITL Command                                             | Description                                                      |
+===========================================================================+==================================================================+
| ``message COMMAND_INT 0 0 6 195 0 0 0 0 0 0 -353632632 1491663846 10``    | Point at Lat:-35.3632632 Lon:149.1663846 Alt:10m above home      |
+---------------------------------------------------------------------------+------------------------------------------------------------------+
| ``message COMMAND_INT 0 0 0 195 0 0 0 0 0 0 -353632632 1491663846 10``    | Point at Lat:-35.3632632 Lon:149.1663846 Alt:10m above sea level |
+---------------------------------------------------------------------------+------------------------------------------------------------------+
| ``message COMMAND_INT 0 0 11 195 0 0 0 0 0 0 -353632632 1491663846 10``   | Point at Lat:-35.3632632 Lon:149.1663846 Alt:10m above terrain   |
+---------------------------------------------------------------------------+------------------------------------------------------------------+

MAV_CMD_DO_SET_ROI_SYSID to point at another vehicle
----------------------------------------------------

The gimbal can be pointed at another vehicle by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the
command and param1 fields set as specified for the `MAV_CMD_DO_SET_ROI_SYSID <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID>`__ command.

This feature relies on the main vehicle receiving the other vehicle's position at regular intervals via the `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`__ message.

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
   <td>MAV_CMD_DO_SET_ROI_SYSID=198</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>System ID of other vehicle</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Gimbal device id (unused)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>unused</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>unused</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>unused</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>unused</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>unused</td>
   </tr>
   </tbody>
   </table>

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter:

- module load message

+---------------------------------------------------+-------------------------------+
| Example MAVProxy/SITL Command                     | Description                   |
+===================================================+===============================+
| ``message COMMAND_LONG 0 0 198 0 2 0 0 0 0 0 0``  | Point at vehicle with SysId=2 |
+---------------------------------------------------+-------------------------------+
