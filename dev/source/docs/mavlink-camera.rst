.. _mavlink-camera:

================
Control a Camera
================

This page explains how MAVLink can be used to control a camera.  In particular this page describes these commands

- MAV_CMD_DO_DIGICAM_CONTROL to take a picture
- MAV_CMD_DO_SET_CAM_TRIGG_DIST to take pictures at regular distance intervals

These commands and messages are not yet supported but may be in future releases

- MAV_CMD_IMAGE_START_CAPTURE to take a picture
- MAV_CMD_VIDEO_START_CAPTURE to start recording video
- MAV_CMD_VIDEO_STOP_CAPTURE to stop recording a video
- MAV_CMD_SET_CAMERA_ZOOM to control the camera zoom
- MAV_CMD_SET_CAMERA_FOCUS to manually or automatically focus the camera
- MAV_CMD_REQUEST_CAMERA_INFORMATION
- MAV_CMD_REQUEST_CAMERA_SETTINGS
- MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS
- MAV_CMD_RESET_CAMERA_SETTINGS
- MAV_CMD_SET_CAMERA_MODE
- MAV_CMD_IMAGE_STOP_CAPTURE
- MAV_CMD_DO_TRIGGER_CONTROL
- MAV_CMD_CAMERA_TRACK_POINT to initiate tracking of a point on the video feed
- MAV_CMD_CAMERA_TRACK_RECTANGLE to initiate tracking of a rectangle on the video feed
- MAV_CMD_CAMERA_STOP_TRACKING to stop tracking
- MAV_CMD_VIDEO_START_STREAMING and MAV_CMD_VIDEO_STOP_STREAMING to start and stop streaming a video to the ground station
- MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION
- MAV_CMD_REQUEST_VIDEO_STREAM_STATUS
- CAMERA_INFORMATION
- CAMERA_SETTINGS
- CAMERA_CAPTURE_STATUS
- CAMERA_IMAGE_CAPTURED
- CAMERA_FOV_STATUS
- CAMERA_TRACKING_IMAGE_STATUS
- CAMERA_TRACKING_GEO_STATUS

.. note::

    ArduPilot's MAVLink interface for controlling cameras was upgraded for 4.4 compared with earlier versions. This page primarily discusses the interface for 4.4 (and higher) which aims to comply with the `MAVLink Camera protocol <https://mavlink.io/en/services/camera.html>`__

.. note::

    The :ref:`user wiki pages for cameras and gimbals is here <copter:common-cameras-and-gimbals>`.

MAV_CMD_DO_DIGICAM_CONTROL to take a picture
--------------------------------------------

A picture can be taken by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param5 fields set as specified for the `MAV_CMD_DO_DIGICAM_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_DIGICAM_CONTROL>`__ command.

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
   <td>MAV_CMD_DO_DIGICAM_CONTROL=203</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Session Control (unused)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Zoom Absolute (unused)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Zoom Relative (unused)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>Focus (unused)</td>
   </tr>
   <tr>
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>Shoot Command=1</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>Command Identify (unused)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>Shot ID (unused)</td>
   </tr>
   </tbody>
   </table>

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter

- module load message

+------------------------------------------------------+---------------------------------+
| Example MAVProxy/SITL Command                        | Description                     |
+======================================================+=================================+
| ``message COMMAND_LONG 0 0 203 0 0 0 0 0 0 0 1``     | Take a picture                  |
+------------------------------------------------------+---------------------------------+

MAV_CMD_DO_SET_CAM_TRIGG_DIST to take a picture at regular distance intervals
-----------------------------------------------------------------------------

A picture can be taken at regular distance intervals by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param fields set as specified for the `MAV_CMD_DO_SET_CAM_TRIGG_DIST <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_DIST>`__ command.

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
   <td>MAV_CMD_DO_SET_CAM_TRIGG_DIST=206</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Distance in meters or 0 to stop triggering</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Shutter (unused)</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Trigger camera once immediately. (0 = no trigger now, 1 = trigger now)</td>
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

+----------------------------------------------------+---------------------------------------------+
| Example MAVProxy/SITL Command                      | Description                                 |
+====================================================+=============================================+
| ``message COMMAND_LONG 0 0 206 0 10 0 0 0 0 0 0``  | Take a picture every 10m                    |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 206 0 10 0 1 0 0 0 0``  | Take a picture now and then again every 10m |
+----------------------------------------------------+---------------------------------------------+
