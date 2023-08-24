.. _mavlink-camera:

================
Control a Camera
================

This page explains how MAVLink can be used to control a camera.  These commands are supported in all versions of ArduPilot:

- MAV_CMD_DO_DIGICAM_CONTROL to take a picture
- MAV_CMD_DO_SET_CAM_TRIGG_DIST to take pictures at regular distance intervals

These commands are supported in ArduPilot 4.4.0 and higher:

- MAV_CMD_IMAGE_START_CAPTURE to take a picture
- MAV_CMD_SET_CAMERA_ZOOM to control the camera zoom
- MAV_CMD_SET_CAMERA_FOCUS to manually or automatically focus the camera
- MAV_CMD_VIDEO_START_CAPTURE to start recording video
- MAV_CMD_VIDEO_STOP_CAPTURE to stop recording a video

These commands are supported in ArduPilot 4.5.0 and higher:

- MAV_CMD_CAMERA_TRACK_POINT to initiate tracking of a point on the video feed
- MAV_CMD_CAMERA_TRACK_RECTANGLE to initiate tracking of a rectangle on the video feed
- MAV_CMD_CAMERA_STOP_TRACKING to stop tracking
- CAMERA_INFORMATION includes vendor and model name, firmware version, etc for use by GCS
- CAMERA_SETTINGS includes zoom and focus level for use by GCS

These commands and messages are not yet supported but may be in future releases

- MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS
- MAV_CMD_RESET_CAMERA_SETTINGS
- MAV_CMD_SET_CAMERA_MODE
- MAV_CMD_IMAGE_STOP_CAPTURE
- MAV_CMD_DO_TRIGGER_CONTROL
- MAV_CMD_VIDEO_START_STREAMING and MAV_CMD_VIDEO_STOP_STREAMING to start and stop streaming a video to the ground station
- MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION
- MAV_CMD_REQUEST_VIDEO_STREAM_STATUS
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
| ``message COMMAND_LONG 0 0 203 0 0 0 0 0 1 0 0``     | Take a picture                  |
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
| ``message COMMAND_LONG 0 0 206 0 0 0 0 0 0 0 0``   | Stop taking pictures at regular intervals   |
+----------------------------------------------------+---------------------------------------------+

MAV_CMD_IMAGE_START_CAPTURE to take a picture
---------------------------------------------

A picture can be taken by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param3 fields set as specified for the `MAV_CMD_IMAGE_START_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE>`__ command.

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
   <td>MAV_CMD_IMAGE_START_CAPTURE=2000</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>unused</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Interval=0 (unsupported)</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Total Images=1 (multiple images not supported)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>Sequence Number (unsupported)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>Sequence Number (unsupported)</td>
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

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter

- module load message

+------------------------------------------------------+---------------------------------+
| Example MAVProxy/SITL Command                        | Description                     |
+======================================================+=================================+
| ``message COMMAND_LONG 0 0 2000 0 0 0 1 0 0 0 0``    | Take a picture                  |
+------------------------------------------------------+---------------------------------+

MAV_CMD_SET_CAMERA_ZOOM to control the camera zoom
--------------------------------------------------

The camera zoom can be controlled by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param fields set as specified for the `MAV_CMD_SET_CAMERA_ZOOM <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_ZOOM>`__ command.

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
   <td>MAV_CMD_SET_CAMERA_ZOOM=531</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Zoom Type=1 (step=0, continous=1, range=2, focal length=3)</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Zoom Value (zoom in=1, zoom out=-1, stop=0)</td>
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

+----------------------------------------------------+---------------------------------------------+
| Example MAVProxy/SITL Command                      | Description                                 |
+====================================================+=============================================+
| ``message COMMAND_LONG 0 0 531 0 1 1 0 0 0 0 0``   | Zoom in                                     |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 531 0 1 -1 0 0 0 0 0``  | Zoom out                                    |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 531 0 1 0 0 0 0 0 0``   | Stop zooming in or out                      |
+----------------------------------------------------+---------------------------------------------+

MAV_CMD_SET_CAMERA_FOCUS to manually or automatically focus the camera
----------------------------------------------------------------------

The camera zoom can be controlled by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param fields set as specified for the `MAV_CMD_SET_CAMERA_FOCUS <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_FOCUS>`__ command.

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
   <td>MAV_CMD_SET_CAMERA_FOCUS=532</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Focus Type=1 OR 4 (step=0, continous=1, range=2, meters=3, auto=4, auto single=5, auto continuous=6)</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Focus Value (focus in=-1, focus out=1, hold=0)</td>
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

+----------------------------------------------------+---------------------------------------------+
| Example MAVProxy/SITL Command                      | Description                                 |
+====================================================+=============================================+
| ``message COMMAND_LONG 0 0 532 0 1 -1 0 0 0 0 0``  | Manual focus in                             |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 532 0 1 1 0 0 0 0 0``   | Manual focus out                            |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 532 0 1 0 0 0 0 0 0``   | Manual focus hold                           |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 532 0 4 0 0 0 0 0 0``   | Auto focus                                  |
+----------------------------------------------------+---------------------------------------------+

MAV_CMD_VIDEO_START_CAPTURE, MAV_CMD_VIDEO_STOP_CAPTURE to start or stop recording video
----------------------------------------------------------------------------------------

To start or stop recording video send a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param fields set as specified for the `MAV_CMD_VIDEO_START_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_START_CAPTURE>`__ or `MAV_CMD_VIDEO_STOP_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_STOP_CAPTURE>`__ commands.

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
   <td>MAV_CMD_VIDEO_START_CAPTURE=2500, MAV_CMD_VIDEO_STOP_CAPTURE=2501</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Stream ID (All=0, 1st camera=1, 2nd camera=2)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Status Frequency (unused)</td>
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

+----------------------------------------------------+---------------------------------------------+
| Example MAVProxy/SITL Command                      | Description                                 |
+====================================================+=============================================+
| ``message COMMAND_LONG 0 0 2500 0 0 0 0 0 0 0 0``  | Start recording video on all cameras        |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 2500 0 1 0 0 0 0 0 0``  | Start recording video on 1st camera         |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 2501 0 0 1 0 0 0 0 0``  | Stop recording video on all cameras         |
+----------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 0 0 2501 0 1 0 0 0 0 0 0``  | Stop recording video on 1st camera          |
+----------------------------------------------------+---------------------------------------------+

MAV_CMD_CAMERA_TRACK_POINT to start tracking a point on the live video stream
-----------------------------------------------------------------------------

To start tracking a point on the live video stream send a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param fields set as specified for the `MAV_CMD_CAMERA_TRACK_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT>`__ commands.  The `MAV_CMD_CAMERA_STOP_TRACKING <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING>`__ can be used to stop tracking.

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
   <td>MAV_CMD_CAMERA_TRACK_POINT=2004</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Point X (0 to 1, 0 is left, 1 is right)</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Point Y (0 to 1, 0 is top, 1 is bottom)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Radius (unused)</td>
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

+--------------------------------------------------------+----------------------------------------------------------+
| Example MAVProxy/SITL Command                          | Description                                              |
+========================================================+==========================================================+
| ``message COMMAND_LONG 0 0 2004 0 0.5 0.5 0 0 0 0 0``  | Start tracking a point on the middle of the video stream |
+--------------------------------------------------------+----------------------------------------------------------+
| ``message COMMAND_LONG 0 0 2010 0 0 0 0 0 0 0 0``      | Stop tracking                                            |
+--------------------------------------------------------+----------------------------------------------------------+

MAV_CMD_CAMERA_TRACK_RECTANGLE to start tracking a rectangle on the live video stream
-------------------------------------------------------------------------------------

To start tracking a rectangle on the live video stream send a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command and param fields set as specified for the `MAV_CMD_CAMERA_TRACK_RECTANGLE <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE>`__ commands.  The `MAV_CMD_CAMERA_STOP_TRACKING <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING>`__ can be used to stop tracking.

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
   <td>MAV_CMD_CAMERA_TRACK_RECTANGLE=2005</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Top Left X (0 to 1, 0 is left, 1 is right)</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>Point Y (0 to 1, 0 is top, 1 is bottom)</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Bottom Right X (0 to 1, 0 is left, 1 is right)</td>
   </tr>
   <tr>
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>Bottom Right Y (0 to 1, 0 is top, 1 is bottom)</td>
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

+-----------------------------------------------------------+--------------------------------------------------------------+
| Example MAVProxy/SITL Command                             | Description                                                  |
+===========================================================+==============================================================+
| ``message COMMAND_LONG 0 0 2005 0 0.4 0.4 0.6 0.6 0 0 0`` | Start tracking a rectangle on the middle of the video stream |
+-----------------------------------------------------------+--------------------------------------------------------------+
| ``message COMMAND_LONG 0 0 2005 0 0.4 0.0 0.6 0.1 0 0 0`` | Start tracking a rectangle in the top middle                 |
+-----------------------------------------------------------+--------------------------------------------------------------+
| ``message COMMAND_LONG 0 0 2010 0 0 0 0 0 0 0 0``         | Stop tracking                                                |
+-----------------------------------------------------------+--------------------------------------------------------------+

CAMERA_INFORMATION and CAMERA_SETTINGS include information useful for GCSs
--------------------------------------------------------------------------

These two messages include information that can be useful for the ground station.  The `CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION>`__ includes vendor and model name, firmware version, sensor size, sensor resolution and camera capabilities.  The `CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#CAMERA_SETTINGS>`__ is much simpler and only includes the mode, zoom level and focus level.

A ground station can request the messages be sent (just once) using the `MAV_CMD_REQUEST_MESSAGE  <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE>`__ command as described on the :ref:`Requesting Data From The Autopilot <mavlink-requesting-data>` page.

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter:

- module load message

+----------------------------------------------------+-----------------------------------------------------+
| Example MAVProxy/SITL Command                      | Description                                         |
+====================================================+=====================================================+
| ``message COMMAND_LONG 0 0 512 0 259 0 0 0 0 0 0`` | Request the CAMERA_INFORMATION message be sent once |
+----------------------------------------------------+-----------------------------------------------------+
| ``message COMMAND_LONG 0 0 512 0 260 0 0 0 0 0 0`` | Request the CAMERA_SETTINGS message be sent once    |
+----------------------------------------------------+-----------------------------------------------------+
