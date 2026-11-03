.. _common-camera-controls:

==============================
Camera Controls and Parameters
==============================

This page provides an overview of general camera setup parameters and the three different ways that cameras can be controlled.  The related page detailing :ref:`gimbal controls can be found here <common-mount-targeting>`.

Parameters
==========

ArduPilot supports up to two cameras. The following is for the first camera:

- :ref:`CAM1_TYPE<CAM1_TYPE>`: Camera shutter (trigger) type
- :ref:`CAM1_DURATION<CAM1_DURATION>`: Camera open shutter duration
- :ref:`CAM1_SERVO_ON<CAM1_SERVO_ON>`: Camera servo ON PWM value (Servo type shutter only)
- :ref:`CAM1_SERVO_OFF<CAM1_SERVO_OFF>`: Camera servo OFF PWM value (Servo type shutter only)
- :ref:`CAM1_TRIGG_DIST<CAM1_TRIGG_DIST>`: Camera trigger distance. If this value is non-zero then the camera will trigger whenever the position changes by this number of meters regardless of what mode the autopilot is in.
- :ref:`CAM1_RELAY_ON<CAM1_RELAY_ON>`: Camera relay ON value. This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera(Relay type shutter only).
- :ref:`CAM1_INTRVAL_MIN<CAM1_INTRVAL_MIN>`: Camera minimum time interval between photos
- :ref:`CAM1_MNT_INST<CAM1_MNT_INST>`: If the camera is associated with a MOUNTx instance, this indicates which MOUNTx instance. For example if CAM1 is associated with MOUNT2, then this value will be 2. The default value of 0 for this parameter means that the Mount instance is the same as the Camera instance, ie. CAM1 is in MOUNT1 and is the same as value "1" in this case. This allows Camera commands to be directed to the correct MOUNT instance.
- :ref:`CAM1_OPTIONS<CAM1_OPTIONS>`: if bit 0 set and camera/mount has the ability to start/stop video recording, then it will start on arm and stop on disarm events.
- :ref:`CAM1_COMPID<CAM1_COMPID>`: (ArduPilot 4.8 and later) MAVLink component ID of a MAVLink camera (``CAM1_TYPE`` = 6, "MAVLinkCamV2"). The default of 0 uses component ID 100 for the first camera and 101 for the second. Set a value from 7 to 255 if the camera uses a different component ID. Each MAVLink camera must use a different component ID. A reboot is required after changing this parameter.


.. note:: be sure to set the ``CAMx_INTRVAL_MIN`` to be greater than the fastest the camera can take photos when using the camera trigger functions.

- :ref:`CAM_MAX_ROLL<CAM_MAX_ROLL>`: Maximum photo roll angle. Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
- :ref:`CAM_AUTO_ONLY<CAM_AUTO_ONLY>`: Distance-triggering in AUTO mode only.

MAVLink Cameras
---------------

In ArduPilot 4.8 and later, a camera using the MAVLink camera protocol (``CAMx_TYPE`` = 6) keeps its own MAVLink identity. Its camera information, video streams, capture status and other messages are relayed to ground stations with the camera's own system and component IDs, so a ground station can discover the camera and control it directly, as well as through the autopilot. Only one camera/gimbal unit per MAVLink link is supported; use MAVLink2 on the camera's port. Setting the camera port's ``MAVx_OPTIONS`` to "Unicast" (bit 4, see :ref:`MAVLink channel options <common-serial-options>`) is recommended so that the camera is isolated from other MAVLink traffic while still allowing the ground station full access to it.

Control with an RC transmitter
==============================

RC :ref:`auxiliary functions <common-auxiliary-functions>` allow the pilot to control the camera features using an RC transmitter switch.

- set :ref:`RC6_OPTION <RC6_OPTION>` = 9 ("Camera Trigger") to take a picture
- set :ref:`RC7_OPTION <RC7_OPTION>` = 166 ("Camera Record Video") to start/stop recording video
- set :ref:`RC8_OPTION <RC8_OPTION>` = 167 ("Camera Zoom") to zoom in or out
- set :ref:`RC9_OPTION <RC9_OPTION>` = 168 ("Manual Focus") to focus in or out
- set :ref:`RC10_OPTION <RC10_OPTION>` = 169 ("Auto Focus") to auto focus

Control from a Ground Station
=============================

Ground stations can send MAVLink commands to control the camera.  While each GCS's interface is different below are the controls provided by Mission Planner.

Take a picture using the right-mouse-click menu, select "Trigger Camera NOW"

.. image:: ../../../images/camera-controls-mp-trigger-camera-now.png
    :target: ../_images/camera-controls-mp-trigger-camera-now.png

Use any of the auxiliary function controls listed above from the Data, Aux Functions tab.

.. image:: ../../../images/camera-controls-mp-aux-functions.png
    :target: ../_images/camera-controls-mp-aux-functions.png
    :width: 450px
    
Note that these buttons are "edge triggered" which means that to trigger a function multiple times you may need to push the "Low" or "Mid" button between pushes of "High".

Control during Auto mode missions
=================================

See these pages for details on controlling the camera during Auto mode missions including specifying when the camera shutter should trigger or a distance that the vehicle should travel between shots.

- :ref:`Camera Control in Auto Missions <common-camera-control-and-auto-missions-in-mission-planner>`
- :ref:`Copter Mission Command List <mission-command-list>` 
- :ref:`Mission Commands <common-mavlink-mission-command-messages-mav_cmd>` pages

Control from a Companion Computer or MAVLink
============================================

Cameras and mounts may also be controlled via MAVLink commands from a companion computer or other source.
See :ref:`dev:mavlink-camera`  and :ref:`dev:mavlink-gimbal-mount` documentation.
