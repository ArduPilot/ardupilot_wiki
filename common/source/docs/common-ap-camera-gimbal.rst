.. _common-ap-camera-gimbal:

[copywiki destination="plane,copter,rover,sub"]

================
AP Camera Gimbal
================

.. image:: https://raw.githubusercontent.com/ArduPilot/AP_CameraGimbal/master/assets/camera-gimbal.svg
    :target: https://github.com/ArduPilot/AP_CameraGimbal
    :width: 150px

`AP Camera Gimbal <https://github.com/ArduPilot/AP_CameraGimbal/blob/master/README.md>`__ is open source software that can be installed and run on these camera gimbals:

- :ref:`Siyi camera gimbals <common-siyi-zr10-gimbal>`: MT11, A8 mini, ZR10 and ZT30 (ZT30 supports application updates only and needs the AP launchers already installed)
- :ref:`XFRobot Z-1 mini <common-xfrobot-gimbal>`
- RPi CM4 paired with :ref:`XFRobot C-20T <common-xfrobot-gimbal>` and Sony IMX477 connected via CSI/MIPI

This software has most (but perhaps not all) of the same features as the manufacturer's original software but also includes:

- Recorded video files include the vehicle position and gimbal attitude which allows the `Video Review Tool <https://plot.ardupilot.org/video/>`__ to be used to display side-by-side video and map display
- `Splat Generator <https://plot.ardupilot.org/splat/>`__ allows converting the recorded video into a 3D Gaussian splat
- Live video streaming to the `Support Proxy <https://github.com/ArduPilot/SupportProxy>`__ server

How to Install
--------------

For first time installations:

- Download the appropriate firmware for your camera gimbal from `firmware.ardupilot.org <https://firmware.ardupilot.org/Tools/AP_CameraGimbal/release/>`__
- Follow :ref:`Siyi <common-siyi-zr10-gimbal>` or :ref:`XFRobot <common-xfrobot-gimbal>` firmware update instructions but use the firmware downloaded above. Each camera's folder on `firmware.ardupilot.org <https://firmware.ardupilot.org/Tools/AP_CameraGimbal/release/>`__ includes a README.md with install and revert instructions for that camera
- RPi CM4 users should instead follow the `RPi CM4 instructions <https://github.com/ArduPilot/AP_CameraGimbal/blob/master/packaging/rpi_libcam_caddx/README.md>`__. The software is built on the Pi, MAVLink is routed from the autopilot to UDP port 16001 using Rpanion and Rpanion's video streaming must be left stopped

Subsequent upgrades can be done using AP_CameraGimbal's built-in firmware update web page (not available on the RPi CM4).

The camera gimbal can be reverted to the manufacturer's original software by following the :ref:`Siyi <common-siyi-zr10-gimbal>` or :ref:`XFRobot <common-xfrobot-gimbal>` firmware update instructions.

Configuration
-------------

The autopilot should be physically connected following the regular :ref:`Siyi <common-siyi-zr10-gimbal>` or :ref:`XFRobot <common-xfrobot-gimbal>` instructions.

Set the parameters below assuming the autopilot's Serial2 port is used and the Camera1 control instance. A serial connection can be used with the Siyi A8 mini, MT11 and ZR10 but not the XFRobot Z-1 mini (AP_CameraGimbal supports only Ethernet on this gimbal, even though the manufacturer's software also supports serial) or the RPi CM4 (see the RPi instructions above). ZT30 serial MAVLink is untested:

- :ref:`CAM1_TYPE <CAM1_TYPE>` = 6 (MAVLinkCamV2)
- :ref:`MNT1_TYPE <MNT1_TYPE>` = 6 (MAVLink)
- :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 2 (MAVLink2) instead of 8 (Gimbal)
- :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 115 (115200 bps)
- Reboot the autopilot

Alternatively if the camera gimbal is connected to the autopilot over Ethernet:

- :ref:`CAM1_TYPE <CAM1_TYPE>` = 6 (MAVLinkCamV2)
- :ref:`MNT1_TYPE <MNT1_TYPE>` = 6 (MAVLink)
- :ref:`NET_ENABLE <NET_ENABLE>` = 1 and reboot the autopilot
- :ref:`NET_P1_TYPE <NET_P1_TYPE>` = 1 (UDP Client) and reboot the autopilot
- Set :ref:`NET_P1_IP0 <NET_P1_IP0>`, :ref:`NET_P1_IP1 <NET_P1_IP1>`, :ref:`NET_P1_IP2 <NET_P1_IP2>`, :ref:`NET_P1_IP3 <NET_P1_IP3>` to the gimbal's IP address (e.g. 192.168.144.25)
- :ref:`NET_P1_PORT <NET_P1_PORT>` = 16001 (AP_CameraGimbal's default MAVLink port). This should match the MAVLink port shown on the camera's Parameters web page.
- :ref:`NET_P1_PROTOCOL <NET_P1_PROTOCOL>` = 2 (MAVLink2) instead of 8 (Gimbal)
- Reboot the autopilot

.. note:: On Plane 4.7, set :ref:`SCHED_LOOP_RATE <SCHED_LOOP_RATE>` to 100 or higher or the gimbal will not be detected (the autopilot cannot send it attitude updates at 50Hz at Plane's default 50Hz loop rate). QuadPlane already defaults to 300.

Live Video
----------

The video is available using the RTSP URLs shown below.  These URLs assume the camera has been configured with IP address 192.168.144.25

- rtsp://192.168.144.25:8554/video1 (normally RGB)
- rtsp://192.168.144.25:8554/video2 (secondary stream, thermal only on thermal-capable models such as the MT11 and ZT30. Not available on the RPi CM4 which has video1 only)

If connected to a PC, `VLC <https://www.videolan.org/>`__ can be used to test the feed:

- Open VLC
- Select "Media", "Open Network Stream" and enter the RTSP URL

.. image:: ../../../images/siyi-a8-vlc.png
    :target: ../_images/siyi-a8-vlc.png

If using QGC, the live video can be configured from the "Application Settings", "General" screen.  Set "Source" to "RTSP Video Stream" and "RTSP URL" to one of the RTSP URLs listed above.

.. image:: ../../../images/siyi-a8-qgc.png
    :target: ../_images/siyi-a8-qgc.png

If using Mission Planner, the live video can be displayed on the Data screen by following :ref:`these instructions <planner:live-video>`.

Control and Testing
-------------------

See :ref:`Gimbal / Mount Controls <common-mount-targeting>` and :ref:`Camera Controls <common-camera-controls>` for details on how to control the camera and gimbal using RC, GCS or Auto mode mission commands.
