.. _common-geotagging-airpixel-tag-e:

================================================================
AirPixel TAG-E for Camera Control and EXIF Geotagging of ILX-LR1
================================================================

.. image:: https://airpixel.cz/wp-content/uploads/2025/07/shop-tage-coin.webp
    :target: https://airpixel.cz/
    :width: 300px

The `TAG-E <https://airpixel.cz/>`_ is a camera controller and geotagger for the Sony ILX-LR1 camera. TAG-E uses MAVLink Camera Protocol v2 to give the ground station access to the camera's features. QGroundControl automatically shows exposure controls, triggering, timelapse and configuration on Android devices and PC/Mac. For Mission Planner there is a plugin for camera and geotagging control.

TAG-E connects to the camera with a single USB-C cable: no WiFi and no additional modules. Geotagging is instant and needs no initialisation. Triggering runs at the camera's full speed, with no slowdown for geotagging. TAG-E also sets the camera's internal clock from the GPS time received over MAVLink, so images always have the correct creation date.

.. image:: https://airpixel.cz/wp-content/uploads/2026/09/QGC-UI-TAG-E.webp
    :width: 100px

Features
========

- Compatible only with Sony ILX-LR1
- Photos are automatically geotagged (via EXIF and XMP) with Lat, Lon, Altitude and camera angles (read from the gimbal)
- Geotags can optionally be extended with GPS time, rangefinder measurement, GPS/IMU accuracy, focal plane distance and resolution, or a custom user label
- HereLink camera control via the QGC UI
- HereLink camera control via MavCam (optional)
- Mission Planner implementation via plugin
- Precise lever arm calculation based on antenna-to-camera offsets
- Geotagging at the *maximum speed of the camera*
- Automatic camera clock configuration from GPS time
- Enhanced file/folder grouping per flight
- Video geotagging by subtitles

Connection and Setup
====================

ArduPilot 4.6.0 or later is required. TAG-E needs a 5V supply able to deliver at least 2A.

#. Connect TAG-E's IO-B connector to a telemetry port on the autopilot (e.g. TELEM1 or TELEM2), and TAG-E's USB-C port to the camera.
#. Set the parameters for the serial port used (SERIAL2 shown as an example):

   - :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 2 (MAVLink2)
   - :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 921 (921600 bps)
   - All SR2_ stream rate parameters = 0. TAG-E requests the messages it needs itself.
   - :ref:`CAM1_TYPE <CAM1_TYPE>` = 5 (MAVLink) or 6 (MAVLinkCamV2). Both work.

#. Reboot the autopilot. When the camera is online, TAG-E's LED turns green and the camera appears in the ground station.

TAG-E units with older firmware use 230400 bps on the MAVLink port. See the manufacturer's `ArduPilot connection manual <https://airpixel.cz/docs/tag-e-mavlink-connection/>`_ for changing it, and the `GPS delay calculation <https://airpixel.cz/docs/gps-delay-calculation/>`_ for the best geotag accuracy.

Triggering
==========

Once :ref:`CAM1_TYPE <CAM1_TYPE>` is set, the normal ArduPilot camera triggers reach the ILX-LR1: ``DO_SET_CAM_TRIGG_DIST`` (for example from a Mission Planner survey grid), ``DO_DIGICAM_CONTROL``, ``IMAGE_START_CAPTURE``/``IMAGE_STOP_CAPTURE``, ``VIDEO_START_CAPTURE``/``VIDEO_STOP_CAPTURE``, RC auxiliary switches (Camera Trigger, Camera Record Video) and the ground station shutter button. See :ref:`common-camera-controls` for details.

Because TAG-E writes the geotags into the images on the camera's SD card during the flight, no :ref:`log-based geotagging <common-geotagging-images-with-mission-planner>` is needed afterwards.

More information
================

- Product page: `airpixel.cz <https://airpixel.cz/>`_
- Setup guide: `ArduPilot camera control for the ILX-LR1 <https://airpixel.cz/nw/guides/ardupilot-ilx-lr1-camera-control/>`_
- Documentation: `airpixel.cz/docs-tag-e <https://airpixel.cz/docs-tag-e/>`_

[copywiki destination="copter,plane,rover,sub"]
