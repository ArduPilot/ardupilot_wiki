.. _common-geotagging-airpixel-tag-e:

================================================================
Airpixel TAG-E for Camera Control and EXIF Geotagging of ILX-LR1
================================================================

.. image:: https://airpixel.cz/wp-content/uploads/2025/07/Cube-Tag-e.webp
    :target: https://airpixel.cz/tag-e/
    :width: 445px

The `TAG-E <https://airpixel.cz/tag-e/>`_ is new camera controller and geotagger for Sony ILX-LR1 camera. TAG-E uses MAVLink Camera Control protocol v2 to interface camera features to the user. QGC will automatically show exposure controls, triggering, timelapse and and configuration features in any Android device or PC/MAC. For MissionPlanner there is an plugin for camera and geotagging control.
TAG-E is using single USB-C connection to the camera, no WiFi, no additional modules. Geotagging is instant, without necessity of geoagging initiation. Triggering is available at full speed, no slow-down due to geotagging processing. TAG-E also automatically configure camera internal clock by GPS time received from the MAVLink, so images will have correct creation date every time.


- Compatible only with Sony ILX-LR1
- Photos are automatically geotagged (via EXIF) with the Lat, Lon, Altitude and camera angles (read from the gimbal)
- Optionally can geotags be expanded for GPS time, Rangefinder measurement, GPS/IMU accuracy, Focal plane distance and resolution or Custom user label
- HereLink camera control via QGC UI
- HereLink camera control via MavCam (optional)
- Mission Planner implementation via plugin
- Precise lever arm calculations based on Antenna to Camera offsets
- GeoTagging is available at the *maximum speed of the camera*
- Automatic camera clock configuration by GPS time
- Enhanced file/folder grouping per flight for better user exprience
- Video geotagging by subtitles


More info at `www.airpixel.cz <https://airpixel.cz/tag-e/>`_
Documentation at `www.airpixel.cz/docs <https://airpixel.cz/docs>`_

[copywiki destination="copter,plane,rover,sub"]
