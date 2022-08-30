.. _skyrocket-gcs:

====================================
SkyRocket with other Ground Stations
====================================

The SkyViper has its own app that runs on android or iOS, but to access the full capabilities of the ArduPilot firmware you may prefer to use another ground station package. The SkyViper can work with any of the :ref:`ArduPilot compatible ground stations <common-choosing-a-ground-station>` including :ref:`Mission Planner <planner:install-mission-planner>`.  For any of them:

- connect your PC/tablet/phone to the drone's wifi access point
- on the ground station set the connection type to "UDP" and press connect.  If asked, set the port to "14550"

.. image:: ../images/skyrocket-with-mp.png
    :width: 70%

If using Mission Planner, the video from the SkyViper can be made to appear in the “HUD” (the Heads Up Display which normally shows the blue and green attitude display) The SkyViper uses UDP broadcasts you can connect from multiple devices at once. Use the following setup sequence:

1. Install the gstreamer 1.0-x86_64-1.9.2.msi package from [here](https://gstreamer.freedesktop.org/data/pkg/windows/1.9.2/)
2. In Mission Planner, right mouse click on the HUD and select "Video" then "Set GStreamer Source".
3. Change the IP setting and port for the SkyViper video in Sonix board.

**Default Setting in MP**
rtspsrc location=rtsp://192.168.1.133:8554/video1 ! application/x-rtp ! rtpjpegdepay ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink

**Setting for SkyViper Video MP**
rtspsrc location=rtsp://192.168.99.1:5600/video1 ! application/x-rtp ! rtpjpegdepay ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink

1. Launch Mission Planner after the SkyViper has been powered up and the laptop WiFi has been connected to the drone’s wifi access point.

.. warning::

      The SkyViper uses MAVLink2 by default, which does not work with some older GCS software (such as Tower) that only use MAVLink1. To use these older GCSs you may need to change the :ref:`SERIAL1_PROTOCOL <SERIAL1_PROTOCOL>` parameter to 1 from the default of 2.
