.. _common-non-gps-navigation-landing-page:

[copywiki destination="copter,rover,blimp"]

==================
Non-GPS Navigation
==================

..  youtube:: FjuC1mN8nU4
    :width: 100%

These are the available options that allow a vehicle to estimate its position without a GPS.  Once enabled this allows all autonomous and semi-autonomous modes just as they do would a GPS is available.

.. toctree::
    :maxdepth: 1

    Intel RealSense T265 <common-vio-tracking-camera>
    MarvelMind Beacons <common-marvelmind>
    Nooploop Beacons <common-nooploop>
[site wiki="copter"]
    Nokov Indoor Optical Tracking <https://discuss.ardupilot.org/t/nokov-indoor-optical-tracking-system>
    Optical Flow <common-optical-flow-sensors-landingpage>
    OptiTrack motion capture system <common-optitrack>
[/site]
    Pozyx Beacons <common-pozyx>
    ROS with Google Cartographer (Developers only) <https://ardupilot.org/dev/docs/ros-cartographer-slam.html>
    Vicon Positioning System <common-vicon-for-nongps-navigation>
    Visual Odometry with OpenKai and ZED <common-zed>
[site wiki="rover"]
    Wheel Encoders <wheel-encoder>
[/site]
    GPS/Non-GPS Transitions <common-non-gps-to-gps>

.. note::

   The low cost IMUs (accelerometers, gyros, compass) used in most autopilots drift too quickly to allow position estimation without an external velocity or position source.  In other words, low-cost IMUs on their own are not sufficient for estimating position
   
.. note::

   A board with more than 1MB of flash is required to run non-GPS navigation. See :ref:`Firmware Limitations <common-limited_firmware>` for details
   
