.. _common-non-gps-navigation-landing-page:

[copywiki destination="copter,rover"]

==================
Non-GPS Navigation
==================

..  youtube:: FjuC1mN8nU4
    :width: 100%

These are the available options that allow a vehicle to estimate its position without a GPS.  Once enabled this allows all autonomous and semi-autonomous modes just as they do would a GPS is available.

.. toctree::
    :maxdepth: 1
  
    MarvelMind Beacons <common-marvelmind>
[site wiki="copter"]
	Optical Flow <common-optical-flow-sensors-landingpage>
    OptiTrack motion capture system <common-optitrack>
[/site]
    Pozyx Beacons <common-pozyx>
    Vicon Positioning System <common-vicon-for-nongps-navigation>
    Visual Odometry with OpenKai and ZED <common-zed>
    Intel RealSense T265 <common-vio-tracking-camera>
[site wiki="rover"]
    Wheel Encoders <wheel-encoder>
[/site]

.. note::

   The low cost IMUs (accelerometers, gyros, compass) used in most autopilots drift too quickly to allow position estimation without an external velocity or position source.  In other words, low-cost IMUs on their own are not sufficient for estimating position
