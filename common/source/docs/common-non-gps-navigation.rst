.. _common-non-gps-navigation:

[copywiki destination="copter,rover"]

==================
Non-GPS Navigation
==================

..  youtube:: FjuC1mN8nU4
    :width: 100%

These are the available options that allow a vehicle to estimate its position without a GPS.  Once enabled this allows all autonomous and semi-autonomous modes just as they do would a GPS is available.

- :ref:`MarvelMind Beacons <common-marvelmind>`
- :ref:`Optical Flow <common-optical-flow-sensors-landingpage>`
- :ref:`OptiTrack motion capture system <common-optitrack>`
- :ref:`Pozyx Beacons <common-pozyx>`
- :ref:`ROS cartographer <ros-cartographer-slam>`
- :ref:`Visual Odometry with OpenKai and ZED <common-zed>`
- :ref:`Visual Odometry with VIO tracking camera <common-vio-tracking-camera>`
- :ref:`Visual Odometry with VIO tracking camera and ROS <ros-vio-tracking-camera>`

.. note::

   The low cost IMUs (accelerometers, gyros, compass) used in most flight controllers drift too quickly to allow position estimation without an external velocity or position source.  In other words, low-cost IMUs on their own are not sufficient for estimating position
