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
    Luxonis OAK-D <common-vio-oak-d>
    MarvelMind Beacons <common-marvelmind>
    ModalAI VOXL <common-modalai-voxl>
    Nooploop Beacons <common-nooploop>
[site wiki="copter"]
    Nokov Indoor Optical Tracking <https://discuss.ardupilot.org/t/nokov-indoor-optical-tracking-system>
[/site]
[site wiki="copter,plane"]
    Optical Flow <common-optical-flow-sensors-landingpage>
[/site]
[site wiki="copter"]
    OptiTrack motion capture system <common-optitrack>
[/site]
    Pozyx Beacons <common-pozyx>
    ROS with Google Cartographer (Developers only) <https://ardupilot.org/dev/docs/ros-cartographer-slam.html>
    Vicon Positioning System <common-vicon-for-nongps-navigation>
[site wiki="rover"]
    Wheel Encoders <wheel-encoder>
[/site]
    GPS/Non-GPS Transitions <common-non-gps-to-gps>

.. note:: Most of the above systems (except Beacons) require that the ORIGIN be set manually, except if a GPS is present. In order to do this the user must either use the GCS to set Origin, as shown below using Mission Planner, or use a lua script like this `one <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/ahrs-set-origin.lua>`__

.. image:: ../../../images/setorigin.jpg

.. note::
   The low cost IMUs (accelerometers, gyros, compass) used in most autopilots drift too quickly to allow position estimation without an external velocity or position source.  In other words, low-cost IMUs on their own are not sufficient for estimating position.
   
.. note::
  A board with more than 1MB of flash is required to run non-GPS navigation, except for Vicon as 1MB boards still support the GPS_INPUT message, although they don't support the GLOBAL_VISION_POSITION_ESTIMATE so they have to be run using the GPS_INPUT message. See :ref:`Firmware Limitations <common-limited_firmware>` for details. 
   
