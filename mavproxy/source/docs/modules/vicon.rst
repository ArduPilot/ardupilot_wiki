========
Vicon
========

.. code:: bash

    module load vicon
    vicon set
    vicon start


This module connects to a Vicon motion tracking system and sends position data to the UAV. For more information on how to integrate the Vicon system with ArduPilot, :ref:`see here <copter:common-vicon-for-nongps-navigation>`.

The module has the following settings, which via be set via ``vicon set``.

==================   =====================================================================  ===============================
Setting              Description                                                            Default
==================   =====================================================================  ===============================
host                 IP Address or hostname of Vicon server                                 vicon
origin_lat           Latitude of the Vicon world frame.                                     -35.363261 (Canberra Model Aero Club Runway)
origin_lon           Longitude of Vicon world frame                                         149.165230
origin_alt           Altitude of Vicon world frame                                          584.0 m
vision_rate          Rate to send VISION_ESTIMATE messages                                  14 Hz
vel_filter_hz        Velocity Filter cutoff frequency                                       30 Hz
gps_rate             Rate to send GPS_INPUT messages                                        5 Hz
gps_nsats            Number of satellites to report in the simulated GPS message            5
object_name          Name of the object you want to track in the Vicon Tracker software     None
==================   =====================================================================  ===============================

By default, the module will auto-detect the first object that is found by the Vicon system. If you have other objects in the space that you don't want to be detected, you can set the ``object_name`` parameter.