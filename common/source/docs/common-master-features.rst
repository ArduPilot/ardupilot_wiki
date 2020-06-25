. _common-master-features:

===========================
Items in Master Branch Only
===========================

This lists features or changes merged in master and build in the "latest" firmware, but not in a stable or beta release yet. These items should be considered at the "alpha" test stage.


.. toctree::
    :maxdepth: 1

[site wiki="copter"]
    In-Flight FTT and filter control <common-imu-fft>
[/site]
[site wiki="copter,plane,rover"]
    Hott Telem (already in Plane release) <common-hott-telemetry>
    RunCam Control <common-camera-runcam>
[/site]

[site wiki="plane"]
- :ref:`Change to PID scaling in QuadPlane Tailsitters <4.1-gainscaling>`
[/site]
- :ref:`Camera Trigger MAV Command change <mav_cmd_do_set_cam_trigg_dist>`


[copywiki destination="plane,copter,rover,dev"]