.. _ros2-sitl:

===============
ROS 2 with SITL
===============

Once ROS2 is correctly :ref:`installed <ros2>`, and SITL is also :ref:`installed <sitl-simulator-software-in-the-loop>`, source your workspace and launch Ardupilot SITL with ROS 2! 

.. code-block:: bash

    source ~/ros2_ws/install/setup.bash
    ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501

If the ROS 2 topics aren't being published, set the ardupilot parameter DDS_ENABLE to 1 manually and reboot the launch

.. code-block:: bash
    
    export PATH=$PATH:~/ros2_ws/src/ardupilot/Tools/autotest
    sim_vehicle.py -w -v ArduPlane --console -DG --enable-dds

    param set DDS_ENABLE 1

For more information refer to `ardupilot/Tools/ros2/README.md <https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2#readme>`__. There you can find examples of launches using serial connection instead of udp, as well as a step-by-step breakdown of what the launch files are doing.