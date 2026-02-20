.. _ros2-sitl:

===============
ROS 2 with SITL
===============

After the ROS2 environment,  `Micro-XRCE-DDS-Gen` and the ROS2 package `micro-ROS-Agent` are correctly installed following :ref:`Install ROS2 <ros2>`, you still need to build the simulator of Ardupilot to launch SITL in ROS2.  

The simulator will be built from the source code in the `ardu_ws/src/ardupilot` that is cloned in :ref:`Installation (Ubuntu) <ros2_installation_ubuntu>`.

You need to install the ArduPilot dependencies first.

.. code-block:: bash

    cd ardu_ws/src/ardupilot
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y

Then, build Ardupilot for SITL with DDS enabled. The example below shows how to build the copter firmware, but you can replace `copter` with `plane`, `rover`, etc.

You can build the ROS2 packages `ardupilot_msgs`, `micro_ros_agent`, `ardupilot_sitl` and `ardupilot_dds_tests` as 

.. code-block:: bash

    cd ardu_ws/
    colcon build --packages-up-to ardupilot_sitl  


Then, source your workspace 

.. code-block:: bash

    cd ~/ardu_ws
    source ./install/setup.bash


and you are able to launch the SITL in ROS2 with the following command

.. tabs::

   .. group-tab:: ArduPilot 4.5

      .. code-block:: bash

        source /opt/ros/humble/setup.bash
        cd ~/ardu_ws/
        colcon build --packages-up-to ardupilot_sitl
        source install/setup.bash
        ros2 launch ardupilot_sitl sitl_dds_udp.launch.py \
        transport:=udp4 \
        refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml \
        synthetic_clock:=True \
        wipe:=False \
        model:=quad \
        speedup:=1 \
        slave:=0 \
        instance:=0 \
        defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm \
        sim_address:=127.0.0.1 \
        master:=tcp:127.0.0.1:5760 \
        sitl:=127.0.0.1:5501


   .. group-tab:: ArduPilot 4.6 and later

      .. code-block:: bash

        source /opt/ros/humble/setup.bash
        cd ~/ardu_ws/
        colcon build --packages-up-to ardupilot_sitl
        source install/setup.bash
        ros2 launch ardupilot_sitl sitl_dds_udp.launch.py \
        transport:=udp4 \
        synthetic_clock:=True \
        wipe:=False \
        model:=quad \
        speedup:=1 \
        slave:=0 \
        instance:=0 \
        defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm \
        sim_address:=127.0.0.1 \
        master:=tcp:127.0.0.1:5760 \
        sitl:=127.0.0.1:5501

More modules of `mavproxy` like `map` and `console` can be added in the above command, for example, `map:=True console:=True`.

It launches three three processes:

* `micro-ROS-Agent` is a wrapper around `Micro-XRCE-DDS-Agent` and provides connection to the ROS2,
* `ardupilot_sitl`  is a ROS2 package that start the SITL binary connected to ROS2 through `micro-ROS-Agent`,
* `mavproxy` is a GCS that connects to SITL through the MAVLink protocol. 

For more information refer to `ardupilot/Tools/ros2/README.md <https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2#readme>`__.
There you can find examples of launches using serial connection instead of udp, as well as a step-by-step breakdown of what the launch files are doing.

ROS 2 CLI
=========

Once everything is running, you can now interact with ArduPilot through the ROS 2 CLI.

.. code-block:: bash

    source ~/ardu_ws/install/setup.bash
    # See the node appear in the ROS graph
    ros2 node list
    # See which topics are exposed by the node
    ros2 node info /ap
    # Echo a topic published from ArduPilot
    ros2 topic echo /ap/geopose/filtered

DDS is responsible for ROS2 communication. If the ROS2 topics are not published, first check if `DDS_ENABLE` is set to ``1`` or not. If not, set as ``1``. It can be done through Mission Planner, QGroundControl, or the commands in `mavproxy` as bellow, then reboot the launch.

.. code-block:: bash

    export PATH=$PATH:~/ardu_ws/src/ardupilot/Tools/autotest
    # a Copter is simulated in this example, but you can replace it with Plane, Rover, etc.
    sim_vehicle.py -w -v ArduCopter --console -DG --enable-DDS 

    param set DDS_ENABLE 1


The second aspect to check, ensure the ArduPilot parameter `DDS_DOMAIN_ID` matches your environment variable ``ROS_DOMAIN_ID``.The default is ``0`` for ArduPilot, which corresponds to the environment variable being unset. You may need to relaunch the SITL after changing the parameters. 


MAVProxy
========

To test and fly around, you can launch a `mavproxy <https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html>`__ instance in yet another terminal:

.. code-block:: bash
    
    mavproxy.py --console --map --aircraft test --master=:14550


Next up
=======

Add Gazebo in :ref:`ROS 2 with Gazebo <ros2-gazebo>`
