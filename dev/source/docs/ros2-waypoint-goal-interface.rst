.. _ros2-waypoint-goal-interface:

====================================
Goal Interface - Waypoints ROS2
====================================

This page shows how to set up ROS 2 DDS with ArduPilot SITL for global position control. 

.. note::

    these pages are a work-in-progress


Usage
============

This page assumes that you've successfully followed the ROS 2 basic setups from this wiki: :ref:`ROS 2 <ros2>`, :ref:`ROS 2 with SITL <ros2-sitl>`, and :ref:`ROS 2 with gazebo <ros2-gazebo>`.

Once that's done, start the MicroROS Agent.

.. tabs::

    .. group-tab:: ArduPilot 4.5

        .. code-block:: bash

            cd ~/ardu_ws/src
            source ~/ardu_ws/install/setup.bash
            cd ardupilot/libraries/AP_DDS
            ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019 -r dds_xrce_profile.xml

    .. group-tab:: ArduPilot 4.6 and later

        .. code-block:: bash

            cd ~/ardu_ws/src
            source ~/ardu_ws/install/setup.bash
            cd ardupilot/libraries/AP_DDS
            ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

And, in another terminal, run SITL.

.. code-block:: bash

    cd ~/ardu_ws/src/ardupilot
    source ~/ardu_ws/install/setup.bash
    ./Tools/autotest/sim_vehicle.py -w -v ArduPlane --console  --enable-DDS --map -DG

Now that SITL is running, you have two options: run the ROS2 Node or use CLI commands.

Using CLI Commands
==================

1. **Arming the Plane**:
    - Run the following command to arm the motors:

    .. code-block:: bash

        ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"

2. **Switching to Takeoff Mode**:
    - Change the flight mode to takeoff:

    .. code-block:: bash

        ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 13}"

3. **Reaching Takeoff Altitude**:
    - Wait until the plane reaches the takeoff altitude. You can monitor the altitude using:

    .. code-block:: bash

        ros2 topic echo /ap/geopose/filtered

    - Ensure the plane's altitude reaches the desired level (e.g., 630 meters AGL).

4. **Switching to Guided Mode**:
    - Once the takeoff altitude is reached, switch to guided mode:

    .. code-block:: bash

        ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 15}"

5. **Sending Waypoint Command**:
    - Publish the goal position to guide the plane to the desired waypoint:

    .. code-block:: bash

        ros2 topic pub /ap/cmd_gps_pose ardupilot_msgs/msg/GlobalPosition "{header: {frame_id: 'map'}, coordinate_frame: 5, latitude: 40.08370, longitude: -105.21740, altitude: 1630.0}"

Using ROS2 Node
===============

.. code-block:: bash

    cd ~/ardu_ws
    source ./install/setup.bash
    ros2 run ardupilot_dds_tests plane_waypoint_follower

Understanding the ROS 2 Node
============================

Here's a detailed explanation of how this `ROS2 Node works <https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/plane_waypoint_follower.py>`_:

1. **Initialization**:
    - The node is initialized with `PlaneWaypointFollower(Node)`, which sets up the necessary parameters and clients for communication with ArduPilot services.

2. **Service Clients**:
    - `self._client_arm` and `self._client_mode_switch` are created to handle arming the motors and switching flight modes, respectively.
    - These clients wait for the respective services to become available before proceeding.

3. **Publishers and Subscribers**:
    - A publisher `self._global_pos_pub` is created to send global position commands.
    - A subscriber `self._subscription_geopose` listens for GeoPose messages to update the current position of the plane.

4. **Callback Function**:
    - `geopose_cb(self, msg: GeoPoseStamped)` processes incoming GeoPose messages and updates the current position.

5. **Arming and Mode Switching Functions**:
    - `arm(self)` sends a request to arm the motors.
    - `switch_mode(self, mode)` sends a request to change the flight mode to either takeoff or guided.
    - `arm_with_timeout(self, timeout: rclpy.duration.Duration)` and `switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration)` try to arm and switch modes with a timeout.

6. **Mission Execution**:
    - The node attempts to arm the plane and switch to takeoff mode.
    - Once the plane reaches the takeoff altitude, it switches to guided mode.
    - The node sends a goal position for the plane to reach using guided control.
    - The `achieved_goal(goal_global_pos, cur_geopose)` function checks if the plane has reached the goal.

Demo
====

Here's a quick video showcasing the results expected from this tutorial:

..  youtube:: SHHw190RaHc
    :width: 100%

..  youtube:: FnChCmwBbHA 
    :width: 100%

For more information, refer to this `PR <https://github.com/ArduPilot/ardupilot/pull/25722>`__.
