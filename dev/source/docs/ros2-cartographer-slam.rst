.. _ros2-cartographer-slam:

====================================
Cartographer SLAM with ROS 2 in SITL
====================================

This page shows how to setup ROS 2 with ArduPilot SITL and run Google Cartographer as a SLAM source. 

Installation
============

This page assumes that you've successfully followed the ROS 2 basic setups from this wiki: :ref:`ROS 2 <ros2>`, :ref:`ROS 2 with SITL <ros2-sitl>` and :ref:`ROS 2 with gazebo <ros2-gazebo>`.

Once that's done, simply run:

.. code-block:: bash

    cd ~/ros2_ws/src
    git clone git@github.com:ArduPilot/ardupilot_ros.git

.. code-block:: bash
    
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r --skip-keys gazebo-ros-pkgs

Now source your workspace and build `ardupilot_ros`:

.. code-block:: bash

    cd ~/ros2_ws
    source ./install/setup.bash
    colcon build --packages-up-to ardupilot_ros ardupilot_gz_bringup

Usage
=====

This package is used in combination with `ardupilot_gz`, first we will launch a simulation containing a copter equipped with a 360 degress 2D lidar in a maze world.
To launch rviz and gazebo, run:

.. code-block:: bash
    
    source ~/ros2_ws/install/setup.sh
    ros2 launch ardupilot_gz_bringup iris_maze.launch.py

Now, we can launch Google Cartographer to generate SLAM, check if a map is being generated correctly in RVIZ.
In another terminal, run:

.. code-block:: bash
    
    source ~/ros2_ws/install/setup.sh
    ros2 launch ardupilot_ros cartographer.launch.py

Configure ArduPilot
===================

If you'd like to get the information from Cartographer to go into ArduPilot's extended kalman filter, you will need to change some parameters, you can do that through any GCS, including mavproxy:

-  :ref:`AHRS_EKF_TYPE <copter:AHRS_EKF_TYPE>` = 3 to use EKF3
-  :ref:`EK2_ENABLE <copter:EK2_ENABLE>` = 0 to disable EKF2
-  :ref:`EK3_ENABLE <copter:EK3_ENABLE>` = 1 to enable EKF3
-  :ref:`EK3_SRC1_POSXY <copter:EK3_SRC1_POSXY>` = 6 to set position horizontal source to ExternalNAV
-  :ref:`EK3_SRC1_POSZ <copter:EK3_SRC1_POSZ>` = 1 to set position vertical source to Baro
-  :ref:`EK3_SRC1_VELXY <copter:EK3_SRC1_VELXY>` = 6 to set velocity horizontal source to ExternalNAV
-  :ref:`EK3_SRC1_VELZ <copter:EK3_SRC1_VELZ>` = 6 to set vertical velocity source to ExternalNAV
-  :ref:`EK3_SRC1_YAW <copter:EK3_SRC1_YAW>` = 6 to set yaw source to ExternalNAV
-  :ref:`VISO_TYPE <copter:VISO_TYPE>` = 1 to enable visual odometry
-  :ref:`ARMING_CHECK <copter:ARMING_CHECK>` = 388598 (optional, to disable GPS checks)


After changing the values above, reboot the flight controller.

.. warning::
    The parameters above are recommended for SITL. If you plan on using this on a real copter, it is a good idea to setup a second source of EKF. This way the robot doesn't crash if the external odometry you are providing stops publishing or gets lost.

    Please refer to this link for more information on `Common EKF Sources <https://ardupilot.org/copter/docs/common-ekf-sources.html>`__ as well as this guide on `GPS / Non-GPS Transitions <https://ardupilot.org/copter/docs/common-non-gps-to-gps.html>`__.
 


Demo
====

Here's a quick video showcasing the results expected from this tutorial:

..  youtube:: bpjGyAECKHA
    :width: 100%

For more information regarding the `ardupilot_ros` package refer to `ardupilot_ros's github page <https://github.com/ArduPilot/ardupilot_ros>`__.