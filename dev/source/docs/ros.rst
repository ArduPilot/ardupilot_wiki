.. _ros:

=============
ROS 1 / ROS 2
=============

.. image:: ../images/logos/rosorg_logo.png
    :target: ../_images/logos/rosorg_logo.png

ArduPilot capabilities can be extended with `ROS <http://www.ros.org/>`__ (aka Robot Operating System).

`ROS <http://www.ros.org/>`__ provides libraries, tools, hardware abstraction, device drivers, visualizers, message-passing, package management, and more to help software developers create robot applications.
ROS1 is being replaced by `ROS 2 <http://design.ros2.org/articles/why_ros2.html>`__


.. image:: ../images/logos/ros2_logo.jpg
    :target: ../_images/logos/ros2_logo.jpg

The tutorials below are split between ROS 1 and ROS 2.
If you aren't sure which version to use, the ArduPilot development team recommends ROS 2 because ROS 1 is end-of-life in 2025.

Prerequisites
=============

Before using ArduPilot with ROS, you should first be familiar with both ArduPilot and ROS before trying to integrate them together.

- Learn how to use ArduPilot first by following the relevant wiki for `Rover <https://ardupilot.org/rover/index.html>`__, `Copter <https://ardupilot.org/copter/index.html>`__ or `Plane <https://ardupilot.org/plane/index.html>`__. In particular, make sure the vehicle works well in Manual and Autonomous modes like Guided and Auto before trying to use ROS.
- Learn how to use ROS by reading the `ROS 1 tutorials <http://wiki.ros.org/ROS/Tutorials>`__ or the `ROS 2 tutorials <https://docs.ros.org/en/humble/Tutorials.html#>`__.  In the case of a problem with ROS, it is best to ask on ROS community forums or `Robotics Stack Exchange <https://robotics.stackexchange.com/>`__ first (or google your error). You will find many other tutorials about ROS like `Emlid <https://docs.emlid.com/navio2/ros>`__.

MAVROS
======

`MAVROS <http://wiki.ros.org/mavros>`__ is a ROS package that can convert between ROS topics and `MAVLink messages <https://github.com/ArduPilot/mavlink>`__ allowing ArduPilot vehicles to communicate with ROS.
The `MAVROS code can be found here <https://github.com/mavlink/mavros/tree/master/mavros>`__.
Starting in ArduPilot 4.5, ArduPilot supports a direct DDS interface compatible with ROS 2, which removes the need to use MAVROS for certain applications.

ROS 1
=====

.. toctree::
    :maxdepth: 1

        Install ROS and MAVROS <ros-install>
        Connecting to ArduPilot from ROS <ros-connecting>
        Hector SLAM for non-GPS navigation <ros-slam>
        Google Cartographer SLAM for non-GPS navigation <ros-cartographer-slam>
        VIO tracking camera for non-GPS navigation <ros-vio-tracking-camera>
        Sending Commands from rviz <ros-rviz>
        Object Avoidance <ros-object-avoidance>
        Clock/Time syncronisation <ros-timesync>
        Send data from AP to ROS/mavros <ros-data-from-ap>
        ROS with SITL <ros-sitl>
        ROS with SITL in Gazebo <ros-gazebo>
        ROS with distance sensors <ros-distance-sensors>
        ROS with Aruco Boards detection <ros-aruco-detection>
        ROS with Apriltag Boards detection <ros-apriltag-detection>

ROS 2
=====

.. toctree::
    :maxdepth: 1

        Install ROS 2 <ros2>
        ROS 2 with SITL <ros2-sitl>
        ROS 2 with SITL in Gazebo <ros2-gazebo>
        ROS 2 over Ethernet <ros2-over-ethernet>
        Cartographer SLAM with ROS 2 in SITL <ros2-cartographer-slam>


Conclusion
==========

ROS is capable of extending autopilot capabilities with a wider ecosystem of technologies that can run on more powerful computers.

Here is ArduPilot Rover performing path planning around objects using ROS navigation.

..  youtube:: tqVH4lWk51Y
    :width: 100%

We are keen to improve ArduPilot's support of ROS so if you find issues (such as commands that do not seem to be supported), please report them in the `ArduPilot issues list <https://github.com/ArduPilot/ardupilot/issues>`__ with a title that includes "ROS" and we will attempt to resolve them as quickly as possible.
