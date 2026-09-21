.. _ros2:

=====
ROS 2
=====

.. image:: ../images/logos/ros2_logo.png
    :target: ../_images/logos/ros2_logo.png

ArduPilot capabilities can be extended with `ROS <http://www.ros2.org/>`__ (aka Robot Operating System).


Prerequisites
=============

Before using ArduPilot with ROS, you should first be familiar with both ArduPilot and ROS before trying to integrate them together.

- Learn how to use ArduPilot first by following the relevant wiki for `Rover <https://ardupilot.org/rover/index.html>`__, `Copter <https://ardupilot.org/copter/index.html>`__ or `Plane <https://ardupilot.org/plane/index.html>`__. In particular, make sure the vehicle works well in Manual and Autonomous modes like Guided and Auto before trying to use ROS.
- Learn how to use ROS by reading the `ROS 2 tutorials <https://docs.ros.org/en/humble/Tutorials.html#>`__.  In the case of a problem with ROS, it is best to ask on ROS community forums or `Robotics Stack Exchange <https://robotics.stackexchange.com/>`__ first (or google your error). You will find many other tutorials about ROS like `Emlid <https://docs.emlid.com/navio2/ros>`__.

ROS 2
=====

.. toctree::
    :maxdepth: 1

        Install ROS 2 <ros2-install>
        ROS 2 with SITL <ros2-sitl>
        ROS 2 Interfaces <ros2-interfaces>
        ROS 2 with SITL in Gazebo <ros2-gazebo>
        ROS 2 over Ethernet <ros2-over-ethernet>
        ROS 2 waypoint goal interface <ros2-waypoint-goal-interface>
        Cartographer SLAM with ROS 2 in SITL <ros2-cartographer-slam>
        ROS 2 on Raspberry Pi <ros2-pi>


Conclusion
==========

ROS is capable of extending autopilot capabilities with a wider ecosystem of technologies that can run on more powerful computers.

Here is ArduPilot Rover performing path planning around objects using ROS navigation.

..  youtube:: tqVH4lWk51Y
    :width: 100%

We are keen to improve ArduPilot's support of ROS so if you find issues (such as commands that do not seem to be supported), please report them in the `ArduPilot issues list <https://github.com/ArduPilot/ardupilot/issues>`__ with a title that includes "ROS" and we will attempt to resolve them as quickly as possible.
