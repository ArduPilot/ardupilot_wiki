.. _ros:

===
ROS
===

.. image:: ../images/logos/rosorg_logo.png
    :target: ../_images/logos/rosorg_logo.png

ArduPilot capabilities can be extended with `ROS <http://www.ros.org/>`__ (aka Robot Operating System).

`ROS <http://www.ros.org/>`__ provides libraries, tools, hardware abstraction, device drivers, visualizers, message-passing, package management, and more to help software developers create robot applications.  In the future we expect ROS will be replaced by `ROS2 <http://design.ros2.org/articles/why_ros2.html>`__

`MAVROS <http://wiki.ros.org/mavros>`__ is a ROS "node" that can convert between ROS topics and `MAVLink messages <https://github.com/ArduPilot/mavlink>`__ allowing ArduPilot vehicles to communicate with ROS.  The `MAVROS code can be found here <https://github.com/mavlink/mavros/tree/master/mavros>`__.

These pages will show you how to:

- :ref:`Install ROS and MAVROS <ros-install>`
- :ref:`Connecting to ArduPilot from ROS <ros-connecting>`
- :ref:`SLAM for non-GPS navigation <ros-slam>`
- Retrieve ArduPilot information in ROS
- Use ROS to command an ArduPilot vehicle
- Simulate ArduPilot in `Gazebo <http://gazebosim.org/>`__

Prerequisites
=============

- Learn on to use ArduPilot first by following the relevant wiki for `Rover <http://ardupilot.org/rover/index.html>`__, `Copter <http://ardupilot.org/copter/index.html>`__ or `Plane <http://ardupilot.org/plane/index.html>`__. In particular make sure the vehicle works well in Manual and Autonomous modes like Guided and Auto before trying to use ROS.
- Learn how to use ROS by reading the `beginner tutorials <http://wiki.ros.org/ROS/Tutorials>`__.  In the case of a problem with ROS, it is best to ask on ROS community forums first (or google your error). You will find many other tutorials about ROS like `Emlid <https://docs.emlid.com/navio2/common/dev/ros/>`__.
- Install ROS Kinetic Kame on Ubuntu Linux (16.04). Those are default and LTS version of both ROS and Ubuntu.  ROS does not yet officially support Windows nor MAC.

Next tutorials :

.. toctree::
    :maxdepth: 1

        Installing ROS and MAVROS <ros-install>
        Connecting to ArduPilot from ROS <ros-connecting>
        SLAM for non-GPS navigation <ros-slam>
        ROS with SITL <ros-sitl>
        ROS with SITL in Gazebo <ros-gazebo>
        ROS with distance sensors <ros-distance-sensors>

Instructions for using :ref:`Gazebo with ArduPilot are here <using-gazebo-simulator-with-sitl>` and an old version has been `blogged about here <http://diydrones.com/profiles/blogs/705844:BlogPost:2151758>`__.

..  youtube:: orMXVby-tSI
    :width: 100%

    VR Robotics has successfully used Rover with ROS for SLAM as demonstrated below.

..  youtube:: DUsPAa20YdQ
    :width: 100%

    We are keen to improve ArduPilot's support of ROS so if you find issues (such as commands that do not seem to be supported), please report them in the `ArduPilot issues list <https://github.com/ArduPilot/ardupilot/issues>`__ with a title that includes "ROS" and we will attempt to resolve them as quickly as possible.
