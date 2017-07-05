.. _ros:

===
ROS
===

.. image:: ../images/logos/rosorg_logo.png
    :target: ../_images/logos/rosorg_logo.png

ArduPilot capabilities can be extended with `ROS <http://www.ros.org/>`__. ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.
ROS is completely open source (BSD) and free for others to use, change and commercialize upon. The primary goal is to enable software developers to build more capable robot applications quickly and easily on a common platform.
Those pages will show you how to:

- Connect ardupilot to ROS
- Retrieve ardupilot information in ROS
- Use ROS to command an ardupilot vehicle
- Simulate ardupilot in `Gazebo <http://gazebosim.org/>`__

.. warning::

    Those pages won't show you how to:

    - Install and setup ROS
    - Make a fully autonomous vehicle
    - Make non-GPS vehicle

Those are outside the scope of ardupilot and difficult. Even if ROS and ardupilot are powerful, their combined usage will require more work than the example shown in this wiki to make advance things (SLAM, autonomous movement etc). You can find a good description of current ROS limits in http://design.ros2.org/articles/why_ros2.html

Preconditions
=============

- Learn on to use ardupilot first! Don't expect that things will work magically especially with a real vehicle. You need to understand and make your vehicle working well in MANUAL and GUIDED mode before trying to use ROS. We won't explain here how to setup your vehicle nor why it won't arm or takeoff.
- Learn how to use ROS! Please do at least ROS all `beginner tutorials <http://wiki.ros.org/ROS/Tutorials>`__. In the case of a problem with ROS, ask on ROS community first (or google your error). You will find many other tutorials about ROS like `Emlid <https://docs.emlid.com/navio2/common/dev/ros/>`__, reading some of them is always beneficial.

- Use ROS Kinetic Kame on Ubuntu Linux (16.04). Those are default and LTS version of both ROS and Ubuntu. THIS DON'T WORK ON WINDOWS NOR MACOS ! We assume now that you are using Ubuntu and ROS Kinetic.
- If you are on a desktop PC, please install ROS Desktop-Full, on a companion computer ROS-Base is enough, see `<http://wiki.ros.org/kinetic/Installation/Ubuntu>`__.
- Please add ROS tool to your shell as stated in ROS wiki.
- Please don't use root as the default user.

- To make the connection between ROS and ardupilot, we will use `MAVROS <http://wiki.ros.org/mavros>`__. Install it should be as simple as

.. code-block:: bash

    sudo apt install ros-kinetic-mavros

and for simpler usage on a desktop computer, please install RQT

.. code-block:: bash

    sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-rqt-robot-plugins


- We recommend the usage of `caktin_tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`__ instead of the default catkin_make as it is more powerful and got some linter to help you

.. code-block:: bash

    sudo apt-get install python-catkin-tools

.. tip::

    ROS got tabs completion so abuse of it!

Next tutorials :

.. toctree::
    :maxdepth: 1

        ROS with SITL <ros-sitl>
        ROS with SITL in Gazebo <ros-gazebo>
        ROS with real vehicle <ros-vehicle>
        ROS with distance sensors <ros-distance-sensors>


Instructions for using :ref:`Gazebo with ArduPilot are here <using-gazebo-simulator-with-sitl>` and an old version has been `blogged about here <http://diydrones.com/profiles/blogs/705844:BlogPost:2151758>`__.

..  youtube:: orMXVby-tSI
    :width: 100%

    VR Robotics has successfully used Rover with ROS for SLAM as demonstrated below.

..  youtube:: DUsPAa20YdQ
    :width: 100%

    We are keen to improve ArduPilot's support of ROS so if you find issues (such as commands that do not seem to be supported), please report them in the `ArduPilot issues list <https://github.com/ArduPilot/ardupilot/issues>`__ with a title that includes "ROS" and we will attempt to resolve them as quickly as possible.
