.. _using-gazebo-simulator-with-sitl:

================================
Using Gazebo Simulator with SITL
================================

This article explains how to use Gazebo http://gazebosim.org/
as an external simulator for ArduPilot Rover, Copter and Plane.

Overview
========

Gazebo is a well-known and respected robotics simulator, and is also the official DARPA Virtual Robotics Simulator.
No current release has built-in support for ArduPilot,  however. (Previous PRs for built-in support were not merged as of April-2017).

.. warning::

   If you find that some of these instructions are outdated, please open an issue for the ardupilot_wiki repository on github.

.. tip::

   Gazebo is particularly useful for defining autonomous
   indoor flights or swarms.


Preconditions
=============

We recommend Ubuntu starting from 16.04 or 18.04 as those were  the platform used for testing this approach. They are also known to be compatible with SITL.

We will be using a standard version of ArduPilot but a custom plugin for Gazebo, until the gazebo plugin gets merged into Gazebo-master.

This plugin can be used with or without ROS integration. In both case we recommend to use Gazebo from the OSRF repository.

::

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt update

The plugin we will be using works with gazebo versions 7 to 9. We recommend gazebo9 as the new development will append on this version.
It isn't available for older versions of  Ubuntu though, so you may need to use an earlier version depending on your system.

::

    sudo apt install gazebo9 libgazebo9-dev


The first time gazebo is executed requires the download of some models and it could take some time, so please be patient. Let's give a try.

::

    gazebo --verbose

It should open an empty world.

Plugin installation
===================

The following plugin is a pure Gazebo plugin, so ROS is not needed to use it. You can still use ROS with Gazebo with normal gazebo-ros packages.
We have  two version of the plugin : khancyr and SwiftGust one's.
The one from `khancyr <https://github.com/khancyr/ardupilot_gazebo>`__ is the original one. It is stable and only has necessary file to work.
The one from `SwiftGust <https://github.com/SwiftGust/ardupilot_gazebo>`__  has more examples and a little bit more documentation.
Both use the same plugin, they only differ in the documentation and examples they provide. 

We will be using khancyr plugin for the following exaplation. First clone it somewhere in your home directory. (If you need help with git, please see :ref:`installed git <git-install>`, :ref:`forked <git-fork>` and :ref:`cloned <git-clone>`.)

::

    git clone https://github.com/khancyr/ardupilot_gazebo
    cd ardupilot_gazebo
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install


These instructions will clone the repository  into the ardupilot_gazebo directory, and create a new build directory within it. The plugin will then be  compiled and installed  in the Gazebo plugin directory.

Start the Simulator
===================

In a terminal window start Gazebo:

::

    gazebo --verbose worlds/iris_arducopter_runway.world


In another terminal window, enter the ArduCopter directory and start the SITL simulation:

::

    cd ~/ardupilot/ArduCopter
    ../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map

If all works well, you should see this:

..  youtube:: n_M5Vs5FBGY
    :width: 100%


.. note::

    ROS is commonly used together with Gazebo, but this is out of the scope of this article. If you are using ROS,
    some packages to consider using are:
    - *mavros* (for sending and receiving MAVLink packets)
    - *ros_gazebo_camera* (for publishing Gazebo's virtual camera stream to a ROS topic)
