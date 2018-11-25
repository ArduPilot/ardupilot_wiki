.. _using-gazebo-simulator-with-sitl:

================================
Using Gazebo Simulator with SITL
================================

This article explains how to use Gazebo http://gazebosim.org/
as an external simulator for ArduPilot Rover, Copter and Plane.

Overview
========

Gazebo is a well-known and respected robotics simulator. Also Gazebo is well-known as official DARPA Virtual Robotics Simulator.
But, no current release has built-in support for ArduPilot(Previous PRs for built-in support didn't merged as of April-2017).

.. warning::

   If it seems that these instructions are outdated, please open an issue on the ardupilot_wiki github.

.. tip::

   Gazebo is particularly useful for defining autonomous
   indoor flights or swarms.


Preconditions
=============

We recommend Ubuntu starting from 16.04 or 18.04 as those are the platform used for testing this approach and are also known to be compatible with SITL.

We will be using a standard version of ArduPilot but a custom plugin for Gazebo, until the gazebo plugin gets merge into Gazebo-master.

This plugin can be use with or without ROS integration. In both case we recommend to use Gazebo from OSRF repository.

::

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt update

The plugin we will be using is working from gazebo 7 to 9. We recommend gazebo9 as the new development will append on this version.
But on older Ubuntu it isn't available so use the correct version for you system.

::

    sudo apt install gazebo9 libgazebo9-dev


The first time gazebo is executed requires the download of some models and it could take some time, please be patient. Let's give a try.

::

    gazebo --verbose

It should have open an empty world.

Plugin installation
===================

The following plugin is a pure Gazebo plugin, so ROS is not needed to use it. You can still use ROS with Gazebo with normal gazebo-ros packages.
We got two version of the plugin : khancyr and SwiftGust one's.
The one from `khancyr <https://github.com/khancyr/ardupilot_gazebo>`__ is the original one. It is stable and only have necessary file to work.
The one from `SwiftGust <https://github.com/SwiftGust/ardupilot_gazebo>`__  got more examples and little bit more documentation.
Both use the same plugin, only the documentation and example provide are changing.

On the following explanation, we will be using khancyr plugin. Clone it somewhere. If you need help on git, please see :ref:`installed git <git-install>`, :ref:`forked <git-fork>` and :ref:`cloned <git-clone>`.

::

    git clone https://github.com/khancyr/ardupilot_gazebo
    cd ardupilot_gazebo
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install


Those instructions will clone the repo into ardupilot_gazebo directory, create a new build directory in it. Then it will compile the plugin and install it on Gazebo plugin directory.

Start the Simulator
===================

In terminal start Gazebo:

::

    gazebo --verbose worlds/iris_arducopter_runway.world


In another terminal, enter the ArduCopter directory and start the SITL simulation:

::

    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -f gazebo-iris --console --map

If all works well, you should see this:

..  youtube:: n_M5Vs5FBGY
    :width: 100%


.. note::

    ROS is commonly used together with Gazebo, but this is out of the scope of this article. If you are using ROS,
    some packages to consider using are:
    - *mavros* (for sending and receiving mavlink packets)
    - *ros_gazebo_camera* (for publishing Gazebo's virtual camera stream to a ROS topic)
