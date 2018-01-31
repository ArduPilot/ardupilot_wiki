.. _using-gazebo-simulator-with-sitl:

================================
Using Gazebo Simulator with SITL
================================

This article explains how to use Gazebo http://gazebosim.org/
as an external simulator for Ardupilot Rover, Copter and Plane.

Overview
========

Gazebo is a well-known and respected robotics simulator. Also Gazebo is well-known as official DARPA Virtual Robotics Simulator.
But, no current release has built-in support for ArduPilot(Previous PRs for built-in support didn't merged as of April-2017).
New instruction for setting up gazebo simulator for STIL is available at https://github.com/swiftgust/ardupilot_gazebo. 
This wiki will be updated more gazebo support becomes available.


.. warning::

   Gazebo support is still under development.
   If it seems that these instructions are outdated, please open an issue on the ardupilot_wiki github.

.. tip::

   Gazebo is particularly useful for defining autonomous
   indoor flights or swarms.


Preconditions
=============

We recommend Ubuntu starting from 16.04.2LTS (it is untested on 16.10) as this is the platform used for testing
this approach and is also known to be compatible with SITL.

We will be using a standard version of ArduPilot but a custom fork of Gazebo, until the gazebo plugin gets merge into Gazebo-master.

These instructions are derived from http://gazebosim.org/tutorials?tut=install_from_source

Setup your computer to accept software from packages.osrfoundation.org.

::

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install build-essential cmake git libboost-all-dev mercurial libcegui-mk2-dev libopenal-dev libswscale-dev libavformat-dev libavcodec-dev  libltdl3-dev libqwt-dev ruby libusb-1.0-0-dev libbullet-dev libhdf5-dev libgraphviz-dev libgdal-dev
    
Install dependencies
::

    wget https://bitbucket.org/osrf/release-tools/raw/default/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
    ROS_DISTRO=dummy . /tmp/dependencies.sh
    sudo apt-get install $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPENDENCIES) $(sed 's:\\ ::g' <<< $BASE_DEPENDENCIES)

Remove old Gazebo versions
::

    sudo apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*'
    
Make a gazebo workspace
::

    mkdir ~/gazebo_ws
    cd gazebo_ws/
    
Compiling and installing Gazebo From Source
===========================================
.. warning::

   Following Installing gazebo pre-built packages guides below is recommended.
   This guide is outdated, you will be able to compile Ignition Maths, Msgs, Tools. 
   But, installing SDFormat and Gazebo requires some fix at the moment.
   
Build and install Ignition Maths
   
::

    hg clone https://bitbucket.org/ignitionrobotics/ign-math ~/gazebo_ws/ign-math
    cd ~/gazebo_ws/ign-math
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j4
    sudo make install

Build and install Ignition Msgs
::

    sudo apt-get install libprotobuf-dev protobuf-compiler libprotoc-dev libignition-math4-dev
    hg clone https://bitbucket.org/ignitionrobotics/ign-msgs ~/gazebo_ws/ign-msgs
    cd ~/gazebo_ws/ign-msgs
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j4
    sudo make install

Build and install Ignition Tools
::

    hg clone https://bitbucket.org/ignitionrobotics/ign-tools ~/gazebo_ws/ign-tools
    cd ~/gazebo_ws/ign-tools
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j4
    sudo make install

Build and install SDFormat
::

    hg clone https://bitbucket.org/osrf/sdformat ~/gazebo_ws/sdformat
    cd ~/gazebo_ws/sdformat
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j4
    sudo make install

Build and install Gazebo
::

    hg clone https://bitbucket.org/osrf/gazebo ~/gazebo_ws/gazebo
    cd ~/gazebo_ws/gazebo
    hg checkout ardupilot
    mkdir build
    cd build
    cmake ../
    make -j4 # NOTE: This will take a long time!
    sudo make install
    
Now try running Gazebo by typing `gazebo`. If it works, you're done. If it says

::

    gazebo: error while loading shared libraries: libgazebo_common.so.1: cannot open shared object file: No such file or directory

Then find the file libgazebo_common.so.1, probably under `/usr/local/lib` or `/usr/local/lib/x86_64-linux-gnu`, and then add it like so:

::

    echo '<insert directory here>' | sudo tee /etc/ld.so.conf.d/gazebo.conf
    sudo ldconfig

.. note::

    Compiling Gazebo from source will not be necessary once this pull request gets merged:
    https://bitbucket.org/osrf/gazebo/pull-requests/2450/ardupilot-refactor-and-minor-improvements/diff

Installing gazebo pre-built packages
====================================

Instead of building gazebo yourself you can instead install a prebuilt
set of packages if you are running a suitable distro of Linux.

Instructions for installing the "SASC" version of gazebo that works
with ArduPilot SITL are here:

  https://github.com/osrf/uctf/tree/master/doc/install_binary

That will install gazebo in /opt/sasc, so you need to also do:

::

   export PATH=/opt/sasc/bin:$PATH

and install the custom gazebo models using the instructions below.
  
Installing Custom Gazebo Models
-------------------------------

We will also need to get a gazebo model of a quadcopter.

::

    hg clone https://bitbucket.org/osrf/gazebo_models ~/gazebo_ws/gazebo_models
    cd ~/gazebo_ws/gazebo_models
    hg checkout zephyr_demos
    echo 'export GAZEBO_MODEL_PATH=~/gazebo_ws/gazebo_models' >> ~/.bashrc
    source ~/.bashrc

.. note::

    This step will not be necessary once this pull request gets merged:
    https://bitbucket.org/osrf/gazebo_models/pull-requests/221/zephyr_demos/diff


Set up PATH to build tools
--------------------------

If you have not already done so, you need to set up the PATH to the build tools
(located in **/ardupilot/Tools/autotest**) so that the build system can find
**sim_vehicle.py**.

Navigate the file system to the home directory and open the **.bashrc**
file. Add the following line to the end of **.bashrc**:

::

    export PATH=$PATH:$HOME/ardupilot/Tools/autotest

.. note::

   Use your own path to ardupilot folder in the line above!

Start the Simulator
===================

In one terminal, enter the ArduCopter directory and start the SITL simulation:

::

    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -f gazebo-iris -D --console --map

In another terminal start Gazebo:

::

    cd ~/gazebo
    gazebo --verbose worlds/iris_arducopter_demo.world

If all works well, you should see this:

..  youtube:: n_M5Vs5FBGY
    :width: 100%



.. note::

   If you get error from gazebo [Server.cc:376] Could not open file [wolrds/iris_arducopter_demo.world]
   you can download it here
   https://bitbucket.org/osrf/gazebo/pull-requests/2450/ardupilot-refactor-and-minor-improvements/diff
   and copy it to /usr/share/gazebo-7/worlds

.. note::

    ROS is commonly used together with Gazebo, but this is out of the scope of this article. If you are using ROS,
    some packages to consider using are:
    - *mavros* (for sending and receiving mavlink packets)
    - *ros_gazebo_camera* (for publishing Gazebo's virtual camera stream to a ROS topic)
