.. _using-gazebo-simulator-with-sitl:

================================
Using Gazebo Simulator with SITL
================================

This article explains how to use Gazebo <http://gazebosim.org/>`__
as an external simulator for Copter.

Overview
========

Gazebo is a well-known and respected robotics simulator. We will be compiling 
Gazebo from source, because no current release has built-in support for ArduCopter.

.. tip::

   Gazebo is particularly useful for defining autonomous
   indoor flights.


Preconditions
=============

We recommend Ubuntu 14.04.2 as this is the platform used for testing
this approach and is also known to be compatible with SITL.

Compiling and installing Gazebo
===============================

We will be using a standard version of ArduPilot, but a custom fork of Gazebo.

These instructions are derived from <http://gazebosim.org/tutorials?tut=install_from_source>`__.

::
    # Remove old Gazebo versions
    sudo apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*'

    # Install dependencies
    wget https://bitbucket.org/osrf/release-tools/raw/default/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
    ROS_DISTRO=dummy . /tmp/dependencies.sh
    sudo apt-get install $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPENDENCIES) $(sed 's:\\ ::g' <<< $BASE_DEPENDENCIES)

    # Skip this part if using Ubuntu version 14.10 or newer
    sudo apt-add-repository ppa:libccd-debs
    sudo apt-add-repository ppa:fcl-debs

    # Install more dependencies
    sudo apt-add-repository ppa:dartsim
    sudo apt-get update
    sudo apt-get install libdart-core5-dev

    # Build and install Ignition
    hg clone https://bitbucket.org/ignitionrobotics/ign-math /tmp/ign-math
    cd /tmp/ign-math
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j4
    sudo make install

    # Build and install SDFormat
    hg clone https://bitbucket.org/osrf/sdformat /tmp/sdformat
    cd /tmp/sdformat
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j4
    sudo make install

    # Build and install Gazebo
    cd ~
    hg clone https://bitbucket.org/osrf/gazebo
    cd gazebo
    hg checkout aero_default
    mkdir build
    cd build
    cmake ../
    make -j4 # NOTE: This will take a long time!
    sudo make install

    # Add some things to your path
    echo "export LD_LIBRARY_PATH=<install_path>/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
    echo "export PATH=<install_path>/local/bin:$PATH" >> ~/.bashrc
    echo "export PKG_CONFIG_PATH=<install_path>/local/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
    source ~/.bashrc
    
Now try running Gazebo by typing `gazebo`. If it works, you're done. If it says

::

    gazebo: error while loading shared libraries: libgazebo_common.so.1: cannot open shared object file: No such file or directory

Then find the file libgazebo_common.so.1, probably under `/usr/local/lib` or `/usr/local/lib/x86_64-linux-gnu`, and then add it like so:

::

    echo '<insert directory here>' | sudo tee /etc/ld.so.conf.d/gazebo.conf
    sudo ldconfig

.. note::

    Compiling Gazebo from source will not be necessary once this pull request gets merged:
    <https://bitbucket.org/osrf/gazebo/pull-requests/2264/aerodynamic-updates/diff>`__


Installing Custom Gazebo Models
-------------------------------

We will also need to get a gazebo model of a quadcopter.

::

    cd ~
    hg clone https://bitbucket.org/osrf/gazebo_models
    cd gazebo_models
    checkout aero_testing_john
    echo 'export GAZEBO_MODEL_PATH=~/gazebo_models' >> ~/.bashrc
    source ~/.bashrc

.. note::

    This step will not be necessary once this pull request gets merged:
    <https://bitbucket.org/osrf/gazebo_models/pull-requests/177/aerodynamics-models/diff>`__


Set up PATH to build tools
--------------------------

If you have not already done so, you need to set up the PATH to the build tools
(located in **/ardupilot/Tools/autotest**) so that the build system can find
**sim_vehicle.sh**.

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
    sim_vehicle.sh -f gazebo

In another terminal start Gazebo:

::

    cd ~/gazebo
    gazebo --verbose worlds/iris_standoff_demo.world

If all works well, you should see this:

..  youtube:: n_M5Vs5FBGY
    :width: 100%

.. note::

    ROS is commonly used together with Gazebo, but this is out of the scope of this article. If you are using ROS,
    some packages to consider using are:
    - *mavros* (for sending and receiving mavlink packets)
    - *ros_gazebo_camera* (for publishing Gazebo's virtual camera stream to a ROS topic)
