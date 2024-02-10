.. _sitl-with-gazebo:

======================
Using SITL with Gazebo
======================

`Gazebo <http://gazebosim.org/>`__ is a well known and respected robotics
simulator which has been used in a number of robotics simulation challenges
for ground, marine and space based robots, including the DARPA Robotics
Challenge, DARPA Subterranean Challenge and Virtual RobotX Competition. 

There are two main generations of Gazebo. To use SITL with Gazebo11 and earlier 
versions see the
:ref:`instructions for legacy versions <sitl-with-gazebo-legacy>`.

This article explains how to use the latest generation of Gazebo as an
external simulator for ArduPilot Rover, Coper and Plane.


.. youtube:: rJCEN5Htu2s
    :width: 100%

|

.. tip::

   Gazebo is particularly useful for defining autonomous indoor flights,
   swarms, non-flying vehicle types such as rovers and boats,
   and multi-vehicle scenarios such as the ship landing simulation in the
   video above.


Install ArduPilot SITL
----------------------

Ensure that you have an ArduPilot development environment set up,
including MAVProxy as a GCS.
Verify that you can run SITL examples for Copter and Plane.
The `ArduPilot Development wiki <https://ardupilot.org/dev/index.html>`__
has all the information required.


Install Gazebo
--------------

We currently support
`Gazebo Garden <https://gazebosim.org/docs/garden/install>`__
and
`Gazebo Harmonic <https://gazebosim.org/docs/harmonic/install>`__
which are available for Ubuntu 20.04 (Focal) and 22.04 (Jammy) and 
macOS Big Sur (11), Monterey (12), and Ventura (13).

#. Install Gazebo following the instructions for your platform.

#. Check that the Gazebo installation is working:

    ::

        gz sim -v4 -r shapes.sdf

  This should open a world containing a variety of shapes (on macOS you will
  need to run the Gazebo server and client in separate terminals).


Install the ArduPilot Gazebo Plugin
-----------------------------------

We use a standard version of ArduPilot with a custom plugin for Gazebo which
is hosted on GitHub at: `https://github.com/ArduPilot/ardupilot_gazebo <https://github.com/ArduPilot/ardupilot_gazebo>`__.

.. note::

    While Gazebo is commonly used with ROS / ROS2, the ArduPilot Gazebo plugin
    does not depend on ROS.


.. note::

    The original version of the plugin located at
    `https://github.com/khancyr/ardupilot_gazebo <https://github.com/khancyr/ardupilot_gazebo>`__
    is compatible with the
    `legacy versions of Gazebo <https://classic.gazebosim.org/>`__.

#. Install additional dependencies

    Ubuntu

      For Gazebo garden

      ::

        sudo apt update
        sudo apt install libgz-sim7-dev rapidjson-dev

      For Gazebo Harmonic

      ::

        sudo apt update
        sudo apt install libgz-sim8-dev rapidjson-dev

    macOS

    ::

      brew update
      brew install rapidjson

#. Create a workspace folder and clone the repository

    ::

      mkdir -p gz_ws/src && cd gz_ws/src
      git clone https://github.com/ArduPilot/ardupilot_gazebo

#. Build the plugin

    Set GZ_VERSION environment variable according to installed gazebo version (replace harmonic with garden if required):

    ::

      export GZ_VERSION=harmonic
      cd ardupilot_gazebo
      mkdir build && cd build
      cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
      make -j4


Configure the Gazebo environment
--------------------------------

Gazebo uses a number of environment variables to locate plugins and models
at run time. These may be set in the terminal used to run Gazebo, or set
in your `.bashrc` or `.zshrc` files:

  In a terminal

  ::

    export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
    export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

  In `.bashrc` or `.zshrc`

  ::

    echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
    echo 'export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc


Using Gazebo with ArduPilot
---------------------------

Two models are provided as examples with the plugin: an Iris quadcopter and a
Zephyr delta-wing.

Iris quadcopter
+++++++++++++++

#. Run Gazebo

    ::

      gz sim -v4 -r iris_runway.sdf

#. Run SITL

    ::

      sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

#. Arm and takeoff

    ::

      STABILIZE> mode guided
      GUIDED> arm throttle
      GUIDED> takeoff 5


Zephyr delta-wing
+++++++++++++++++

The Zephyr is positioned for vertical takeoff.

#. Run Gazebo

    ::

      gz sim -v4 -r zephyr_runway.sdf

#. Run SITL

    ::

      sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console

#. Arm, takeoff and circle

    ::

      MANUAL> mode fbwa
      FBWA> arm throttle
      FBWA> rc 3 1800
      FBWA> mode circle


Changing the simulation speed
-----------------------------

By default Gazebo will attempt to run the simulation with a
Real Time Factor (RTF) of 1. To increase the simulation speed add the
following XML block into the world file just after the opening `<world>`
element:

  ::

    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>-1.0</real_time_factor>
    </physics>

Then set the simulation speed-up parameter in MAVProxy

  ::

      MANUAL> param set SIM_SPEEDUP 10


Previous versions
-----------------

.. toctree::
    :maxdepth: 1

    Using SITL with legacy versions of Gazebo <sitl-with-gazebo-legacy>
