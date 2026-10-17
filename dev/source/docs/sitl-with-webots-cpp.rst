.. _sitl-with-webots-cpp:

============================
Using SITL with Webots C/C++
============================

.. youtube:: 68NUISDvL0M
    :width: 100%


`Webots <https://cyberbotics.com/>`__ is a simulator mainly used for robotics. It is easy to build many vehicles using it. ArduPilot has Rover, Quadcopter, and Tricopter examples that have been built especially for this simulator.


You can download Webots simulator from `www.cyberbotics.com <https://www.cyberbotics.com/#download/>`__. To run Webots just type webots in command line.


Installing Webots
=================

The worlds and controllers are saved as Webots R2025a and were verified against that release. Older worlds (for example your own R2021 worlds) can be converted with ``tests/migrate_worlds.py`` (see below).

On Ubuntu the Webots .deb may leave ``libsndio7.0`` unsatisfied, which makes ``webots`` fail to start with ``error while loading shared libraries: libsndio.so.7``. ``sudo apt install libsndio7.0`` fixes it.

The first time a world is opened, Webots downloads the PROTO assets named in its ``EXTERNPROTO`` lines. This takes a little while; later runs are cached.

Running the Examples
====================

#. Open Webots using the command line.
#. Open a world from the `worlds folder <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/Webots/worlds>`__ such as ``webots_quadX.wbt``.
#. Build the robot controller and the WorldInfo physics plugin if Webots has not already done so (it builds them when loading a world, or open ``ardupilot_SITL_QUAD.c`` and ``sitl_physics_env.c`` and use ``Build > Build``). Alternatively set ``WEBOTS_HOME`` and run ``make`` in each controller directory. More info about building with Webots can be found `here <https://cyberbotics.com/doc/guide/webots-built-in-editor>`__.
#. Press Run on the Webots GUI. Now the simulator is running.
#. From the ArduPilot repository root, run SITL using the script that matches the world, for example:

::

   ./libraries/SITL/examples/Webots/run_quadX.sh
   or
   ./Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-quad:127.0.0.1:5577 --add-param-file=libraries/SITL/examples/Webots/quadX.parm

Each world has a launch script named after it (the ``webots_`` prefix is dropped):

=============================== ================================== ====================
World                           Script                             Controller port(s)
=============================== ================================== ====================
``webots_quadX.wbt``            ``run_quadX.sh``                   5577
``webots_quadPlus.wbt``         ``run_quadPlus.sh``                5599
``webots_tricopter.wbt``        ``run_tricopter.sh``               5599
``webots_rover.wbt``            ``run_rover.sh``                   5599
``pyramidMap.wbt``              ``run_pyramidMap.sh``              5599
``webots_two_quadX.wbt``        ``run_two_quadX.sh``               5599, 5598
``webots_two_tricopter.wbt``    ``run_two_tricopter.sh``           5599, 5598
``pyramidMap_two_quads.wbt``    ``run_pyramidMap_two_quads.sh``    5599, 5598
=============================== ================================== ====================

The multi-vehicle scripts open one xterm per vehicle and send MAVLink to UDP 14450 (vehicle 1) and 14550 (vehicle 2). Build once first with ``./waf copter``.

You can stop and re-run ArduPilot SITL without restarting Webots: the controller waits for the next SITL connection and puts the vehicle back at its start pose. While no SITL is connected the simulation is paused, since it only advances in step with SITL.

.. note::

   Start Webots before ArduPilot's SITL, otherwise the socket connection may not establish.

.. note::

   In the two-vehicle worlds, wind and drag from the ``sitl_physics_env`` plugin act on the first vehicle only.

Rotor Model
===========

Webots drives each propeller with a ``Propeller`` node, where thrust and torque are proportional to the square of the rotor speed. The controllers command ``omega = sqrt(throttle) * maxVelocity`` so that thrust is linear in ArduPilot's motor output, and the bundled ``.parm`` files therefore set ``MOT_THST_EXPO 0``.

Webots cannot report a propeller's real shaft speed, so the controllers send estimated rotor speeds to SITL (in servo channel order), which allows RPM logging and the RPM-driven harmonic notch (``RPM1_TYPE 10``) to be exercised.

After changing a vehicle's mass or number of rotors, recalibrate the rotor constants with:

::

   python3 libraries/SITL/examples/Webots/tests/migrate_worlds.py calibrate webots_quadX.wbt

``migrate_worlds.py status`` summarises every world, and ``migrate_worlds.py migrate`` converts a world saved by an older Webots version. Both ask before changing anything.

Simulation Parameters
=====================

There are two types of parameters, the first type is passed to SITL and the second is configured in Webots.

SITL communicates with Webots using TCP sockets, so SITL should have the same port number as Webots. Usually you don't need to change the default port 5599, but when you run multiple robots you will need to use different ports for each simulation instance.



Advantage of Webots
===================

#. Controllers can be written in c, c++, python, and MatLab.
#. A lot of sensors exist.
#. Ability to add custom physics to simulate things such as wind.
#. Ability to add OpenStreetMap and run the simulator in environments very similar to reality. 


How to Connect Your Own World with ArduPilot
============================================

WorldInfo
~~~~~~~~~

The following parameters are important to be set in WorldInfo

- Field *"basicTimeStep"* = 1 or 2

- Field *"physics"* is used to attach the physics plugin file *"sitl_physics_env"*  which is used to simulate wind and drag in a very simple way.


Vehicle Robot
~~~~~~~~~~~~~
You can copy & paste the vehicle robot multiple times, each time you need to:

#. give the new robot a new name.
#. Field *"CustomData"* should be equal to robot index number i.e. 1,2,3,...etc.
#. Field *"controller"* should is selected based on the vehicle used.
#. Field *"controllerArgs"* specifies many important factors, mainly the TCP port which should be equal to SITL's TCP port that will connect to this robot in the simulator. The multirotor controllers also accept ``-df <drag factor>`` and ``-mv <rad/s>`` (cap on rotor speed); the rover controller accepts ``-ms <m/s>`` and ``-sa <rad>`` (speed and steering angle at full scale).
#. Robots must be supervisors (``supervisor TRUE``) so they can be reset when SITL restarts.
#. World files must declare an ``EXTERNPROTO`` line for every PROTO node they use; R2025a skips undeclared nodes (a world without its terrain makes the vehicle fall forever).


