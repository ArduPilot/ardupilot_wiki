.. _sitl-with-webots-cpp:

============================
Using SITL with Webots C/C++
============================

.. youtube:: c5CJaRH9Pig
    :width: 100%


`Webots <https://cyberbotics.com/>`__ is a simulator mainly used for robotics. It is easy to build many vehicles using it. ArduPilot has Rover, Quadcopter, and Tricopters examples that have been built especially for this simulator.


You can download Webots simulator from `www.cyberbotics.com <https://www.cyberbotics.com/#download/>`__. To run Webots just type webots in command line.


Running the Examples
====================

#. Open Webots using the command line.
#. Download and open a vehicle from the `vehicle examples folder <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/Webots/worlds>`__   such as  ``webots_quadPlus.wbt``.
#. Compile the Robot Controller and WorldInfo physics plugin using Webots GUI by opening their respective source files (`ardupilot_SITL_QUAD.c <https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/Webots/controllers/ardupilot_SITL_QUAD/ardupilot_SITL_QUAD.c>`__ and `sitl_physics_env.c <https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/Webots/plugins/physics/sitl_physics_env/sitl_physics_env.c>`__) and running ``Build > Build``. More info about building with Webots can be found `here <https://cyberbotics.com/doc/guide/webots-built-in-editor>`__.
#. Press Run on Webots GUI. Now the simulator is running.
#. Run ArduPilot's SITL using available scripts in the `examples folder <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/Webots>`__. 

::

   ./libraries/SITL/examples/Webots/dronePlus.sh
   or
   ./Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-quad:127.0.0.1:5599 --add-param-file=libraries/SITL/examples/Webots/quadPlus.parm


.. warning::

   It is important to run Webots before ArduPilot's SITL , otherwise the socket connection may not establish.
   
   As of writing, these instructions work with Webots 2021b. Newer versions may or may not work correctly.

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
#. Field *"controllerArgs"* specifies many important factors, mainly the TCP port which should be equal to SITL's TCP port that will connect to this robot in the simulator.


