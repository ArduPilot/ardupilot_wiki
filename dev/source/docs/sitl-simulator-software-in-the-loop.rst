.. _sitl-simulator-software-in-the-loop:

=====================================
SITL Simulator (Software in the Loop)
=====================================

.. image:: ../images/sitl.jpg

The SITL (software in the loop) simulator allows you to run Plane,
Copter or Rover without any hardware. It is a build of the autopilot
code using an ordinary C++ compiler, giving you a native executable that
allows you to test the behaviour of the code without hardware.

This article provides an overview of SITL's benefits and architecture.

Overview
========

SITL allows you to run ArduPilot on your PC directly, without any
special hardware. It takes advantage of the fact that ArduPilot is a
portable autopilot that can run on a very wide variety of platforms.
Your PC is just another platform that ArduPilot can be built and run on.

When running in SITL the sensor data comes from a flight dynamics model
in a flight simulator. ArduPilot has a wide range of vehicle simulators
built in, and can interface to several external simulators. This allows
ArduPilot to be tested on a very wide variety of vehicle types. For
example, SITL can simulate:

-  multi-rotor aircraft
-  fixed wing aircraft
-  ground vehicles
-  underwater vehicles
-  camera gimbals
-  antenna trackers
-  a wide variety of optional sensors, such as Lidars and optical flow
   sensors

Adding new simulated vehicle types or sensor types is straightforward.

A big advantage of ArduPilot on SITL is it gives you access to the full
range of development tools available to desktop C++ development, such as
interactive debuggers, static analyzers and dynamic analysis tools. This
makes developing and testing new features in ArduPilot much simpler.

Running SITL
============

The ArduPilot SITL environment has been developed to run natively on both
Linux and Windows. For setup instructions see :ref:`Setting Up SITL <SITL-setup-landingpage>`
for more information. Using SITL is explained in :ref:`Using SITL <using-sitl-for-ardupilot-testing>`. For examples of starting and using SITL for a particular vehicle see :ref:`sitl-examples`. 

Mission Planner (Windows) also provides a simple means of running SITL for the master branch and stable branches of vehicles. See :ref:`planner:mission-planner-simulation`.

SITL Architecture
=================

The SITL executable contains a built-in physics model for each vehicle type, so in most cases no separate simulator process is needed. ``sim_vehicle.py`` starts MAVProxy, which connects to SERIAL0 on TCP port 5760, sends RC input to UDP port 5501 and forwards MAVLink to ground stations on UDP port 14550. SERIAL1 and SERIAL2 listen on TCP ports 5762 and 5763 and can be used for another GCS, a companion computer or a simulated peripheral (see :ref:`SITL Serial Ports <learning-ardupilot-uarts-and-the-console>`).

An external simulator such as JSBSim or a JSON simulator (Gazebo, Webots or a custom script) can replace the built-in physics, and FlightGear can be used to display the vehicle.

The port numbers shown are for the first instance. Each additional instance started with ``-I n`` adds 10 × n to every port, and most ports can be changed on the command line.

.. image:: ../images/sitl-architecture.svg
    :target: ../_images/sitl-architecture.svg

.. toctree::
    :maxdepth: 1

    Setting Up SITL <SITL-setup-landingpage>
    Running SITL on the Autopilot Itself<sim-on-hardware>
    Using SITL <using-sitl-for-ardupilot-testing>
    Examples of using SITL by Vehicle <sitl-examples>
    SITL Serial Ports <learning-ardupilot-uarts-and-the-console>
    SITL Parameter List <sitl-parameters>
