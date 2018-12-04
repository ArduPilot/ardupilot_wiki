.. _sitl-with-morse:

=====================
Using SITL with Morse
=====================

  .. image:: ../images/morse_rover.jpg
    :target: ../_images/morse_rover.jpg

`Morse <http://www.openrobots.org/morse/doc/stable/morse.html>`__ is
an open robotics platform that uses python APIs and the blender 3D
environment to create a complete robotics platform.

ArduPilot has a Morse SITL simulation backend that allows ArduPilot to
control vehicles created within morse.

Installing Morse
================

See instructions here:

http://www.openrobots.org/morse/doc/stable/user/installation.html

For recent Ubuntu Linux installs all you need is

::

   sudo apt install morse-simulator

Builder Scripts
===============

The Morse simulator has the concept of builder scripts, which are
python scripts setup to run within the Morse/Blender environment.

Some example builder scripts for use with ArduPilot are provided here:

https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/Morse

The examples include:

 - a simple rover
 - a simple quadcopter
 - a rover with a scanning laser rangefinder

These scripts setup the standard sensor suite that ArduPilot needs and
exports them using the socket API. It is recommended that you read
through both the rover.py and quadcopter.py examples to better
understand how to interface Morse with ArduPilot.

Running the Examples
====================

The following steps will get you running with the Rover example.

First, make sure you have a copy of the :ref:`ArduPilot code <where-to-get-the-code>`

Then open a terminal and start the rover simulator from within your /ardupilot directory:

::

   morse run libraries/SITL/examples/Morse/rover.py

Open another terminal to /ardupilot directory and start ArduPilot SITL, using the morse-rover simulation backend

::

   sim_vehicle.py -v APMrover2 --model morse-rover --add-param-file=libraries/SITL/examples/Morse/rover.parm --console --map

That will give you something like this:

  .. image:: ../images/morse_rover_sitl.jpg
    :target: ../_images/morse_rover_sitl.jpg

Now for the quadcopter simulator:

::

   morse run libraries/SITL/examples/Morse/quadcopter.py

Then start ArduPilot SITL, using the morse-quad simulation backend

::

   sim_vehicle.py -v ArduCopter --model morse-quad --add-param-file=libraries/SITL/examples/Morse/quadcopter.parm --console --map

That will give you something like this:

  .. image:: ../images/morse_quad.jpg
    :target: ../_images/morse_quad.jpg
             
Notice that in this case it is showing a wire-frame view instead of a
rendered 3D view. That is selected by setting fastmode=True in the
Environment() declaration in quadcopter.py. Using fast mode will lower
CPU usage a lot which is good for slow machines.

Laser Scanner Support
=====================

The Morse SITL backend supports a laser scanner sensor. This allows
you to use the proximity avoidance systems in ArduPilot with vehicles
created in Morse.

There is an example of a Rover setup with a laser scanner in the
rover_scanner.py script. Run it like this:

::

   morse run libraries/SITL/examples/Morse/rover_scanner.py

Then start ArduPilot SITL, using the morse-rover simulation backend

::

   sim_vehicle.py -v APMrover2 --model morse-rover --add-param-file=libraries/SITL/examples/Morse/rover_scanner.parm --console --map

That will give you something like this:

  .. image:: ../images/morse_rover_scanner.jpg
    :target: ../_images/morse_rover_scanner.jpg

The red area shows the extent that the laser scanner proximity sensor is seeing.

Video
-----

..  youtube:: Zk8MYmt03-Q
    :width: 100%
