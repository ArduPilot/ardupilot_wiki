.. _sitl-with-xplane:

==========================
Using SITL with X-Plane 10
==========================

.. figure:: ../images/xplane-pt60.jpg
   :target: ../_images/xplane-pt60.jpg

This article describes how to use X-Plane 10 as a simulation backend for
ArduPilot :ref:`SITL <sitl-simulator-software-in-the-loop>`.

.. youtube:: llRii8hmG1M
    :width: 100%
       
Overview
========

X-Plane 10 is a commercial flight simulator with a rich networking
interface that allows it to be interfaced to other software. In this
case we will be interfacing it to the ArduPilot SITL system, allowing
ArduPilot to fly a wide variety of aircraft.

Using X-Plane with SITL is a good way to get some experience flying
ArduPilot and learning how to use the ground control station. It can
also be used to see how ArduPilot handles unusual aircraft and to
develop support for aircraft features that may not be available in
other simulator backends.

Setup of X-Plane 10
===================

Before starting SITL the only thing you need to setup on X-Plane is
the network data to send the sensor data to the IP address of the
computer that will run ArduPilot. This can be the same computer that
is running X-Plane (in which case you should use an IP address of
127.0.0.1) or it can be another computer on your network.

Go to the Settings -> Net Connections menu in X-Plane and then to the
Data tab. Set the right IP address, and set the destination port
number as 49001. Make sure that the receive port is 49000 (the
default). If using loopback (ie. 127.0.0.1) then you also need to make
sure the "port that we send from" is not 49001. In the example below
49002 is used.

.. figure:: ../images/xplane-network-data.jpg
   :target: ../_images/xplane-network-data.jpg

If you have a joystick then you can configure the joystick for
X-Plane. A joystick controlled by X-Plane will be available as R/C
input when ArduPilot is in control of X-Plane, allowing you to fly the
aircraft with the joystick in ArduPilot flight modes.

For joystick setup go to Settings -> Joystick and Equipment. You
should setup controls for roll, pitch, yaw and throttle. Note that
X-Plane has an unusual throttle setup where the bar is fully to the
left at full throttle and fully to the right at zero throttle.

.. figure:: ../images/xplane-joystick-setup.jpg
   :target: ../_images/xplane-joystick-setup.jpg

Right now you can't use the joystick for other that basic axes
controls, so you can't use it for flight mode changes. We may be able
to add support for that in the future.

Starting SITL
=============

There are three approaches to starting SITL with X-Plane depending on
what you are wanting to do.

  - running SITL from within MissionPlanner on Windows
  - building SITL yourself and connecting from your favourite GCS
  - building and running SITL using sim_vehicle.py and MAVProxy

The first approach is good if you just want to test ArduPilot with
SITL but you don't want to make changes to the code. MissionPlanner
will download a build of ArduPilot SITL for Windows that is built each
night from git master.

The second approach is good if you want to do ArduPilot development
and try out code changes and you want to use a ground station of your
choice. Any ground station that supports MAVLink over TCP can be used.

The third approach is good if you want the full capabilities of
MAVProxy for ArduPilot SITL testing. MAVProxy has a rich graphing and
control capability that is ideal for long term ArduPilot software
development.

Using SITL from MissionPlanner
------------------------------

To start SITL directly from MissionPlanner you need to have a very
recent version of MissionPlanner. Get the latest beta from the help
screen.

Then go the SIMULATION tab:

.. figure:: ../images/xplane-missionplanner1.jpg
   :target: ../_images/xplane-missionplanner1.jpg

In the SIMULATION tab select X-plane and Xplane 10. Then select
Advanced IP Settings an click through the IP addresses, set them 
to 127.0.0.1, with the default network ports.

Press "Start SITL"

.. figure:: ../images/xplane-missionplanner2.jpg
   :target: ../_images/xplane-missionplanner2.jpg

In the SITL screen you need to select Model "xplane" and then select
"Plane". At the moment we only support fixed wing and helicopter
aircraft in X-Plane with SITL. In the future we may support other
aircraft types. See below for more information on flying a helicopter.

When you select "Plane" MissionPlanner will download a nightly build
of ArduPilot SITL and will then launch SITL.

.. figure:: ../images/xplane-missionplanner3.jpg
   :target: ../_images/xplane-missionplanner3.jpg

You then need to load an appropriate set of parameters for the
aircraft (or setup the aircraft just like you would a real aircraft)
and enjoy flying as usual with MissionPlanner.

When setting up the aircraft it is useful to use the joystick to move
the control surfaces to make sure they are all going the right
way. You can change channel direction in the normal way with ArduPilot
parameters.


Using SITL with your own GCS
----------------------------

The second approach to running X-Plane 10 with SITL is to build
ArduPilot SITL manually and then run it from the cygwin command
line. You can then connect with your favourite GCS.

You should checkout the latest ArduPilot git tree in cygwin, and then
change directory to the top "ardupilot" directory. Then run the
following commands::

  $ modules/waf/waf-light configure --board sitl
  $ modules/waf/waf-light plane
  $ build/sitl/bin/arduplane --model xplane

.. figure:: ../images/xplane-waf.jpg
   :target: ../_images/xplane-waf.jpg

That will start SITL and wait for a GCS to connect. You should connect
on TCP port 5760 and configure ArduPilot as usual.

Using SITL with sim_vehicle.py
------------------------------

The sim_vehicle.py script gives you a lot of options for launching all
of the different simulation systems that work with ArduPilot,
including X-Plane 10.

To use sim_vehicle.py you will need to install MAVProxy. If you are on
Linux then make sure pip is installed and run::

  $ pip install --upgrade pymavlink mavproxy
  
If you are on Windows then download and install MAVProxy from
https://firmware.ardupilot.org/Tools/MAVProxy/

Then do a git checkout of ArduPilot master and change directory to the
ArduPlane directory. I like to create a sub-directory for each
aircraft I fly in SITL so that settings are remembered
per-aircraft. If you want to do that then create a subdirectory in the
ArduPlane directory and run sim_vehicle.py from there. In the
following example I will be using the PT60 aircraft in X-Plane, so I
create a PT60 directory::

 $ cd ArduPlane
 $ mkdir PT60
 $ cd PT60
 $ sim_vehicle.py -D -f xplane --console --map


Flying a Helicopter
-------------------

It is also possible to fly a helicopter with XPlane-10. The setup is
similar to a plane, with two additional requirements:

  - you need to setup your XPlane joystick to map the collective stick
    to flaps
  - you need to map a key or joystick button to turn on and off the
    "generator1" electrical system

These strange requirements are because of limitations in the remote
control of helicopters in X-Plane 10. The flaps input is something
that ArduPilot SITL is able to read remotely while not interfering
with flight of the helicopter. The "generator1 on/off" is used to
simulate the interlock switch (channel 8) in ArduPilot helicopter
support.

Note that for "generator on/off" you do need to map two separate
events, one for on and one for off. If using a two position switch
then map one to the switch on position and the other to the switch off
position.

See this example for typical joystick setup

.. figure:: ../images/xplane-heli-joystick1.jpg
   :target: ../_images/xplane-heli-joystick1.jpg

and this one for mapping the generator on/off switch to a joystick
switch

.. figure:: ../images/xplane-heli-joystick2.jpg
   :target: ../_images/xplane-heli-joystick2.jpg

A full set of parameters for the Bell JetRanger Helicopter in X-Plane
10 are available here http://uav.tridgell.net/XPlane/

You also need to start SITL with the model set to "xplane-heli"
instead of "xplane" to activate Helicopter controls.

The startup procedure for a helicopter is:

  - set interlock on (so RC input channel 8 is low)
  - set zero collective (so RC input channel 3 is low)
  - arm the helicopter
  - set interlock off (so RC input channel 8 is high)
  - wait for the head to reach full speed
  - takeoff

.. youtube:: JNNSoMrAFn4
    :width: 100%
