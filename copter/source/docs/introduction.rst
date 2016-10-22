.. _introduction:

==================
Introducing Copter
==================

Copter is an advanced open-source autopilot system for multicopters,
helicopters, and other rotor vehicles.

Overview
========

Copter is a complete open-source autopilot solution for multi-rotor
vehicles, offering both enhanced remote control flight (via a number of
intelligent flight modes) and execution of fully autonomous missions.

As part of the wider ArduPilot software platform it
works seamlessly with Ground Control Station software that can monitor
vehicle telemetry and perform powerful mission planning activities. It
also benefits from other parts of the Ardupilot ecosystem, including
simulators, log analysis tools, and higher level APIs for vehicle
management and control.

Copter is on the cutting edge of aerial robotics and intended for those
people who want to try advanced technology, leading edge techniques and
new flight styles. It is already a preferred platform for numerous
commercially available ready-to-fly vehicles, and can easily be added to
enhance your own DIY multirotor craft.

.. image:: ../images/copter-introduction-diagram.jpg
    :target: ../_images/copter-introduction-diagram.jpg

Key features
============

Key features include:

-  *High quality auto-level and auto-altitude control*: Fly level and
   straight or use the awesome "simple" or "super simple" flight modes, which make Copter
   one of the easiest multicopters to fly.

   Don't worry about keeping an eye on your multicopter's orientation -
   just push the stick the way you want to go, and the autopilot figures
   out what that means for whatever orientation the copter is in, using
   its onboard compass. "Front", "back" ... who cares!
-  *Automatic takeoff and landing*: Flick a switch and watch Copter
   execute its mission completely autonomously, returning home and
   landing by itself when it's done.
-  *"Loiter" mode*: Copter will hold its position using its GPS and
   altitude sensors.
-  *Return to launch*: Flip a switch to have Copter fly back to the
   launch location automatically.
-  *Fail safety:* Automatically detect when the vehicle loses
   transmitter contact (or is outside a defined geofence) and return to
   the launch point. Will also attempt to land safely if hardware
   failures are detected.
-  *No programming required*: Use the desktop *Mission Planner* software
   to load the autopilot (with just one click) and set up Copter. The
   Mission Planner (and other compatible ground stations) deliver visual
   displays for vehicle state, settings and telemetry, including a
   point-and-click mission planning interface.
-  *Missions with hundreds of GPS waypoints*: Just point and click
   waypoints in a Mission Planner, and Copter will fly itself to them.
   You can automate entire missions, including camera control! The only
   endurance limits are those of your vehicle power supply.
-  *Mission planning while in flight*: Using a two-way wireless
   connection, waypoints, mode changing, even changing the values of
   every control parameter can be done from your laptop or mobile device
   - even while Copter is in the air!

Getting started
===============

If you're using Copter on a ready-to-fly vehicle then it is likely that
it will be already setup, configured and tuned, ready for your first
flight. We recommend you *read your manufacturer's instructions*,
particularly those related to safety, before flying.

Once you're familiar with the default setup of your vehicle you may want to
configure your RC transmitter/vehicle to use more challenging :ref:`flight modes <flight-modes>`, or :ref:`choose a ground station <common-choosing-a-ground-station>` and start flying automated
missions.

.. tip::

   Whether using an RTF or DIY vehicle, autonomous vehicles are
   potentially dangerous! Always follow :ref:`best safety practices <safety-multicopter>` and pay close attention to all safety
   warnings.

If you're working on a DIY project, this wiki has everything you need!
You should start by reading this section in order to understand what a
multicopter can do, and how to select a frame, flight controller board,
and other essential components. You can then proceed to :ref:`First Time Setup <initial-setup>` to learn how to assemble your Copter and then
:ref:`First Flight <flying-arducopter>` to learn how to configure and tune
it.

The development team
====================

Copter is developed and maintained by a dedicated group of volunteers
from the open source community. Follow their continuing efforts and read
about new project developments at
`ArduPilot's Discuss Server <http://discuss.ardupilot.org/c/arducopter>`__.

*All of us involved with this project care a great deal about the
privacy and safety of those whom we share this planet with. Please be a
good steward of this technology. It is the product of many evenings and
weekends, we make it available for benevolent use.*

Learn more about Copter
=======================

To find out more about Copter and your main configuration decisions,
please see the topics below:


.. toctree::
    :maxdepth: 1

    How Multicopters Work <what-is-a-multicopter-and-how-does-it-work>
    What You’ll Need <what-you-need>
    MultiCopter Safety <safety-multicopter>
    Choosing a MultiCopter Frame <choosing-a-frame>
    Choosing a Flight Controller <common-choosing-a-flight-controller>
    Choosing a Ground Station <common-choosing-a-ground-station>
    Use-Case Overview <copter-use-case-overview>
