.. _home:

=========================================
Welcome to the ArduPilot Development Site
=========================================

.. tip::

    Keep up with the latest ArduPilot related blogs on `ArduPilot.org! <http://ardupilot.org/>`__

:ref:`ArduPilot <ardupilot:home>` (sometimes known as APM) is the leading open source autopilot
system supporting multi-copters, traditional helicopters, fixed wing aircraft, rovers, submarines and antenna trackers.

We pride ourselves on being **versatile** (rich in features with support for a large number of flight controllers, sensors and frame types), **trusted** (reliable and predictable) and **open** (both in terms of software and in our team's organisation and governance).

The source code is developed by a group of voluteer and profession (i.e. paid) developers who, along with our users and `Partners <http://ardupilot.org/about/Partners>`__, make up the `ArduPilot Community <http://ardupilot.org>`__.

Getting Involved
================

New developers are always welcome! The best way to start is to:

- read this wiki to learn the basics of the software and :ref:`how the team works <how-the-team-works>`
- get involved with the other developers by posting on the `Developer Team Forum <http://discuss.ardupilot.org/c/development-team>`__, chat to us on `Gitter <https://gitter.im/ArduPilot/ardupilot>`__ or join the :ref:`weekly development call <ardupilot-mumble-server>`.  You can also find a large number of users and some developers in the `ArduPilot facebook group <https://www.facebook.com/groups/ArduPilot.org>`__.  :ref:`All channels <common-contact-us>` are open to all.  Lurk for a while to get a feel for it, then participate!
- find a specific bug you'd like to fix or a feature you'd like to add (check out the `good first issues <https://github.com/ArduPilot/ardupilot/issues?q=is%3Aopen+is%3Aissue+label%3A%22good+first+issue%22>`__, recent `issues from Randy <https://github.com/ArduPilot/ardupilot/issues/created_by/rmackay9>`__ or our :ref:`roadmap <roadmap>` for ideas).
- fix the bug in your own clone and :ref:`test <simulation-2>` that it's working
- submit the change to the main code base :ref:`via a pull request <submitting-patches-back-to-master>`.

Why the name?
=============

The 'Ardu' part of the ArduPilot name comes from `Arduino <http://www.arduino.cc/>`__. The original :ref:`APM1 and APM2 <common-apm25-and-26-overview>` boards were based around the
Arduino development environment and AVR CPUs. We long ago outgrew these boards so we recommend users use one of the many more capable boards found on our :ref:`Autopilot Hardware Options page <common-autopilots>` including the Pixhawk.

A timeline history of ArduPilot can be found :ref:`here <history-of-ardupilot>`.

Supported boards
================

The :ref:`Autopilot Hardware Options <common-autopilots>` page provides an overview for all the supported controller boards, including
:ref:`Pixhawk <common-pixhawk-overview>`, :ref:`The Cube <common-thecube-overview>`,
:ref:`Pixracer <common-pixracer-overview>`, :ref:`NAVIO2 <common-navio2-overview>`, :ref:`Bebop2 <copter:parrot-bebop-autopilot>`, etc.

To get going quickly please consider purchasing one of the :ref:`ready-to-fly vehicles <common-rtf>` including the very low-cost `SkyRocket/SkyViper drone <http://ardupilot.org/copter/docs/skyrocket.html>`__.

The ArduPilot source code includes the `AP-HAL <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL>`__
hardware abstraction layer, making it relatively easy to port the code to a
wide range of autopilot boards. See this `blog post <http://diydrones.com/profiles/blogs/lots-of-changes-to-apm-development>`__
for more information on the move to AP-HAL.

Project List
============

The ArduPilot system is made up of (or relies upon) several
different projects which are listed below.  Those marked with an asterix
(\*) are peer projects that have their own owners outside the core
ArduPilot dev team.

-  Plane (`wiki <http://plane.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/ardupilot>`__) - autopilot for
   planes
-  Copter (`wiki <http://copter.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/ardupilot>`__) - autopilot for
   multicopters and traditional helicopters
-  Rover (`wiki <http://rover.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/ardupilot>`__) - autopilot for
   ground vehicles
-  Antenna Tracker (`wiki <http://ardupilot.org/antennatracker/index.html>`__,
   `code <https://github.com/ArduPilot/ardupilot>`__) - for automatically aiming an antenna at a vehicle
-  Mission Planner (`wiki <http://planner.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/MissionPlanner>`__) - the most
   commonly used ground station written in C# for windows but also runs
   on Linux and MacOS via mono
-  APM Planner 2.0 (`wiki <http://planner2.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/apm_planner>`__) is a ground
   station specifically for APM written in C++ using the Qt libraries
-  `MAVProxy <http://ardupilot.github.io/MAVProxy/html/index.html>`__
   - command line oriented and scriptable ground station (mostly used by developers)
-  `DroneKit <http://dronekit.io/>`__ - APM SDK for apps running on vehicles, mobile devices and/or in the cloud.
-  MinimOSD (`wiki <http://code.google.com/p/arducam-osd/wiki/minimosd>`__,
   `code <https://github.com/diydrones/MinimOSD-Extra>`__)
   - on-screen display of flight data
-  Tower (`wiki <https://github.com/DroidPlanner/Tower/wiki>`__,
   `code <https://github.com/DroidPlanner/Tower>`__, `google play <https://play.google.com/store/apps/details?id=org.droidplanner.android>`__)
   - android ground station
-  `QGroundControl* <http://www.qgroundcontrol.org/>`__ is an alternative ground station written in C++ using the Qt libraries
-  `PX4* <https://pixhawk.org/start>`__ - designers of the original PX4FMU hardware (from which the Pixhawk was developed) and owners of a relatively small number of drivers we use when running on a Pixhawk flight controller board
-  `MAVLink* <http://www.qgroundcontrol.org/mavlink/start>`__ -
   the protocol for communication between the ground station, flight
   controller and some periphers including the OSD. A "Dummy's Guide" to
   working with MAVLink is
   `here <http://diydrones.com/group/arducopterusergroup/forum/topics/mavlink-tutorial-for-absolute-dummies-part-i?xg_source=activity>`__.
-  `UAVCAN* <http://uavcan.org>`__ -
   Lightweight protocol designed for reliable communication in aerospace and robotic 
   applications via CAN bus. Ardupilot is using the `Libuavcan <http://uavcan.org/Implementations/Libuavcan/>`__,
   which is a portable, cross-platform library written in C++ with minimal dependency on the C++ standard library.

How the team works
==================

-  Our annual developers conference is held in Feb/March in Canberra Australia (`2018 annoucement <https://discuss.ardupilot.org/t/developer-meetup-in-canberra-february-2018>`__).
-  The source code for ArduPilot/APM is managed using git on https://github.com/ArduPilot/ardupilot
-  Pre-compiled firmware for supported autopilot boards is available from http://firmware.ardupilot.org
-  User support is available on the `forums <http://discuss.ardupilot.org/>`__.
-  The ArduPilot `automatic test system <http://autotest.ardupilot.org/>`__ shows the test status of each commit. It's described `here <http://diydrones.com/profiles/blog/show?id=705844%3ABlogPost%3A703309>`__.
-  Bug tracking and open issues are tracked using the `github issues system <https://github.com/ArduPilot/ardupilot/issues>`__
-  Vehicle onboard parameter documentation for :ref:`copter <copter:parameters>`, :ref:`plane <plane:parameters>` and :ref:`rover <rover:parameters>` is auto-generated from the source code

Development languages and tools
===============================

The main flight code for ArduPilot is written in C++. Support tools are
written in a variety of languages, most commonly in python.

License
=======

ArduPilot (including Copter, Plane, Rover, Antenna Tracker and MissionPlanner) is
released as free software under the `GNU General Public License <https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt>`__
version 3 or later.  See :ref:`License overview wiki page here. <license-gplv3>`

Didn't find what you are looking for?
=====================================

If you think of something that should be added to this site, please
`open an issue for the wiki <https://github.com/ArduPilot/ardupilot_wiki/issues>`__.


Full Table of Contents
======================

.. toctree::
   :titlesonly:
   
    License (GPLv3) <docs/license-gplv3>
    Downloading the code / Using Git <docs/where-to-get-the-code>
    Building the code <docs/building-the-code>
    Editors & IDEs <docs/code-editing-tools-and-ides>
    Learning the code <docs/learning-the-ardupilot-codebase>
    Simulation & Testing <docs/simulation-2>
    Debugging <docs/debugging>
    Contributing Code <docs/contributing>
    MAVLink Commands <docs/mavlink-commands>
    CAN and UAVCAN <docs/can-bus>
    Companion Computers <docs/companion-computers>
    ROS <docs/ros>
    Porting to a new Flight Controller <docs/porting>
    Pixhawk Advanced Hardware Info <docs/pixhawk-advanced-hardware-info>
    MAVProxy Developer GCS <docs/mavproxy-developer-gcs>
    RTF Vehicle Developer Information <docs/ready-to-fly-rtf-vehicle-developer-information>
    How The Team Works <docs/how-the-team-works>
    Events <docs/events>
    Schools <docs/schools>
    GSoC 2018 project ideas <docs/gsoc-ideas-list>
    Wiki Editing Guide <docs/common-wiki_editing_guide>
    Appendix <docs/common-appendix>
    Full Table of Contents <docs/common-table-of-contents>
