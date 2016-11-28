.. _home:

=========================================
Welcome to the ArduPilot Development Site
=========================================

.. tip::

    Keep up with the latest ArduPilot related blogs on `ArduPilot.org! <http://ardupilot.org/>`__


:ref:`ArduPilot/APM <ardupilot:home>` is a open source autopilot
system supporting multi-copters, traditional helicopters, fixed wing
aircraft and rovers. The source code is developed by a `large community of enthusiasts <http://ardupilot.org>`__. 
New developers are always welcome! The best way to start is by joining the 
`Developer Team Forum <http://discuss.ardupilot.org/c/development-team>`__, which
is open to all and chock-full of daily development goodness. Lurk for a
while to get a feel for it, then participate!

Why the name?
=============

The 'Ardu' part of the ArduPilot name comes from Arduino. The original
APM1 autopilot board was based around the
`Arduino <http://www.arduino.cc/>`__ development environment. We've
since outgrown the Arduino environment and no longer use the Arduino
runtime libraries, although we do still support building the ArduPilot
for the AVR based APM1 and APM2 boards using a slightly modified version
of the Arduino integrated development environment. A timeline history of
ArduPilot can be found :ref:`here <history-of-ardupilot>`.


Supported boards
================

:ref:`Supported AutoPilot Controller Boards <supported-autopilot-controller-boards>` provides an overview
and key links for all the supported controller boards, including
`Pixhawk <https://store.3drobotics.com/products/3dr-pixhawk>`__, `Arsov AUAV-X2 <http://www.auav.co/product-p/auavx2.htm>`__,
`Erle-Brain <http://erlerobotics.com/docs/Intro.html>`__,
`NAVIO+ <http://www.emlid.com/>`__ etc.

The ArduPilot/APM source code is written on top of
the `AP-HAL <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL>`__
hardware abstraction layer, making it possible to port the code to a
wide range of autopilot boards. See this `blog post <http://diydrones.com/profiles/blogs/lots-of-changes-to-apm-development>`__
for more information on the move to AP-HAL.

Project List
============

The ArduPilot system is made up of (or relies upon) several
different projects which are listed below.  Those marked with an asterix
(\*) are peer projects that have their own owners outside the core
ArduPilot dev team.

-  DroneKit (`site <http://dronekit.io/>`__) - APM SDK for apps running
   on vehicles, mobile devices and/or in the cloud.
-  Plane (`wiki <http://plane.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/ardupilot>`__) - autopilot for
   planes
-  Copter (`wiki <http://copter.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/ardupilot>`__) - autopilot for
   multicopters and traditional helicopters
-  Rover (`wiki <http://rover.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/ardupilot>`__) - autopilot for
   ground vehicles
-  Mission Planner (`wiki <http://planner.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/MissionPlanner>`__) - the most
   commonly used ground station written in C# for windows but also runs
   on Linux and MacOS via mono
-  APM Planner 2.0 (`wiki <http://planner2.ardupilot.com/>`__,
   `code <https://github.com/ArduPilot/apm_planner>`__) is a ground
   station specifically for APM written in C++ using the Qt libraries
-  MAVProxy
   (`wiki <http://www.qgroundcontrol.org/mavlink/mavproxy_startpage>`__)
   - command line oriented and scriptable ground station (mostly used by
   developers)
-  MinimOSD (`wiki <http://code.google.com/p/arducam-osd/wiki/minimosd>`__,
   `code <http://code.google.com/p/arducam-osd/source/browse/#svn%2Ftrunk%2FArduCAM_OSD>`__)
   - on-screen display of flight data
-  AndroPilot (`user
   guide <https://github.com/geeksville/arduleader/wiki/Andropilot%20Users%20Guide>`__,
   `code <https://github.com/geeksville/arduleader/tree/master/andropilot>`__,
   `google play <https://play.google.com/store/apps/details?id=com.geeksville.andropilot&hl=en>`__)
   - android ground station
-  DroneAPI
   (`tutorial <http://dev.ardupilot.com/wiki/droneapi-tutorial/>`__,
   `droneshare <http://www.droneshare.com>`__) - A developer API for
   drone coprocessors and web applications.
-  DroidPlanner2
   (`wiki <https://github.com/DroidPlanner/droidplanner/wiki>`__,
   `code <https://github.com/DroidPlanner/droidplanner>`__, `google play <https://play.google.com/store/apps/details?id=org.droidplanner>`__)
   - android ground station
-  `QGroundControl <http://www.qgroundcontrol.org/>`__ is an alternative
   ground station written in C++ using the Qt libraries
-  PX4 (`wiki <http://pixhawk.org/firmware/start>`__) - designers of the
   PX4FMU and owners of the underlying libraries upon which
   Plane/Copter/Rover use when running on the PX4FMU
-  MAVLink (`wiki <http://www.qgroundcontrol.org/mavlink/start>`__) -
   the protocol for communication between the ground station, flight
   controller and some periphers including the OSD. A "Dummy's Guide" to
   working with MAVLink is
   `here <http://diydrones.com/group/arducopterusergroup/forum/topics/mavlink-tutorial-for-absolute-dummies-part-i?xg_source=activity>`__.

Getting Started with ArduPilot Development
==========================================

The main entry points for developing flight controller/antenna tracker
and companion computer code are listed in the sidebar.

For topics related to Ground Control Station development see:

-  :ref:`Building Mission Planner (C#, Windows) <buildin-mission-planner>`
-  `Building APM Planner 2.0 (Qt, C++, Linux, Mac OSX, Windows) <https://github.com/ArduPilot/apm_planner/blob/master/README.md>`__

RTF vehicles
============

-  :ref:`3DR Solo <solo>`

How the team works
==================

-  :ref:`Bringing new members onto the team <guidelines-for-contributors-to-the-apm-codebase>`
-  The main developer discussion mailing list is
   `drones-discuss <https://groups.google.com/forum/#!forum/drones-discuss>`__,
   and is open to anyone to join
-  The development team is also using `gitter <https://gitter.im/>`__
   for APM development discussions -
   https://gitter.im/ArduPilot/ardupilot
-  We have a :ref:`mumble server <ardupilot-mumble-server>`
   for real-time voice discussions
-  Our annual developers conference is
   `DroneCon <http://www.dronecon.org/>`__. See previous years speeches
   and content `here <http://www.dronecon.org/>`__.
-  The source code for ArduPilot/APM is managed using git on
   https://github.com/ArduPilot/ardupilot
-  Pre-compiled firmware for supported autopilot boards is available
   from http://firmware.ardupilot.org
-  User support is available on the `APM forums <http://ardupilot.com/forum/viewforum.php?f=25>`__.
-  The ArduPilot/APM \ `automatic test system <http://autotest.ardupilot.org/>`__ shows the test status of
   each commit. It's described
   `here <http://diydrones.com/profiles/blog/show?id=705844%3ABlogPost%3A703309>`__.
-  Bug tracking and open issues are tracked using the `github issues system <https://github.com/ArduPilot/ardupilot/issues>`__
-  Vehicle onboard parameter documentation for
   :ref:`copter <copter:parameters>`, :ref:`plane <plane:parameters>` and
   :ref:`rover <rover:parameters>` is auto-generated from the source code
-  :ref:`Release Procedures for Copter <release-procedures>`
-  :ref:`Current and Past Dev Team members <planner:common-team>`

Development languages and tools
===============================

The main flight code for ArduPilot is written in C++. Support tools are
written in a variety of languages, most commonly in python. Currently
the main vehicle code is written as '.pde' files, which come from the
Arduino build system. The pde files are preprocessed into a .cpp file as
part of the build. The include statements in the pde files also provide
implied build rules for what libraries to include and link to.

License
=======

ArduPilot (including Copter, Plane, Rover and MissionPlanner) is
released as free software under the `GNU General Public License <https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt>`__
version 3 or later.  See :ref:`License overview wiki page here. <license-gplv3>`

Didn't find what you are looking for?
=====================================

If you think of something that should be added to this site, please
`open an issue <https://github.com/ArduPilot/ardupilot/issues>`__ or
post a comment on the
`drones-discuss <https://groups.google.com/forum/#!forum/drones-discuss>`__
mailing list.



Full Table of Contents
======================

.. toctree::
   :titlesonly:
   
    Working with the ArduPilot Project Code <docs/where-to-get-the-code>
    License (GPLv3) <docs/license-gplv3>
    Supported Controller Boards <docs/supported-autopilot-controller-boards>
    Learning the ArduPilot Codebase <docs/learning-the-ardupilot-codebase>
    Code Overview (Copter) <docs/apmcopter-code-overview>
    Building the code <docs/building-the-code>
    Code Editors & IDEs <docs/code-editing-tools-and-ides>
    Simulation & Testing <docs/simulation-2>
    Debugging <docs/debugging>
    Contributing <docs/contributing>
    MAVLink Commands <docs/mavlink-commands>
    Companion Computers <docs/companion-computers>
    EKF <docs/ekf>
    ROS <docs/ros>
    Pixhawk Advanced Hardware Info <docs/pixhawk-advanced-hardware-info>
    MAVProxy Developer GCS <docs/mavproxy-developer-gcs>
    RTF Vehicle Developer Information <docs/ready-to-fly-rtf-vehicle-developer-information>
    Appendix <docs/common-appendix>
    Full Table of Contents <docs/common-table-of-contents>
