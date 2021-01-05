.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2020
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2020 <https://summerofcode.withgoogle.com/>`__. These are only suggestions, and if you have your own ideas then please discuss them on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__.  We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2020.

- :ref:`Non-GPS navigation improvements using Intel RealSense cameras <common-vio-tracking-camera>`
- :ref:`Object avoidance <common-object-avoidance-landing-page>` improvements for Multicopters and/or Rovers
- Lane following or automatic docking for cars and boats using `JeVois camera <http://www.jevois.org/>`__ (or similar)
- Rover Autotune
- Walking robot support
- 3D aerobatic support for fixed wing aircraft
- Improve :ref:`Morse simulator <sitl-with-morse>` integration including setup to move camera with vehicles
- Create new vehicle models for the Morse simulator, including boats, planes and copters
- Improve :ref:`Gazebo simulator <using-gazebo-simulator-with-sitl>` integration including json protocol, Gazebo9, and new sensors set
- `MathWorks SimuLink <https://www.mathworks.com/products/simulink.html>`__ interface to ArduPilot SITL
- Build system improvements, specifically fixing dependency handling and speeding up the waf build
- Improvements to the `MAVProxy GCS <https://github.com/ArduPilot/MAVProxy>`__. Better multivehicle support, performance improvement. Requires strong python skills.
- Improve helicopter throttle handling for internal combustion engines for autonomous operations.
- Swift Package for MAVLink communications.
- Unified performance counter on ArduPilot

See lower down on this page for more details for some of the projects listed above

Timeline
========

The timeline for `GSoC 2020 is here <https://summerofcode.withgoogle.com/how-it-works/#timeline>`__

How to improve your chances of being accepted
=============================================

When making the difficult decision about which students to accept, we look for:

- Clear and detailed application explaining how you think the project could be done
- Relevant prior experience
- Experience contributing to ArduPilot or other open source projects
- Understanding of Git and/or GitHub

Non-GPS navigation improvements using Intel RealSense cameras
-------------------------------------------------------------

Intel Realsense cameras can already be used with ArduPilot but there is still room for improvement including:

- Allow vehicles to move seamlessly between GPS environments and non-GPS environments.  This will likely require enhancements to ArduPilot's EKF.
- Provide obstacle data from an Intel Realsense camera to ardupilot using the MAVLink `OBSTACLE_DISTANCE <https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE>`__ message
- Prepare `APSync <https://ardupilot.org/dev/docs/apsync-intro.html>`__ images to ease user setup

Object Avoidance improvements for Multicopters and/or Rovers
------------------------------------------------------------

ArduPilot supports three methods for object avoidance, `Bendy Ruler <https://ardupilot.org/copter/docs/common-oa-bendyruler.html>`__, `Dijkstra's <https://ardupilot.org/copter/docs/common-oa-dijkstras.html>`__ and `Simple avoidance <https://ardupilot.org/copter/docs/common-simple-object-avoidance.html>`__ but there is room for improvement in each of them:

- BendyRuler should work in 3D (`issue <https://github.com/ArduPilot/ardupilot/issues/13215>`__)
- BendyRuler can be hesitant about which direction to choose (`issue <https://github.com/ArduPilot/ardupilot/issues/11961>`__)
- Rover's using BendyRuler may impact the fence after clearing obstacles (`issue <https://github.com/ArduPilot/ardupilot/issues/11565>`__)
- Dijkstra's should work with Spline waypoints (`issue <https://github.com/ArduPilot/ardupilot/issues/12691>`__)
- Simple avoidance should backaway from objects (`issue <https://github.com/ArduPilot/ardupilot/issues/7706>`__)

Lane following or automatic docking for cars and boats
------------------------------------------------------

This project involves using machine vision to add lane following or automatic docking to to ArduPilot's Rover firmware

- Either a low-cost `JeVois camera <http://www.jevois.org/>`__ or a high powered `companion computer <https://ardupilot.org/dev/docs/companion-computers.html>`__ could be used
- Recognise the road or docking target using machine vision or learning (for docking an AprilTag could be used)
- Either create a new control mode to control the vehicle or send velocity commands (probably using the `SET_GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__ or `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`__ message) to move the vehicle in the correct direction
- If a companion computer is used, add the solution to `APSync <https://ardupilot.org/dev/docs/apsync-intro.html>`__
- Document the implementation

Rover Autotune
--------------

This project would involve adding an autotune feature for rover and boat like for copter.
The autotune should be able to learn and set most of the rover parameters for autonomous behavior.
This will need a good understanding of control theory.

Walking robot support
---------------------

This project would involve adding basic support for four legged walking robots and could involve:

- Identifying a reasonably priced four legged robot frame
- Control system improvements to allow the frame to stand and walk
- Documentation of the setup

Expenses for purchasing the frame and autopilot will be covered by ArduPilot.

Improve fixed-wing 3D aerobatics support in ArduPilot
-----------------------------------------------------

With the addition of prop-hang in ArduPilot (`see here <https://discuss.ardupilot.org/t/ardupilot-flying-3d-aircraft-including-hovering/14837>`__) we now have the beginnings of a nice 3D aerobatics for fixed wing.
This project involves taking that to the next level to add support for "trick" mode. In trick mode the user will have access to a a variety of common 3D maneuvers, including knife-edge, loops, harrier and rolling loops. Implementing this will involve some careful use of quaternion controllers, but a good UI design so the stick inputs to control these tricks are easy to learn.
Testing can be done in the FlightAxis simulator (as in the above video), allowing for development without risking real aircraft.

Improve Morse simulator integration including supporting boats / ROVs
---------------------------------------------------------------------

Improve ArduPilot's integration with :ref:`Morse simulator <sitl-with-morse>` software including

- Adding support for boats and ROVs with simulated waves to test ArduPilot controls
- Default camera view to follow the vehicle

Unified performance counter on ArduPilot
----------------------------------------

This project would involve adding unified support for performance accross our HAL.
Currently, Linux board get the most performant performance counter, but we should be able to some on Chibios and SITL to allow better profiling of the code.

MathWorks SimuLink
------------------

`MathWorks SimuLink <https://www.mathworks.com/products/simulink.html>`__ is a popular model based control algorithm design program.  The purpose of this project would be to allow SimuLink to create attitude control algorithm code (in C++) that can then be compiled into ArduPilot and flown in the simulator or on a real vehicle.

Improve helicopter throttle handling for internal combustion engines for autonomous operations
----------------------------------------------------------------------------------------------

The helicopter code manages the throttle for all propulsion types through the rotor speed controller.  This controller provides very basic throttle control for internal combustion engines through rotor run-up and shutdown sequence.  It ramps the throttle from the idle setting to the bottom of the throttle curve.  It does not provide any warm up or cool down period for autonomous operations.  The goal of this project would be to incorporate an automated rotor startup sequence after engine start and rotor shutdown, engine cooldown and engine cut to support fully autonomous operations.  Similar work has been conducted in this area with an off-shoot of ardupilot but it relies on pilot interaction although it incorporates a torque limited rotor spool up which would be a great to incorporate in arducopter RSC.  Details of the rotor speed controller can be found in the `traditional helicopter RSC setup wiki <https://ardupilot.org/copter/docs/traditional-helicopter-rsc-setup.html>`__.  A heli with an internal combustion engine is not necessarily required to complete this project but would be helpful.  The RealFlight simulation linked with ardupilot SITL is required to do initial testing and proof of concept. This setup is described in the  `Using SITL with Realflight wiki <https://ardupilot.org/dev/docs/sitl-with-realflight.html>`__.

Swift Package for Mavlink
-------------------------

`Swift Packages <https://developer.apple.com/documentation/swift_packages>`__ are Apples solution for creating reusable components that can be used in iOS and Mac applications. MAVLink currently has several attempts to create a communications package for iOS, but they are currently not compatible with ArduPilot. The goal for this project would be to either create our own universal MAVLink package or adapt one of the existing ones (`MAVSDK Swift <https://github.com/mavlink/MAVSDK-Swift>`__, `pymavlink Swift Generator <https://github.com/ArduPilot/pymavlink/blob/master/generator/swift/MAVLink.swift>`__)to work with ArduPilot and be easily deployable as a Swift package so that any one who wants to use it to create their own iOS based app can integrate it.

Projects Completed in past years
--------------------------------

In 2019, students successfully completed these projects:

- AirSim Simulator Support for Ardupilot SITL
- Development of Autonomous Autorotations for Traditional Helicopters
- Further Development of Rover Sailboat Support
- Integration of ArduPilot and VIO tracking camera for GPS-less localization and navigation
- MAVProxy GUI and module development

In 2018, students successfully completed these projects:

- `BalanceBot <https://ardupilot.org/rover/docs/balance_bot-home.html>`__
- RedTail integration with ArduPilot
- Live video improvements for APSync

In 2017, 3 student successfully completed these projects:

- Smart Return-To-Launch which involves storing vehicle's current location and maintaining the shortest possible safe path back home
- Rework ArduRover architecture to allow more configurations and rover type (`see details here <https://github.com/khancyr/GSOC-2017>`__)
- Add "sensor head" operation of ArduPilot, split between two CPUs

 You can find their proposals and works on the `Google GSoC 2017 archive page <https://summerofcode.withgoogle.com/archive/2017/organizations/5801067908431872>`__
