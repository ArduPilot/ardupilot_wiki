.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2020
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2020 <https://summerofcode.withgoogle.com/>`__. These are only suggestions, and if you have your own ideas then please discuss them on the `ArduPilot GSOC gitter channel <https://gitter.im/ArduPilot/GSoC>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__.  We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2020.

- :ref:`Non-GPS navigation improvements using Intel RealSense cameras <common-vio-tracking-camera>`
- :ref:`Object avoidance <common-object-avoidance-landing-page>` improvements for Multicopters and/or Rovers
- `Optical flow <https://ardupilot.org/copter/docs/common-optical-flow-sensors-landingpage.html>`__ position hold performance improvements for multicopters
- `Precision Landing <https://ardupilot.org/copter/docs/precision-landing-with-irlock.html>`__ accuracy improvements using `JeVois camera <http://www.jevois.org/>`__ (or similar) for multicopters
- Automatic docking for cars and boats using `JeVois camera <http://www.jevois.org/>`__ (or similar)
- Lane following or visual follow-me for Copter, Rover or Boat
- Walking robot support
- 3D aerobatic support for fixed wing aircraft
- Improve :ref:`Morse simulator <sitl-with-morse>` integration including setup to move camera with vehicles
- Create new vehicle models for the Morse simulator, including boats, planes and copters
- `MathWorks SimuLink <https://www.mathworks.com/products/simulink.html>`__ interface to ArduPilot SITL
- `AirSim drone simulator <https://github.com/Microsoft/AirSim/>`__ support for ArduPilot SITL
- Build system improvements, specifically fixing dependency handling and speeding up the waf build
- Improvements to the `MAVProxy GCS <https://github.com/ArduPilot/MAVProxy>`__. Adding a parameter editor module, improving waypoint editor. Requires strong python skills.
- Improve helicopter throttle handling for internal combustion engines for autonomous operations.

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
- Provide obstacle data from an Intel Realsense camera to ardupilot using the mavlink `OBSTACLE_DISTANCE <https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE>`__ message
- Prepare `APSync <https://ardupilot.org/dev/docs/apsync-intro.html>`__ images to ease user setup

Optical flow position hold performance improvements for multicopters
--------------------------------------------------------------------

ArduPilot supports `Optical flow sensors <https://ardupilot.org/copter/docs/common-optical-flow-sensors-landingpage.html>`__ but the setup, accuracy and reliability could be improved:

- Multicopters should not trigger an EKF failsafe on loss of GPS if a working optical flow sensor is available (`issue <https://github.com/ArduPilot/ardupilot/issues/9919>`__, `possible fix <https://github.com/ArduPilot/ardupilot/pull/12482>`__)
- Create an easy to use sensor calibration routine so that the sensor's pixel movements are accurately converted to rotation rates
- Verify the EKF's integration of sensor data is correct (i.e. check rotation rate data, sensor's defined position, sensor's update rate and lag are all properly used in the final position and velocity estimates)

Precision Landing accuracy improvements for Multicopters
--------------------------------------------------------

ArduPilot supports `Precision Landing <https://ardupilot.org/copter/docs/precision-landing-with-irlock.html>`__ using the IRLock sensor or companion computer.  This project could involve:

- Add support for the `JeVois camera <http://www.jevois.org/>`__ (or similar) to recognise AprilTags and then provide the target to the autopilot via mavlink
- Improve the reliability and accuracy of the small EKF used to estimate the landing target's relative position and velocity.  In particular adding protection against large lag in the readings leading to unreasonable results.

Lane following or visual follow-me for Copter, Rover or Boat
------------------------------------------------------------

This project involves using machine vision and/or machine learning to add lane following or visual follow-me to ArduPilot's Copter or Rover firmware

- Either a low-cost `JeVois camera <http://www.jevois.org/>`__ or a high powered `companion computer <https://ardupilot.org/dev/docs/companion-computers.html>`__ like the NVidia TX2 could be used
- Recognise the road or user using machine vision or learning (User could have an AprilTag on their clothing)
- Send velocity commands (probably using the `SET_GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__ or `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`__ message) to move the vehicle in the correct direction
- Add solution to `APSync <https://ardupilot.org/dev/docs/apsync-intro.html>`__
- Document the implementation

Expenses for purchasing the companion computer and camera will be covered by ArduPilot

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

MathWorks SimuLink
------------------

`MathWorks SimuLink <https://www.mathworks.com/products/simulink.html>`__ is a popular model based control algorithm design program.  The purpose of this project would be to allow SimuLink to create attitude control algorithm code (in C++) that can then be compiled into ArduPilot and flown in the simulator or on a real vehicle.

Improve helicopter throttle handling for internal combustion engines for autonomous operations
----------------------------------------------------------------------------------------------

The helicopter code manages the throttle for all propulsion types through the rotor speed controller.  This controller provides very basic throttle control for internal combustion engines through rotor run-up and shutdown sequence.  It ramps the throttle from the idle setting to the bottom of the throttle curve.  It does not provide any warm up or cool down period for autonomous operations.  The goal of this project would be to incorporate an automated rotor startup sequence after engine start and rotor shutdown, engine cooldown and engine cut to support fully autonomous operations.  Similar work has been conducted in this area with an off-shoot of ardupilot but it relies on pilot interaction although it incorporates a torque limited rotor spool up which would be a great to incorporate in arducopter RSC.  Details of the rotor speed controller can be found in the `traditional helicopter RSC setup wiki <https://ardupilot.org/copter/docs/traditional-helicopter-rsc-setup.html>`__.  A heli with an internal combustion engine is not necessarily required to complete this project but would be helpful.  The RealFlight simulation linked with ardupilot SITL is required to do initial testing and proof of concept. This setup is described in the  `Using SITL with Realflight wiki <https://ardupilot.org/dev/docs/sitl-with-realflight.html>`__.


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
