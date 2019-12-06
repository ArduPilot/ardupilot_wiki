.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2019
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2019 <https://summerofcode.withgoogle.com/>`__. These are only suggestions, and if you have your own ideas then please discuss them on the `ArduPilot GSOC gitter channel <https://gitter.im/ArduPilot/GSoC>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__.  We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2019.

- `ROS integration with Copter and Rover <https://ardupilot.org/dev/docs/ros.html>`__ for non-GPS navigation and `object avoidance <https://ardupilot.org/dev/docs/code-overview-object-avoidance.html>`__
- `Optical flow <https://ardupilot.org/copter/docs/common-optical-flow-sensors-landingpage.html>`__ position hold performance improvements for multicopters
- `Precision Landing <https://ardupilot.org/copter/docs/precision-landing-with-irlock.html>`__ using `OpenMV camera <https://openmv.io/>`__ (or similar) for multicopters
- Automatic docking for cars and boats using `OpenMV camera <https://openmv.io/>`__ (or similar)
- Lane following or visual follow-me for Copter, Rover or Boat
- `Single Copter or Coax Copter <https://ardupilot.org/copter/docs/singlecopter-and-coaxcopter.html>`__ flight control improvements
- Helicopter autorotation support
- 3D aerobatic support for fixed wing aircraft
- Improve :ref:`Morse simulator <sitl-with-morse>` integration including setup to move camera with vehicles
- create new vehicle models for the Morse simulator, including boats, planes and copters
- `MathWorks SimuLink <https://www.mathworks.com/products/simulink.html>`__ interface to ArduPilot SITL
- `AirSim drone simulator <https://github.com/Microsoft/AirSim/>`__ support for ArduPilot SITL
- Build system improvements, specifically fixing dependency handling and speeding up the waf build
- Improvements to the `MAVProxy GCS <https://github.com/ArduPilot/MAVProxy>`__. Adding a parameter editor module, improving waypoint editor. Requires strong python skills.

See lower down on this page for more details for some of the projects listed above

Timeline
========

The timeline for `GSoC 2019 is here <https://summerofcode.withgoogle.com/how-it-works/#timeline>`__

How to improve your chances of being accepted
=============================================

When making the difficult decision about which students to accept, we look for:

- Clear and detailed application explaining how you think the project could be done
- Relevant prior experience
- Experience contributing to ArduPilot or other open source projects
- Understanding of Git and/or GitHub

ROS integration with Copter and Rover for non-GPS navigation and object avoidance
---------------------------------------------------------------------------------

ROS can already be used with ArduPilot (`see setup instructions here <https://ardupilot.org/dev/docs/ros.html>`__) but there is room for improvement including:

- Improve non-GPS position estimation when using :ref:`Google Cartographer <ros-cartographer-slam>` especially when the vehicle turns quickly
- Allow high level position targets received by ArduPilot (from the ground station via MAVLink) to be passed to ROS which should then return low-level movement commands to ArduPilot
- Improve passing of obstacle distance information (i.e. lidar data) from ROS to ArduPilot for low-level `simple object avoidance <https://ardupilot.org/dev/docs/code-overview-object-avoidance.html>`__
- Improve clock syncronisation between the flight controller and companion computer
- Improve documentation and prepare an `APSync <https://ardupilot.org/dev/docs/apsync-intro.html>`__ images to ease user setup

Optical flow position hold performance improvements for multicopters
--------------------------------------------------------------------

ArduPilot supports `Optical flow sensors <https://ardupilot.org/copter/docs/common-optical-flow-sensors-landingpage.html>`__ including the low-cost `Cheerson CX-OF <https://ardupilot.org/copter/docs/common-cheerson-cxof.html>`__ sensor but this project aims to improve a multicopter's ability to hold position using this sensor.  This could involve:

- Creating an easy to use sensor calibration routine so that the sensor's pixel movements are accurately converted to rotation rates
- Verify the EKF's integration of sensor data is correct (i.e. check rotation rate data, sensor's defined position, sensor's update rate and lag are all properly used in the final position and velocity estimates)
- Check the `FlowHold mode <https://ardupilot.org/copter/docs/flowhold-mode.html>`__ code for issues especially around altitude estimation

Precision Landing using OpenMV camera (or similar) for multicopters
-------------------------------------------------------------------

ArduPilot supports `Precision Landing <https://ardupilot.org/copter/docs/precision-landing-with-irlock.html>`__ using the IRLock sensor or companion computer.  This project involves adding support for a low-cost `OpenMV camera <https://openmv.io/>`__ to recognise AprilTags and then provide the target to ArduPilot so that it can use existing controls to landing on the target.  This project could also involve validating and improving the small EKF used to estimate the landing target's relative position and velocity.

Lane following or visual follow-me for Copter, Rover or Boat
------------------------------------------------------------

This project involves using machine vision and/or machine learning to add lane following or visual follow-me to ArduPilot's Copter or Rover firmware

- Either a low-cost `OpenMV camera <https://openmv.io/>`__ or a high powered `companion computer <https://ardupilot.org/dev/docs/companion-computers.html>`__ like the NVidia TX2 could be used
- Recognise the road or user using machine vision or learning (User could have an AprilTag on their clothing)
- Send velocity commands (probably using the `SET_GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__ or `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`__ message) to move the vehicle in the correct direction
- Add solution to `APSync <https://ardupilot.org/dev/docs/apsync-intro.html>`__
- Document the implementation

Expenses for purchasing the companion computer and camera will be covered by ArduPilot

Single Copter and/or Coax Copter flight control improvements
--------------------------------------------------------------------

`Single Copter and Coax Copters <https://ardupilot.org/copter/docs/singlecopter-and-coaxcopter.html>`__ are vehicles with one or two motors on the top, along with 4 servo controlled fins below to direct the air.  ArduPilot already supports these vehicles and there have been some successful flights but their attitude controllers need more love and attention to bring them up to the level of performance of our other frame types.

This project would involve first running the vehicles in a simulator (probably `RealFlight8 <https://ardupilot.org/dev/docs/sitl-with-realflight.html#sitl-with-realflight>`__) and then testing on a real vehicle.

Developers looking to take on this project should have some understanding of control theory (PID controllers) and be prepared to do detailed analysis of dataflash logs of the simulated and real-flights to ensure our control methods match the physics of these vehicles.

Expenses for purchasing the simulator and vehicle will be covered by ArduPilot.

Helicopter auto-rotation support
--------------------------------

When the engine fails on a helicopter a good pilot can land the helicopter safely using auto-rotation. We would like ArduPilot to support doing this automatically. We already have a very nice simulation system for helicopters using the RealFlight FlightAxis backend, which gives the ideal test environment for developing this feature. The project would involve using the rotor RPM and motor RPM sensors in the simulator to produce a reliable auto-rotation from a variety of heights and flight speeds. If simulator testing goes well then it could be tested on a number of real helicopters.

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

Support for AirSim simulator
----------------------------

Microsoft recently released support for their AirSim drone simulator based on the Unreal 3D gaming engine. It looks like a very nice simulation framework, and we would like to add support for using it for ArduPilot development. The project would involve adding interface code between AirSim and ArduPilot, working with the AirSim developers if needed to enhance their APIs (such as adding lock-step scheduling). Please note that this project will require you to have a fast enough PC to run AirSim (good graphics card and lots of memory).

MathWorks SimuLink
------------------

`MathWorks SimuLink <https://www.mathworks.com/products/simulink.html>`__ is a popular model based control algorithm design program.  The purpose of this project would be to allow SimuLink to create attitude control algorithm code (in C++) that can then be compiled into ArduPilot and flown in the simulator or on a real vehicle.

Projects Completed in past years
--------------------------------

In 2018, students successfully completed these projects:

- `BalanceBot <https://ardupilot.org/rover/docs/balance_bot-home.html>`__
- RedTail integration with ArduPilot
- Live video improvements for APSync

In 2017, 3 student successfully completed these projects:

- Smart Return-To-Launch which involves storing vehicle's current location and maintaining the shortest possible safe path back home
- Rework ArduRover architecture to allow more configurations and rover type (`see details here <https://github.com/khancyr/GSOC-2017>`__)
- Add "sensor head" operation of ArduPilot, split between two CPUs

 You can find their proposals and works on the `Google GSoC 2017 archive page <https://summerofcode.withgoogle.com/archive/2017/organizations/5801067908431872>`__
