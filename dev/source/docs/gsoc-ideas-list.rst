.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2023
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2023 <https://summerofcode.withgoogle.com/>`__. These are only suggestions so if you have your own ideas then please discuss them on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__.  We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2023.

- Rover AutoTune
- Boat object avoidance with Luxonis AI camera
- Copter/Rover camera gimbal integration improvements
- Update ROS integration for Non-GPS navigation and off-board path-planning
- Create more ignition vehicle models, and improve physics of existing models in SITL (software-in-the-loop simulator)
- Improve custom firmware server including adding branch support and improve dependency handling
- Improve :ref:`Gazebo simulator <sitl-with-gazebo>` integration including JSON protocol, Gazebo9, and new sensors set
- Build system improvements, specifically fixing dependency handling and speeding up the waf build
- Improvements to the `MAVProxy GCS <https://github.com/ArduPilot/MAVProxy>`__. Better multivehicle support, performance improvement. Requires strong python skills.
- Swift Package for MAVLink communications

See lower down on this page for more details for some of the projects listed above

Timeline
========

The timeline for `GSoC 2023 is here <https://developers.google.com/open-source/gsoc/timeline>`__

How to improve your chances of being accepted
=============================================

When making the difficult decision about which students to accept, we look for:

- Clear and detailed application explaining how you think the project could be done
- Relevant prior experience
- Experience contributing to ArduPilot or other open source projects
- Understanding of Git and/or GitHub

Rover AutoTune
--------------

- Skills required: C++
- Mentor: Randy Mackay
- Expected Size: 175h or 350h
- Level of Difficulty: Hard
- Expected Outcome: Autotune mode added that automatically calculates the frame limits, speed control gains and steering control gains for `Ackermann <https://ardupilot.org/rover/docs/rover-motor-and-servo-connections.html#separate-steering-and-throttle>`__ and `skid steering vehicles <https://ardupilot.org/rover/docs/rover-motor-and-servo-connections.html#skid-steering>`__.

This project involves `adding a new AutoTune mode to the Rover firmware <https://ardupilot.org/dev/docs/rover-adding-a-new-drive-mode.html>`__ to calculate the frame limits, speed control and steering control gains.  This is essentially an automated version of the manual tuning process documented in `this section of the Rover wiki <https://ardupilot.org/rover/docs/rover-first-drive.html>`__.

Similar to `Copter's autotune mode <https://ardupilot.org/copter/docs/autotune.html>`__ this new mode should include a state machine that provides various throttle and steering outputs and then monitors the response by checking the AHRS/EKF outputs.

The list of parameters that should be tuned includes:

- :ref:`ATC_ACCEL_MAX <rover:ATC_ACCEL_MAX>` and :ref:`ATC_DECEL_MAX <rover:ATC_DECEL_MAX>`
- :ref:`ATC_STR_RAT_FF <rover:ATC_STR_RAT_FF>`
- :ref:`ATC_STR_RAT_MAX <rover:ATC_STR_RAT_MAX>`
- :ref:`ATC_TURN_MAX_G <rover:ATC_TURN_MAX_G>`
- :ref:`CRUISE_SPEED <rover:CRUISE_SPEED>` and :ref:`CRUISE_THROTTLE <rover:CRUISE_THROTTLE>` (or :ref:`ATC_SPEED_FF <rover:ATC_SPEED_FF>`)

See `Issue #8851 <https://github.com/ArduPilot/ardupilot/issues/8851>`__

Some of the development can be completed using the :ref:`SITL simulator <sitl-simulator-software-in-the-loop>` but funding will also be provided for the RC car frame and autopilot

Boat object avoidance with Luxonis AI camera
--------------------------------------------

- Skills required: C++, mavlink, AI
- Mentor: Randy Mackay, Peter Barker
- Expected Size: 350h
- Level of Difficulty: Medium
- Expected Outcome: Autonomous boats is able to avoid other boats, rocks and floating debris using an Luxonis AI camera

This project involves training and integrating a Luxonis AI camera to recognise rocks, floating debris and other boats and then send the estimated position of these obstacles to ArduPilot's existing :ref:`object avoidance features <rover:common-object-avoidance-landing-page>` (Simple avoidance, Bendy Ruler and Dijkstra's/A-Star) so that the vehicle can stop and/or path plan around them.

Much of the development can be completed using one of the :ref:`ArduPilot supported simulators <simulation-2>` but funding will also be provided for the required hardware which will include a `Luxonis AI camera <https://shop.luxonis.com/products/oak-d-iot-75>`__, companion computer, autopilot and a car or boat frame.

Copter/Rover camera gimbal integration improvements
---------------------------------------------------

- Skills required: C++, mavlink
- Mentor: Randy Mackay, Peter Barker
- Expected Size: 175h or 350h
- Level of Difficulty: Medium
- Expected Outcome: Improved support of gimbals in pilot controlled and fully autonomous modes (Auto, Guided)

This project involves resolving numerous small issues with ArduPilot's camera gimbal support.  These include:

- Auxiliary switch to allow pilot to control whether the gimbal maintains an attitude relative to the vehicle's heading or stays pointed in the same direction even as the vehicle turns (aka "earth frame")
- Support for new mavlink ROI messages (see `issue #7658 <https://github.com/ArduPilot/ardupilot/issues/7658>`__)
- Identify and resolve any issues with pilot controlling gimbal using rate or angle control
- Support for pointing gimbal at :ref:`Circle center <copter:circle-mode>`
- Support for pointing gimbal at another vehicle while in :ref:`Follow mode <copter:follow-mode>`
- Support for pointing gimbal at specified waypoint
- Resolve any specific issues with the Gremsy PixyU gimbal (see `issue #14448 <https://github.com/ArduPilot/ardupilot/issues/14448>`__)

Funding will be provided for the required hardware which will likely include a camera gimbal, transmitter, autopilot and a multicopter or car frame.

Update ROS integration for Non-GPS navigation and off-board path-planning
-------------------------------------------------------------------------

- Skills required: ROS, C++, python
- Mentor: Randy Mackay, Jaime Machuca
- Expected Size: 175h or 350h
- Level of Difficulty: Medium

ArduPilot can be `integrated with ROS <https://ardupilot.org/dev/docs/ros.html>`__ both for `Non-GPS position estimation <https://ardupilot.org/dev/docs/ros-cartographer-slam.html>`__ and `object avoidance <https://ardupilot.org/dev/docs/ros-object-avoidance.html>`__.  This project aims to verify and update the instructions for these two features.

Once the above two items are complete, if time permits the next task would be to integrate the offboard object avoidance with ArduPilot Auto mode.  This involves ArduPilot maintaining the final target but then sending it at 1hz (or faster) to ROS's offboard path planning algorithm via mavros.  This will primarily require updating mavros.

- `Randy's video using ROS for path planning around obstacles <https://www.youtube.com/watch?v=u99qwQSl9Z4>`__
- `mavros PR to allow ROS to accept set-position-target-global-int messages <https://github.com/mavlink/mavros/pull/1184>`__ from ArduPilot to be fed into ROS's navigation algorithm

Funding will be provided for the required hardware which will likely include an autopilot, Nvidia or RPI4 companion computer, 360 lidar and multicopter or RC car frame

`Related issue #5608 <https://github.com/ArduPilot/ardupilot/issues/5608>`__

Ignition Modelling
------------------

The new Gazebo Ignition simulation system offers a rich simulation
environment where the vehicle can interact with world objects and
other vehicles. We would like to expand the number of vehicle models
that are available, and improve the physics fidelity of the existing
vehicles. You can see the vehicle models we have now here
`https://github.com/ArduPilot/SITL_Models/tree/master/Ignition
<https://github.com/ArduPilot/SITL_Models/tree/master/Ignition>`__

The successful applicatant will need strong C++ skills, as well as an
understanding of aerodynamics for the creation of vehicle physics
models. Experience with Gazebo or Ignition would be a significant help.


Custom Firmware Server
----------------------

The ArduPilot custom firmware server (see
`https://custom.ardupilot.org <https://custom.ardupilot.org>`__ ) was
developed during GSoC 2021, and has been extremely useful. We would
like to extend the functionalty to multiple branches and add automatic
dependency handling, as well as support for enabling Lua scripts and
setting default parameters.

The successful student will need strong python and web development skills.

Build System Improvements
-------------------------

The build system that ArduPilot uses is based on the python waf
system. It works well, but we would like some improvements to reduce
the CPU overhead and improve dependency handling.

The successful student will need strong python skills and
understanding of build system structures.

MathWorks Simulink
------------------

`MathWorks Simulink <https://www.mathworks.com/products/simulink.html>`__ is a popular model based control algorithm design program.  The purpose of this project would be to allow Simulink to create attitude control algorithm code (in C++) that can then be compiled into ArduPilot and flown in the simulator or on a real vehicle.

Swift Package for Mavlink
-------------------------

`Swift Packages <https://developer.apple.com/documentation/swift_packages>`__ are Apple's solution for creating reusable components that can be used in iOS and Mac applications. MAVLink currently has several attempts to create a communications package for iOS, but they are currently not compatible with ArduPilot. The goal for this project would be to either create our own universal MAVLink package or adopt one of the existing ones (`MAVSDK Swift <https://github.com/mavlink/MAVSDK-Swift>`__, `pymavlink Swift Generator <https://github.com/ArduPilot/pymavlink/blob/master/generator/swift/MAVLink.swift>`__)to work with ArduPilot and be easily deployable as a Swift package so that anyone who wants to use it to create their own iOS based app can integrate it.

ROS2 MAVROS support for ArduPilot
---------------------------------

Currently, there is no MAVROS equivalent for ROS2, with `OSRF <https://www.openrobotics.org>`__ quickly moving to make ROS2 the standard version of ROS, supporting it has become a growing interest in our community. An initial port of the basic features of MAVROS would be a big step towards integrating ArduPilot and ROS2.

Projects Completed in past years
--------------------------------

In 2019, students successfully completed these projects:

- AirSim Simulator Support for ArduPilot SITL
- Development of Autonomous Autorotations for Traditional Helicopters
- Further Development of Rover Sailboat Support
- Integration of ArduPilot and VIO tracking camera for GPS-less localization and navigation
- MAVProxy GUI and module development

In 2018, students successfully completed these projects:

- `BalanceBot <https://ardupilot.org/rover/docs/balance_bot-home.html>`__
- RedTail integration with ArduPilot
- Live video improvements for APSync

In 2017, 3 students successfully completed these projects:

- Smart Return-To-Launch which involves storing the vehicle's current location and maintaining the shortest possible safe path back home
- Rework ArduRover architecture to allow more configurations and rover type (`see details here <https://github.com/khancyr/GSOC-2017>`__)
- Add "sensor head" operation of ArduPilot, split between two CPUs

 You can find their proposals and works on the `Google GSoC 2017 archive page <https://summerofcode.withgoogle.com/archive/2017/organizations/5801067908431872>`__
