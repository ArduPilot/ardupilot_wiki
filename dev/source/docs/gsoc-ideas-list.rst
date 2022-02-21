.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2022
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2022 <https://summerofcode.withgoogle.com/>`__. These are only suggestions so if you have your own ideas then please discuss them on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__.  We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2021.

- Rover AutoTune
- Rover/Boat automatic docking
- Rover/Boat object avoidance with Luxonis AI camera
- Copter/Rover camera gimbal integration improvements
- Update ROS integration for Non-GPS navigation and off-board path-planning
- Create more ignition vehicle models, and improve physics of existing models in SITL (software-in-the-loop simulator)
- Improve custom firmware server including adding branch support and improve dependency handling
- Improve :ref:`Gazebo simulator <using-gazebo-simulator-with-sitl>` integration including JSON protocol, Gazebo9, and new sensors set
- Build system improvements, specifically fixing dependency handling and speeding up the waf build
- Improvements to the `MAVProxy GCS <https://github.com/ArduPilot/MAVProxy>`__. Better multivehicle support, performance improvement. Requires strong python skills.
- Swift Package for MAVLink communications
- Unified performance counter on ArduPilot

See lower down on this page for more details for some of the projects listed above

Timeline
========

The timeline for `GSoC 2021 is here <https://summerofcode.withgoogle.com/how-it-works/#timeline>`__

How to improve your chances of being accepted
=============================================

When making the difficult decision about which students to accept, we look for:

- Clear and detailed application explaining how you think the project could be done
- Relevant prior experience
- Experience contributing to ArduPilot or other open source projects
- Understanding of Git and/or GitHub

Optical Flow Calibration Improvements
-------------------------------------

:ref:`Optical Flow <common-optical-flow-sensors-landingpage>` can provide accurate non-GPS position estimation if the user correctly calibrates the sensor but `this calibration procedure <https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html>`__ is difficult to get right.

This project involves adding an in-flight calibration procedure in which the user enables both GPS and optical flow.  The EKF should then be able to estimate the expected flow measurements, compare them with the actual flow measurements and then calculate the best scaling values.

See `Issue #16631 <https://github.com/ArduPilot/ardupilot/issues/16631>`__.

Object Avoidance support for the MYNT EYE depth camera
------------------------------------------------------

ArduPilot already supports :ref:`object avoidance using the Intel RealSense 435 and 455 depth cameras <common-realsense-depth-camera>`. We should extend support to the `MYNT EYE depth cameras <https://www.mynteye.com/pages/products>`__.

This project involves:

- Writing a python script (similar to `this script for the Intel T435 <https://github.com/thien94/vision_to_mavros/blob/master/scripts/d4xx_to_mavlink.py>`__) to pull the data from the depth camera and package them into OBSTACLE_DISTANCE and/or OBSTACLE_DISTANCE_3D mavlink messages which will then be consumed by ArduPilot's AP_Proximity library
- Creating an :ref:`APSync <apsync-intro>` image for at least one companion computer (RPI4?) that can run the above script

See `Issue #16632 <https://github.com/ArduPilot/ardupilot/issues/16632>`__.

Integrate with ROS for off-board path-planning
----------------------------------------------

ArduPilot can be :ref:`integrated with ROS <ros>` in several ways including for Non-GPS position estimation and object avoidance.  This project aims to allow ROS's path planning routines to be used by ArduPilot while still leaving the mission input in ArduPilot.

- `Randy's video using ROS for path planning around obstacles <https://www.youtube.com/watch?v=u99qwQSl9Z4>`__
- `mavros PR to allow ROS to accept set-position-target-global-int messages <https://github.com/mavlink/mavros/pull/1184>`__ from ArduPilot to be fed into ROS's navigation algorithm

Rover Autotune
--------------

This project involves adding an AutoTune mode to the Rover firmware similar to `Copter's AutoTune mode <https://ardupilot.org/copter/docs/autotune.html>`__ but simpler.  The focus should be on finding the best `turn rate <https://ardupilot.org/rover/docs/rover-tuning-steering-rate.html>`__ and `speed controller <https://ardupilot.org/rover/docs/rover-tuning-throttle-and-speed.html>`__ parameters.  The likely solution will be to provide turn rate or speed targets to the controllers for a short period, measure the response of the vehicle, adjust the gains and repeat until acceptable gains are found.

This project probably requires a good understanding of PID objects and control.

Improve fixed-wing 3D aerobatics support in ArduPilot
-----------------------------------------------------

With the addition of prop-hang in ArduPilot (`see here <https://discuss.ardupilot.org/t/ardupilot-flying-3d-aircraft-including-hovering/14837>`__) we now have the beginnings of a nice 3D aerobatics for fixed wing.
This project involves taking that to the next level to add support for "trick" mode. In trick mode, the user will have access to a variety of common 3D maneuvers, including knife-edge, loops, harrier and rolling loops. Implementing this will involve some careful use of quaternion controllers, but a good UI design so the stick inputs to control these tricks are easy to learn.
Testing can be done in the FlightAxis simulator (as in the above video), allowing for development without risking real aircraft.

Improve Morse simulator integration including supporting boats / ROVs
---------------------------------------------------------------------

Improve ArduPilot's integration with :ref:`Morse simulator <sitl-with-morse>` software including

- Adding support for boats and ROVs with simulated waves to test ArduPilot controls
- Default camera view to follow the vehicle

Unified performance counter on ArduPilot
----------------------------------------

This project would involve adding unified support for performance across our HAL.
Currently, the Linux board gets the most performant performance counter, but we should be able to some on Chibios and SITL to allow better profiling of the code.

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
