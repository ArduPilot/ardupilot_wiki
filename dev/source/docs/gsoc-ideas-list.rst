.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2023
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2023 <https://summerofcode.withgoogle.com/>`__. These are only suggestions so if you have your own ideas then please discuss them on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__.  We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2023.

- Rover AutoTune
- Camera and Gimbal enhancements
- AI & ArduPilot development environment
- Multicopter Swarm Avoidance
- Improve custom firmware server including adding branch support and improve dependency handling

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

- Skills required: C++, Lua
- Mentor: Randy Mackay
- Expected Size: 175h or 350h
- Level of Difficulty: Hard
- Expected Outcome: Autotune mode added that automatically calculates the frame limits, speed control gains and steering control gains for `Ackermann <https://ardupilot.org/rover/docs/rover-motor-and-servo-connections.html#separate-steering-and-throttle>`__ and `skid steering vehicles <https://ardupilot.org/rover/docs/rover-motor-and-servo-connections.html#skid-steering>`__.

This project involves `adding a new AutoTune mode to the Rover firmware <https://ardupilot.org/dev/docs/rover-adding-a-new-drive-mode.html>`__ to calculate the frame limits, speed control and steering control gains.  This is essentially an automated version of the manual tuning process documented in `this section of the Rover wiki <https://ardupilot.org/rover/docs/rover-first-drive.html>`__.

Similar to `Copter's autotune mode <https://ardupilot.org/copter/docs/autotune.html>`__ and `VTOL-quicktune.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/VTOL-quicktune.lua>`__ script this new mode should include a state machine that provides various throttle and steering outputs and then monitors the response by checking the AHRS/EKF outputs.

The list of parameters that should be tuned includes:

- :ref:`ATC_ACCEL_MAX <rover:ATC_ACCEL_MAX>` and :ref:`ATC_DECEL_MAX <rover:ATC_DECEL_MAX>`
- :ref:`ATC_STR_RAT_FF <rover:ATC_STR_RAT_FF>`
- :ref:`ATC_STR_RAT_MAX <rover:ATC_STR_RAT_MAX>`
- :ref:`ATC_TURN_MAX_G <rover:ATC_TURN_MAX_G>`
- :ref:`CRUISE_SPEED <rover:CRUISE_SPEED>` and :ref:`CRUISE_THROTTLE <rover:CRUISE_THROTTLE>` (or :ref:`ATC_SPEED_FF <rover:ATC_SPEED_FF>`)

See `Issue #8851 <https://github.com/ArduPilot/ardupilot/issues/8851>`__

Some of the development can be completed using the :ref:`SITL simulator <sitl-simulator-software-in-the-loop>` but funding will also be provided for the RC car frame and autopilot

Camera and Gimbal enhancements
------------------------------

- Skills required: C++, mavlink
- Mentor: Randy Mackay
- Expected Size: 175h or 350h
- Level of Difficulty: Medium
- Expected Outcome: Improved camera and gimbal support in both pilot controlled and autonomous modes (Auto, Guided)

This project involves numberous small and medium sized enhancements and bug fixes to ArduPilot's camera and gimbal support (see `Camera enhancement <https://github.com/ArduPilot/ardupilot/issues/23151>`__ and `Gimbal enhancement <https://github.com/ArduPilot/ardupilot/issues/20985>`__ lists).  These include:

- Camera Zoom position support
- Camera Focus position support
- Improve compliance with `MAVLink Camera Protocol <https://mavlink.io/en/services/camera.html>`__ (see AP's :ref:`MAVLink Interface Camera Controls wiki <mavlink-camera>`)
- Multiple gimbal support (see `Mount: issues with multi-gimbal support <https://github.com/ArduPilot/ardupilot/issues/21665>`__)
- Resolve any other known issues with the Siyi, Gremsy drivers (e.g. `Siyi should use angle control <https://github.com/ArduPilot/ardupilot/issues/23149>`__)
- Support for pointing gimbal at :ref:`Circle center <copter:circle-mode>`
- DroneCAN gimbal driver (see `Connect a Gimbal via DroneCAN <https://github.com/ArduPilot/ardupilot/issues/22148>`__)
- Enhance SITL gimbal support including adding multiple gimbals to a RealFlight model
- Work with AP QGC developer to ensure AP provides all info required for new camera gimbal control screen
- Mission Planner fixes to `Camera Gimbal setup <https://github.com/ArduPilot/MissionPlanner/issues/3049>`__ and Payload Control screens

Funding will be provided for the required hardware which will likely include a camera gimbal, transmitter and autopilot.

Multicopter Swarm Avoidance
---------------------------

- Skills required: C++, python, mavlink
- Mentor: Peter Barker, Rishabh Singh
- Expected Size: 175h or 350h
- Level of Difficulty: Medium
- Expected Outcome: vehicles in a swarm should avoid each other

This project involves enhanceing ArduPilot's Copter software so that vehicles flying in a swarm avoid each other.  The control logic should run primarily on each drone's flight controller (e.g. not on the ground station nor a companion computer).

- AC_Avoidance class should be enhanced to consume the location and speed of other vehicles.  The "simple avoidance" feature (see :ref:`Copter's object avoidance wiki page <copter:common-object-avoidance-landing-page>`) should then cause the vehicle to stop before hitting another vehicle in most modes (Loiter, Auto, Guided, etc).  Ideally the vehicle should also backaway from other vehicles if they get too close.
- SITL should be used to develop and test this feature
- by centralising remote vehicle knowledge and generalising the follow database.  Allow AC_Avoidance to work on this new database

Once complete, it should be possible to run a demonstration in SITL in which three vehicle are visible on the map.  Two should be acting as obstacles (flying in Guided mode) while the third is flown by a pilot in Loiter mode.  We should be able to move the two "obstacle" vehicles around while the third vehicle will not run into the others regardless of what inputs the pilot provides.

Development should be possible with only an Ubuntu or Windows PC but funding for hardware will also be provided if required.

Custom Firmware Server
----------------------

The ArduPilot custom firmware server (see
`https://custom.ardupilot.org <https://custom.ardupilot.org>`__ ) was
developed during GSoC 2021, and has been extremely useful. We would
like to extend the functionalty to multiple branches and add automatic
dependency handling, as well as support for enabling Lua scripts and
setting default parameters.

The successful student will need strong python and web development skills.

ROS2 MAVROS support for ArduPilot
---------------------------------

Currently, there is no MAVROS equivalent for ROS2, with `OSRF <https://www.openrobotics.org>`__ quickly moving to make ROS2 the standard version of ROS, supporting it has become a growing interest in our community. An initial port of the basic features of MAVROS would be a big step towards integrating ArduPilot and ROS2.

Projects Completed in past years
--------------------------------

In 2022, students worked on these projects:

- `Rover autonomous docking <https://discuss.ardupilot.org/t/gsoc-2022-rover-autodocking-conclusion/90626>`__
- `ROS integration for Non-GPS navigation and off-board path-planning <https://discuss.ardupilot.org/t/gsoc-2022-update-ros-integration-for-non-gps-navigation-and-off-board-path-planning/86948>`__
- `Boat object avoidance with Luxonis AI camera <https://discuss.ardupilot.org/t/gsoc-2022-boat-object-avoidance-with-luxonis-ai-camera/91257>`__

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
