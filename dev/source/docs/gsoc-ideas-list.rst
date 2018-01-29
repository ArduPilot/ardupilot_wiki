.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2018
========================================

This is a list of projects suggested by ArduPilot developers for GSoC 2018. These are only suggestions, and if you have your own ideas then please discuss them on either the gitter channel (at https://gitter.im/ArduPilot/GSoC) or on the discuss server (see http://discuss.ardupilot.org/c/google-summer-of-code). 
We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2018. We're looking for enthusiastic students who can really get stuck into their project and make a substantial contribution to the ArduPilot project.

- `Object Avoidance improvements for Multicopters <http://ardupilot.org/dev/docs/code-overview-object-avoidance.html>`__ and Rovers including adding occupancy grid using `OctoMap <https://octomap.github.io/>`__ or `ROS <http://ardupilot.org/dev/docs/ros.html>`__.
- Path Planning around obstructions for Multicopters and Rovers.
- No-Fly / Stay-Out zones for multicopters and/or rovers
- Live video improvements for `APSync <http://ardupilot.org/dev/docs/apsync-intro.html>`__ including frame rates optimised for bandwidth and video stream discovery
- Improved IoT integration to allow live viewing of drone location on web page
- Improve flight control for `Single Copter or Coax Copter <http://ardupilot.org/copter/docs/singlecopter-and-coaxcopter.html>`__ to bring it to the same level of performance of other multicopters
- Improve SITL simulator to include a 3D viewer and objects
- Helicopter autorotation support
- AirSim drone simulator support for ArduPilot SITL (see https://github.com/Microsoft/AirSim/)
- JavaScript DataFlash log parser and a system for graphing user logs with similar capabilities to MAVExplorer, but hosted in users browsers
- FreeRTOS port of ArduPilot
- improve UAVCAN integration with ArduPilot
- work on 3D aerobatic support for fixed wing aircraft
- Cloud ground station for persistent control of a fleet of drones. Here is a Search and Rescue use case example: http://www.surtsey.org/projects/cloud-control-station/ 

More Details
============

The following sections give a bit more detail on some of the projects listed above.

Object Avoidance improvements for Multicopters and Rovers
---------------------------------------------------------

Multicopters and Rovers already include "simple" object avoidance (`here is how it works <http://ardupilot.org/dev/docs/code-overview-object-avoidance.html>`__) which stops the vehicle before hitting objects but it only works while the sonar/lidar/vision-system senses the object.  This causes problems when using lidar/sonar which cannot see in all directions in 3D.  The vehicle stops before hitting an object head-on but the user can then turn the vehicle 90degrees left or right (so the obstruction is out of view from the lidar) and then fly left or right into the obstruction.  A better approach would be to:

- build up a 3D map of areas where we have sensed obstructions using a program like `OctoMap <https://octomap.github.io/>`__ or `ROS <http://ardupilot.org/dev/docs/ros.html>`__.  It is likely only `high performance flight controllers <http://ardupilot.org/copter/docs/common-autopilots.html>`__ or on a `companion computer <http://ardupilot.org/dev/docs/companion-computers.html>`__ would be capable of running these programs (but this should be checked).
- use the above generated 3D map to decide which directions we can or cannot fly in and then provide this "which way is safe" information to the vehicle's avoidance/proximity libraries using mavlink DISTANCE_SENSOR messages.

Path Planning around obstructions for Multicopters and Rovers
-------------------------------------------------------------

The purpose of this project is to allow multicopters and rovers to move autonomously around obstructions.  There are two key moments when this is important:

- the vehicle is beginning its journey towards a "waypoint".  Currently our navigation controllers record the origin and destination and then send attitude and thrust commands (many times per second) to our lower level controllers in order to follow a straight line path.  In this project, we would add a new path-planning navigation library that would create a path (probably made up of multiple straight lines) based on known obstructions.  This obstruction data would initially include only terrain data (already available) but could be expanded to other sources in the future (i.e. see Stay Out Zones below).
- the vehicle becomes caught behind an obstruction based on real-time sonar/lidar information.  The vehicle could use "Path Planning" to try and find its way around the obstruction.  It might involve a "failsafe" being triggered if the vehicle becomes stuck for more than a few seconds.  The end location the vehicle is trying to get to would be provided to the path-planner and the path-planner would provide 3d movements (probably 3d velocity vectors) to move the vehicle around the obstruction.  This real-time path-planning feature likely requires the above "Object Avoidance improvements for Multicopters and Rover" to be completed so that the 3D map is available.

No-Fly / Stay-Out zones for Multicopters and/or Rovers
------------------------------------------------------

In this project, multicopters and rovers would stop and/or avoid "No-Fly" (aka "Stay-out") zones provided by the user or an external 3rd party data provider (like `Altitude Angel <https://www.altitudeangel.com/>`__).

- Zones would likely be defined as 3D cylinders or cubes with attributes including location (latitude, longitude, altitude) and size.
- The ground stations (i.e Mission Planner or QGroundControl) would be responsible for consolidating the user defined No-Fly zones with the 3rd party data provider information on the ground station computer.
- ArduPilot (running on the flight controller) would send mavlink messages to the ground station to request all the No-Fly zone information within a given area.  The reply messages would be processed and held in a small database on the flight controller similar to how we store Terrain data (`see AP_Terrain library <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Terrain>`__).
- The `AC_Avoidance library <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AC_Avoidance>`__ would be extended to stop the vehicle from entering these No-Fly zones similar to how we avoid hitting the `Fence <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AC_Fence>`__.
- In the future this No-Fly zone information might also be provided to a "Path Planner" (see "Path Planning around obstructions for Multicopters and Rover").

This is an important feature that will be required for regulatory purposes in some countries in coming years.  It is also one of our oldest user requests (see `issue 391 <https://github.com/ArduPilot/ardupilot/issues/391>`__ and `issue 1056 <https://github.com/ArduPilot/ardupilot/issues/1056>`__).

Live video improvements
-----------------------

Most users want live video transmitted from their vehicle to the ground station.  ArduPilot's `APSync <http://ardupilot.org/dev/docs/apsync-intro.html>`__ includes live video using `gstreamer <https://gstreamer.freedesktop.org/>`__ but its performance could be greatly improved by:

- modifying the frame rate and video quality based on the bandwidth
- allow the ground station to discover which video streams are available from the vehicle
- allow switching on/off streaming of video streams

Much work has already gone into the `Intel Camera Streaming Daemon <https://github.com/intel/camera-streaming-daemon>`__ so this could be a good starting point.

Improved IoT integration
------------------------

The purpose of this project is to make it much easier for ArduPilot vehicles to be integrated into the Internet-of-things by adding support for the `MQTT protocol <http://mqtt.org/>`__ either to ArduPilot directly or to `APSync <http://ardupilot.org/dev/docs/apsync-intro.html>`__ running on a `companion computer <http://ardupilot.org/dev/docs/companion-computers.html>`__.

If support was added directly to ArduPilot, it should be handled similarly to how we support the existing `MAVLink <http://qgroundcontrol.org/mavlink/start>`__ and `FrSky <http://ardupilot.org/copter/docs/common-frsky-telemetry.html>`__ protocols meaning that we would add a new library that knows how to consume and publish the mqtt messages, filling them in with data as required from ArduPilot's various subsystems like the GPS, accelerometers, etc.

If support was added to APSync, this project would best be handled by adding a mavlink/mqtt conversion program.  I.e. a translation layer that accepts mavlink from ardupilot and mqtt messages from external sources.

Developers interested in this project will likely first need to spend effort defining which mqtt messages we should support, their format and where this information can be found amongst ArduPilot's subsystems.  This will be made easier by some earlier attempts like `this one <https://github.com/ArduPilot/ardupilot/pull/6325>`__.

Improve Single Copter and/or Coax Copter
----------------------------------------

`Single Copter and Coax Copters <http://ardupilot.org/copter/docs/singlecopter-and-coaxcopter.html>`__ are vehicles with one or two motors on the top, along with 4 servo controlled fins below to direct the air.  ArduPilot already supports these vehicles and there have been some successful flights but their attitude controllers need more love and attention to bring them up to the level of performance of our other frame types.

This project would involve first running the vehicles in a simulator (probably `RealFlight8 <http://ardupilot.org/dev/docs/sitl-with-realflight.html#sitl-with-realflight>`__) and then testing on a real vehicle.

Developers looking to take on this project should have some understanding of control theory (PID controllers) and be prepared to do detailed analysis of dataflash logs of the simulated and real-flights to ensure our control methods match the physics of these vehicles.

Expenses for purchasing the simulator and vehicle will be provided by ArduPilot.

Add 3D Viewer to SITL
---------------------

The ArduPilot software in the loop simulator supports many physics backends. Some of those backends have nice 3D interfaces allowing the flight to be visualised, but the "built-in" physics backends don't have that. The built-in backends are very convenient for rapid development however, so it would be nice to have a way to visualise the vehicle when using those backends. 
We do have a workaround at the moment where we can visualise using FlightGear externally, but it doesn't provide as good a visualisation as we would like, and we have no way to add objects (such as buildings) which are part of the physics.
This projects would involve adding a 3D visualisation backend, along with support for objects in those backends that the physics simulation can interact with. The ability to load different 3D models of the vehicle would be a great bonus.

Helicopter auto-rotation support
--------------------------------

When the engine fails on a helicopter a good pilot can land the helicopter safely using auto-rotation. We would like ArduPilot to support doing this automatically. We already have a very nice simulation system for helicopters using the RealFlight FlightAxis backend, which gives the ideal test environment for developing this feature. The project would involve using the rotor RPM and motor RPM sensors in the simulator to produce a reliable auto-rotation from a variety of heights and flight speeds. If simulator testing goes well then it could be tested on a number of real helicopters.

Support for AirSim simulator
----------------------------

Microsoft recently released support for their AirSim drone simulator based on the Unreal 3D gaming engine. It looks like a very nice simulation framework, and we would like to add support for using it for ArduPilot development. The project would involve adding interface code between AirSim and ArduPilot, working with the AirSim developers if needed to enhance their APIs (such as adding lock-step scheduling). Please note that this project will require you to have a fast enough PC to run AirSim (good graphics card and lots of memory).

JavaScript Log Viewer
---------------------

We would like to be able to offer a browser-based log analysis and graphing tool for ArduPilot users. This will involve writing a JavaScript parser for the ArduPilot DataFlash log format and adding a nice graphing interface for browsers based on the parsed data. The user interaction model we are looking for is similar to the python based MAVExplorer (see http://ardupilot.org/dev/docs/using-mavexplorer-for-log-analysis.html), where arbitrary graph expressions can be used, along with selecting from a menu of common graphs.
Ideally the tool would also support MAVLink telemetry logs, using the pymaylink JavaScript code generator.
This tool will be combined with a log upload website to offer good log analysis for all ArduPilot users.

FreeRTOS port of ArduPilot
--------------------------

ArduPilot is based around a HAL (hardware abstraction layer). The HAL supports a number of operating systems, including NuttX, Linux and QURT. Adding FreeRTOS to the officially supported list of ports would be very nice. There is a FreeRTOS HAL port done by kwikius (see https://github.com/kwikius/ardupilot/tree/quantracker_master) that is flying on an adapted OSD board already (see http://discuss.ardupilot.org/t/aerflite-flight-controller-osd-new-years-eve-maiden-flight-running-arduplane/13840). That port would be a really good starting point for a fully supported ArduPilot FreeRTOS HAL. Two other people have attempted FreeRTOS ports with less success. The project would involve working on the ports to address the remaining issues and get something that can be accepted into ArduPilot master.
One interesting approach would be to get FreeRTOS running on a Pixhawk, allowing for a direct comparison between ArduPilot on NuttX and FreeRTOS on the same hardware.

Sensor-head Port of ArduPilot
-----------------------------

Quite a number of drones now have powerful "companion computers", along with microcontrollers (such as the STM32F4) for flight control. To allow for more advanced control and estimation code in ArduPilot it would be nice to support a new mode of operation where most of the flight code runs on the companion computer, and the microcontroller just acts as a "sensor head". 
As the companion computers usually run Linux already, the port of ArduPilot to that side is easy. The real work involves adding a UART based protocol between the companion computer and the microcontroller that would do the following:

- gather sensor data from all sensors
- get RC inputs from uarts and pins
- send outputs to motors and servos
- control the bi-directional data between the two CPUs

This project could be developed using a wide variety of hwardware. A simple setup would be a RaspberryPi with a pixhawk. Another very nice setup would be a Pixhawk2 with an Edison embedded.
One possible development approach would be as follows:
Run sensor drivers, PWM out and RC input on the STM32 which means we'd need to create a libraries/AP_SensorHead library that implements the sensorhead protocol.
steps would be

- draft the protocol
- implement basic design of AP_SensorHead
- add a libraries/AP_SensorHead/examples/SensorTest test prog, to test the protocol
- implement the protocol within STM32. I'd start by implementing it as an alternative serial output type within ArduPilot firmware (see SerialManager library). Later we may do a more bare metal fw for lower end STM32
- implement SensorHead backends for each of the key sensor libs. So for example libraries/AP_Baro/AP_Baro_SensorHead.cpp
- same for AP_InertialSensor, AP_Compass, AP_GPS

these will be very thin wrappers around calls inside libraries/AP_SensorHead/

Improve fixed-wing 3D aerobatics support in ArduPilot
-----------------------------------------------------

With the addition of prop-hang in ArduPilot (see http://discuss.ardupilot.org/t/ardupilot-flying-3d-aircraft-including-hovering/14837) we now have the beginnings of a nice 3D aerobatics for fixed wing.
This project involves taking that to the next level to add support for "trick" mode. In trick mode the user will have access to a a variety of common 3D maneuvers, including knife-edge, loops, harrier and rolling loops. Implementing this will involve some careful use of quaternion controllers, but a good UI design so the stick inputs to control these tricks are easy to learn.
Testing can be done in the FlightAxis simulator (as in the above video), allowing for development without risking real aircraft.

Projects Completed during GSoC 2017
-----------------------------------

- Smart Return-To-Launch which involves storing vehicle's current location and maintaining the shortest possible safe path back home.
- ChibiOS/RT port of ArduPilot
- Add "sensor head" operation of ArduPilot, split between two CPUs