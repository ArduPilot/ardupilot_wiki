.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2017
========================================

This is a list of projects suggested by ArduPilot developers for GSoC 2017. These are only suggestions, and if you have your own ideas then please discuss them on either the gitter channel (at https://gitter.im/ArduPilot/GSoC) or on the discuss server (see http://discuss.ardupilot.org/c/google-summer-of-code). 
We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2017. We're looking for enthusiastic students who can really get stuck into their project and make a substantial contribution to the ArduPilot project.

- `Improved Object Avoidance for Multicopters <http://ardupilot.org/dev/docs/code-overview-object-avoidance.html>`__ and Rovers including adding occupancy grid
- Improved live video for `APSync <http://ardupilot.org/dev/docs/apsync-intro.html>`__ including video stream discover and publishing
- Improved IoT integration using `APSync <http://ardupilot.org/dev/docs/apsync-intro.html>`__ to allow live viewing of drone location on web page
- (DONE) Safe Return-To-Launch which will involve storing vehicle's current location and maintaining the shortest possible safe path back home.
- Improve flight control for `Single Copter or Coax Copter <http://ardupilot.org/copter/docs/singlecopter-and-coaxcopter.html>`__ to bring it to the same level of performance of other multicopters
- Improve SITL simulator to include a 3D viewer and objects
- Helicopter autorotation support
- Add support for the AirSim drone simulator to ArduPilot SITL (see https://github.com/Microsoft/AirSim/)
- Build a JavaScript DataFlash log parser and a system for graphing user logs with similar capabilities to MAVExplorer, but hosted in users browsers
- work on the FreeRTOS port of ArduPilot
- (IN PROGRESS) work on the ChibiOS/RT port of ArduPilot
- improve UAVCAN integration with ArduPilot
- work on 3D aerobatic support for fixed wing aircraft
- (IN PROGRESS) add support for "sensor head" operation of ArduPilot, split between two CPUs
- Create a cloud based GCS for persistent control of a fleet of drones. Here is a Search and Rescue use case example: http://www.surtsey.org/projects/cloud-control-station/ 

More Details
============

The following sections give a bit more detail on some of the projects listed above.

Improve Single Copter and/or Coax Copter
----------------------------------------

Single copter and coax copter are copter frame types that use one or two motors on the top, along with 4 servo controlled vanes below to direct the air. We added support for these in ArduPilot a while ago and there have been some successful flights, but it needs more work to improve the flight control.

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

