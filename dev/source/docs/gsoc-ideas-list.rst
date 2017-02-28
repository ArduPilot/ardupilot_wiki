.. _gsoc-ideas-list:

=======================================
List of Possible Projects for GSoC 2017
=======================================

- `Improved Object Avoidance for Multicopters <http://ardupilot.org/dev/docs/code-overview-object-avoidance.html>`__ and Rovers including adding occupancy grid
- Improved live video for `APSync <http://ardupilot.org/dev/docs/apsync-intro.html>`__ including video stream discover and publishing
- Improved IoT integration using `APSync <http://ardupilot.org/dev/docs/apsync-intro.html>`__ to allow live viewing of drone location on web page
- Safe Return-To-Launch which will involve storing vehicle's current location and maintaining the shortest possible safe path back home
- Improve flight control for `Single Copter or Coax Copter <http://ardupilot.org/copter/docs/singlecopter-and-coaxcopter.html>`__ to bring it to the same level of performance of other multicopters
- Improve SITL simulator to include a 3D viewer and objects
- Helicopter autorotation support
- Add support for the AirSim drone simulator to ArduPilot SITL (see https://github.com/Microsoft/AirSim/)
- Build a JavaScript DataFlash log parser and a system for graphing user logs with similar capabilities to MAVExplorer, but hosted in users browsers
- work on the FreeRTOS port of ArduPilot
- improve UAVCAN integration with ArduPilot
- work on 3D aerobatic support for fixed wing aircraft
- add support for "sensor head" operation of ArduPilot, split between two CPUs

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

