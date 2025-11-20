.. _frame-type-configuration:

==================================
Frame Class and Type Configuration
==================================

MultiCopters
============

The :ref:`FRAME_CLASS <FRAME_CLASS>` and :ref:`FRAME_TYPE <FRAME_TYPE>` parameters should be set to match the physical frame being used.  See the :ref:`Connect ESCs and Motors <connect-escs-and-motors>` page for the list of supported multicopter frames

If using the mission planner select Initial Setup, **Mandatory Hardware \| Frame Type**.  If you are using another Ground Station it should be possible to set the :ref:`FRAME_CLASS <FRAME_CLASS>` and :ref:`FRAME_TYPE <FRAME_TYPE>` parameters directly through a parameter update screen.

.. figure:: ../images/MissionPlanner_Select_Frame-Type.jpg
   :target: ../_images/MissionPlanner_Select_Frame-Type.jpg

.. note::

   For Traditional Helicopters, "Heli" should already be selected and it should not be changed.
   For :ref:`Single Copter and Coax Copter <singlecopter-and-coaxcopter>` the :ref:`FRAME_CLASS <FRAME_CLASS>` parameter should be set directly from the Full Parameter List until `this issue <https://github.com/ArduPilot/MissionPlanner/issues/1552>`__ is resolved.

Next select the frame "Type" for your vehicle. The default type is **X**.

For Tricopters, Y6, Traditional Helicopters, Bicopters, SingleCopters and CoaxCopters the frame type is ignored.

Motor Order Diagrams
--------------------

See the :ref:`Connect ESCs and Motors <connect-escs-and-motors>` page for the list of supported multicopter frames

Traditional Helicopters
=======================

Traditional Helicopters run a different version of firmware. See :ref:`traditional-helicopters` for setup and tuning information. 

HeliQuads are a hybrid frame using Traditional Helicopter firmware but four motors.

.. toctree::
   :maxdepth: 1

   HeliQuads <heliquads>

Other Configurations
====================

.. toctree::
   :maxdepth: 1

   Tricopter <tricopter>
   SingleCopter and CoaxCopter <singlecopter-and-coaxcopter>

Ardurocket Drone Configurations
====================

The model rocketry user community is mostly unaware of the significant capability of Ardupilot (as is) for RocketPlanes and Ardudrone (as is) for RocketDrones.
The mechanical equivalence of a rocket drone (a 66mm tube, fins, and nose cone built around a top and bottom coupler tube and quad drone X frame) is a short period of vertical powered thrust which operates similar to the toss mode (not recommended), but with higher drag and acceleration of 2-4 seconds. Prototypes in England have successfully been flown and recovered with simple and super simple drone control modes.  It is also possible to use stabilize mode, which works just like a regular drone in vertical flight.  Altitude hold and other modes are not recommended until the air vehicle is at peak altitude (apogee).  At this point a motor delay charge can deploy a parachute or streamer, or the parachute deployment of ardudrone can operate a parachute clamshell nose cone servo.  With an ejection charge, afer reaching apogee and deploying parachute recovery, the drone is carefully guided with hover power settings to a return spot landing.  If the rocket drone is powered with motors capable of 2x mass*g (thrust to weight >2) the motor ejection charge can be ignored (0) or ported without parachute, and the rocket drone can be recovered with any of the drone recovery modes.  It is extremely easy to configure and test prototypes of the rocketdrone.  Load ardudrone on any compatible flight controller which can be mounted on a quad X frame (Pixhawk, Matek, and my favorite Omnibus F4 Pro V3) using bootloaders as described in each flight controller configuration page, and then loading Copter using Mission Planner software. Configure a rocket drone just like a quad X frame, and most of the same modes and configuration parameters are identical for vertical flight orientation.  It is a little harder to know what is front and back, so use orientation lights on the frame or select simple control modes.    

Any model rocket large enough to hold an ardupilot compatible flight controller can use a significant number of the existing modes, barometer, accelerometer, stability, and telemetry capabilities without any required code changes. Work is being done to fork and merge model rocket capability into ardupilot and ardudrone for thrust vector control (TVC) ascent and descent stability capability, and for American Rocketry Challenge (ARC) airbrake and altitude control systems which do require some changes in codebase, pull calls, and possible merges.  Ardurocket code which was forked and modified in 2012 was not very compatible with the community framework, and tried to use fin vector control, which is not automated and mostly unstable when controlled by human manual modes.  The current effort simply focuses on using the software as is, and to only provide automated control for TVC stability, airbrakes, telemetry, and ejection events (no active guidance to GPS targeting or waypoints).  

.. toctree::
   :maxdepth: 1

   RocketDrone <rocketdrone>
