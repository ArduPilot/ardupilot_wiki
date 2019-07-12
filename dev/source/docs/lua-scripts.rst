.. _lua-scripts:

===========
Lua Scripts
===========

ArduPilot has introduced experimental support for `Lua <https://www.lua.org/>`_ scripting.
The scripting layer provides a safe, sandboxed environment for new behaviours to be added to the autopilot.
Lua scripts are run in parallel with the flight code on the autopilot. Lua is a garbage collected, imperative programming language.

What can scripts currently do:

- Multiple scripts can be run at once
- Monitor the vehicle state
- Start to manipulate vehicle state

Scripts are run at a low priority on the system, which allows the RTOS to ensure that the core flight code will continue to execute even if a script is taking a long time to process.
In order to allow multiple scripts to be run, each script is allocated a fixed time chunk (measured in VM instructions) in which it is expected to complete it’s processing.
The script is allowed to specify a new callback function and time at which it should be called, which allows it to be run continuously.
While this serves to bound the worst case execution time safely, scripts are not guaranteed to be run on a reliable schedule.

Each script is run in its own sandboxed environment.
This allows each script to manipulate any global variables it wants to without causing other scripts problems, and allows the script to persist state between calls to its callback.
A side effect of this however is that heavy memory usage in a script can cause other scripts to be short on memory.

This page describes how to setup scripts on your flight controller, the scripting API and some examples to help get started.

This feature will be released with Copter-3.7, Rover-3.6 and Plane-3.11.

Requirements
============

- Flight controller with at least 2 MB’s of flash available
- At least 32 KB of free memory

How to Enable Scripting
=======================

- Use a recent build of master, or compile ArduPilot for yourself
- Set the SCR_ENABLE parameter to 1 to enable scripting
- The HEAP_SIZE parameter can be adjusted to increase or decrease the amount of memory available for scripting. The amount of free memory is highly dependent upon what hardware and peripherals you have, and to test the amount of memory scripts require it is important to ensure you have all the hardware connected and enabled you expect to use with the vehicle. This defaults to 32KB which is sufficient to start with small scripts, and generally seems to fit onto embedded boards.
- When the vehicle is started it will automatically load and start all scripts (files with extension .lua) in the scripts folder. With the simulator the scripts folder is expected to be found in whatever directory the simulation is running in, on real hardware it’s expected to be inside the APM folder on the micro SD card.

A Sample Script
===============

The following script is used to change a servo output based on the aircraft's distance from the home position.

.. code-block:: lua
   :linenos:

   function update () -- periodic function that will be called
     local current_pos = ahrs:get_position() -- fetch the current position of the vehicle
     local home = ahrs:get_home()            -- fetch the home position of the vehicle
     if current_pos and home then            -- check that both a vehicle location, and home location are available
       local distance = current_pos:get_distance(ahrs:get_home()) -- calculate the distance from home in meters
       if distance > 1000 then -- if more then 1000 meters away
         distance = 1000;      -- clamp the distance to 1000 meters
       end
       servo.set_output_pwm(96, 1000 + distance) -- set the servo assigned function 96 (scripting3) to a proportional value
     end
   
     return update, 1000 -- request to be rerun again 1000 milliseconds (1 second) from now
   end

   return update, 1000   -- request to be rerun again 1000 milliseconds (1 second) from now 

This script begins by defining the update function, which will be called periodically.
This function fetches the vehicles current position and home position, and stores them within the local variables current_pos and home.
Both of these variables now either hold a Location userdata, or are nil, if the information was not currently available from the vehicle.
The variables are then checked to ensure that they were actually provided, before calculating the distance between the locations.
After a check to ensure that the distance is never measured as more then 1 kilometer away, the distance is used to set the servo position.
The final line in the function is used to reschedule this function to be run again in 1000 milliseconds from the current time.
The last line of the script is also used to schedule the function to be run for the first time in 1000 milliseconds from the time at which it's loaded.

Script Crashes and Errors:
==========================
If scripts run out of memory (or panic for any reason) all currently running scripts are terminated, and the scripting engine will restart, and reload all scripts from the disk.
This is allowed to happen at all flight stages, even while the vehicle is armed and flying.

API Documentation:
==================

The API documentation described here is not a complete list, but rather a work in progress. This list will be expanded upon in the near future. For a full list of the methods currently available, the `binding generator <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/generator/description/bindings.desc>`_ source is the best reference currently available.

Location
~~~~~~~~

Location is a userdata object that holds locations expressed as latitude, longitude, altitude. The altitude can be in several different frames, relative to home, absolute altitude above mean sea level, or relative to terrain. To create a new Location userdata you can call Location() to allocate an empty location object, or call a method that returns one to you.

A Location userdata object supports the following calls:

- :code:`lat([new_lat])` - If called with no arguments, returns the current latitude in degrees * 1e7 as an integer. If called with one argument it will assign the value to the latitude.

- :code:`lng([new_lng])` - If called with no arguments, returns the current longitude in degrees * 1e7 as an integer. If called with one argument it will assign the value to the longitude.

- :code:`relative_alt([is_relative])` - If called with no arguments returns true if the location is planned as relative to home. If called with a boolean value this will set the relative altitude.

- :code:`terrain_alt([is_terrain])` - If called with no arguments returns true if the location is planned as relative to terrain. If called with a boolean value this will flag if the altitude should be relative to terrain.

- :code:`origin_alt([is_origin])` - If called with no arguments returns true if the location is planned in absolute altitude relative to mean sea level. If called with a boolean value this will set the altitude to be relative to mean sea level.

- :code:`loiter_xtrack([should_xtrack])` - If the location is used for a loiter location this flags if the aircraft should track from the center point, or from the exit location of the loiter.

- :code:`get_distance(other_loc)` - Given a Location this calculates the horizontal distance between the two locations in meters.

- :code:`offset(offset_north, offset_east)` - Translates this Location by the specified number of meters.

- :code:`get_vector_from_origin_NEU()` - Returns nil or Vector3f that contains the offset from the EKF origin to this location. This will return nil if the EKF origin wasn’t available at the time this was called.

AHRS
~~~~

The ahrs library represents the Attitude Heading Reference System computed by the autopilot. It provides estimates for the vehicles attitude, and position.

- :code:`get_position()` - Returns nil or Location userdata that contains the vehicles current position. Note: This will only return a Location if the system considers the current estimate to be reasonable.

- :code:`get_home()` - Returns a Location that contains the vehicles current home waypoint.

- :code:`get_gyro()` - Returns a Vector3f containing the current smoothed and filtered gyro rates (in radians/second)

- :code:`get_hagl()` - Returns nil, or the latest altitude estimate above ground level in meters

- :code:`wind_estimate()` - Returns a Vector3f containing the current wind estimate for the vehicle.

- :code:`groundspeed_vector()` - Returns a Vector2f containing the vehicles velocity in meters/second in north and east components.

Further Information
===================

For more background on technical descisions made before this point you can reference the presentation from the 2019 ArduPilot Unconference.

..  youtube:: ZUNOZMxOwsI
    :width: 100%

