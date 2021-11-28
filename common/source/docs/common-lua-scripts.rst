.. _common-lua-scripts:

[copywiki destination="plane,copter,rover,dev"]

===========
Lua Scripts
===========

ArduPilot has introduced support for `Lua <https://www.lua.org/>`_ scripting.
Scripting provides a safe, "sandboxed" environment for new behaviors to be added to the autopilot without modifying the core flight code.
Scripts are stored on the SD card and run in parallel with the flight code.

This page describes how to setup scripts on your autopilot, the scripting API, scripting applets,and some examples to help get started.

While scripting normally requires at least some LUA programming and editing, there are some **Applets** available which require no user editing and are ready to use. See links below.

.. note::

    Lua scripting support was released with Copter-4.0, Rover-4.0, and Plane-3.9.11.

.. toctree::
    :maxdepth: 1

    API Documentation <common-lua-API>
    LUA Bindings <common-lua-binding-syntax>

Getting Started
---------------

- Ensure your autopilot has at least 2 MB of flash and 70 kB of memory.  High powered autopilots like the Hex Cube Orange and HolyBro Durandal will certainly work well but check the specifications of your :ref:`autopilot <common-autopilots>`.
- Set :ref:`SCR_ENABLE <SCR_ENABLE>` to 1 to enable scripting (refresh or reboot to see all ``SCR_`` parameters).
- Upload scripts (files with extension .lua) to the autopilot's SD card's ``APM/scripts`` folder.

  - If this folder does not exist, you can create it by setting :ref:`SCR_ENABLE<SCR_ENABLE>` to 1 and rebooting. This folder can also be created manually on the SD card. 
  - If using Mission Planner, this can be done using MAVFTP.
  - If using a simulator, the ``scripts`` folder is in the directory the simulator was started from. MAVFTP will work in this case as well.

  .. image:: ../../../images/scripting-MP-mavftp.png
      :target: ../_images/scripting-MP-mavftp.png
      :width: 450px

- **Sample Scripts** can be found `here for stable Plane <https://github.com/ArduPilot/ardupilot/tree/ArduPlane-stable/libraries/AP_Scripting/examples>`__, `Copter <https://github.com/ArduPilot/ardupilot/tree/ArduCopter-stable/libraries/AP_Scripting/examples>`__ and `Rover <https://github.com/ArduPilot/ardupilot/tree/APMrover2-stable/libraries/AP_Scripting/examples>`__.  The latest development scripts can be found `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples>`__.

- **Applets** which are scripts that require no user editing before use, can be found `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets>`_ . Each of these has a .md file of the same name detailing its capabilities, use, and setup. For example, there is a script to allow a user to change a SmartAudio capable video transmitter's output power level from a transmitter channel and set its power-up value via parameter.

- Up to 8 RC channels can be assigned as scripting inputs/controls using the``RCX_OPTION`` = "300-307" options. In addition, four dedicated script parameters are available: :ref:`SCR_USER1<SCR_USER1>` thru :ref:`SCR_USER4<SCR_USER4>` and are accessed with the same method as any other parameter, but these are reserved for script use.
- When the autopilot is powered on it will load and start all scripts.
- Messages and errors are sent to the ground station and, if using Mission Planner, can be viewed in the Data screen's "Messages" tab.
- :ref:`SCR_HEAP_SIZE <SCR_HEAP_SIZE>` can be adjusted to increase or decrease the amount of memory available for scripts. The default of 43 kB is sufficient for small scripts and fits onto most autopilots. The autopilot's free memory depends highly upon which features and peripherals are enabled. If this parameter is set too low, scripts may fail to run. If set too high other autopilot features such as Terrain Following or even the EKF may fail to initialize. On autopilots with a STM32F4 microcontroller, Smart RTL (Rover, Copter) and Terrain Following (Plane, Copter) need to be nearly always disabled. These features are usually enabled by default, set :ref:`SRTL_POINTS <SRTL_POINTS>` = 0, :ref:`TERRAIN_ENABLE <TERRAIN_ENABLE>` = 0).

What Scripts Can Do
-------------------

- Multiple scripts can be run at once
- Monitor the vehicle state
- Start to manipulate vehicle state

Lua is a garbage collected, imperative programming language.
Scripts are run at a low priority on the system, which ensures that the core flight code will continue to execute even if a script is taking a long time to process.
In order to allow multiple scripts to be run, each script is allocated a fixed time chunk (measured in VM instructions) in which it is expected to complete it’s processing.
The script is allowed to specify a new callback function and time at which it should be called, which allows it to be run continuously.
While this serves to bound the worst-case execution time safely, scripts are not guaranteed to be run on a reliable schedule.

Each script is run in its own "sandboxed" environment.
This allows each script to manipulate any global variables it wants to without causing other scripts problems, and allows the script to persist state between calls to its callback.
A side effect of this however is that heavy memory usage in a script can cause other scripts to be short on memory.

A Sample Script
---------------

The following script is used to change a servo output based on the aircraft's distance from the home position.

.. code-block:: lua
   :linenos:

   function update () -- periodic function that will be called
     local current_pos = ahrs:get_position() -- fetch the current position of the vehicle
     local home = ahrs:get_home()            -- fetch the home position of the vehicle
     if current_pos and home then            -- check that both a vehicle location, and home location are available
       local distance = current_pos:get_distance(home) -- calculate the distance from home in meters
       if distance > 1000 then -- if more then 1000 meters away
         distance = 1000;      -- clamp the distance to 1000 meters
       end
       servo.set_output_pwm(96, 1000 + distance) -- set the servo assigned function 96 (scripting3) to a proportional value
     end
   
     return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
   end

   return update, 1000   -- request "update" to be the first time 1000 milliseconds (1 second) after script is loaded

This script begins by defining the update function, which will be called periodically.
This function fetches the vehicles current position and home position, and stores them within the local variables current_pos and home.
Both of these variables now either hold a Location userdata, or are nil, if the information was not currently available from the vehicle.
The variables are then checked to ensure that they were actually provided, before calculating the distance between the locations.
After a check to ensure that the distance is never measured as more then 1 kilometer away, the distance is used to set the servo position.
The final line in the function is used to reschedule this function to be run again in 1000 milliseconds from the current time.
The last line of the script is also used to schedule the function to be run for the first time in 1000 milliseconds from the time at which it's loaded.

Script Crashes and Errors
-------------------------
If scripts run out of memory (or panic for any reason) all currently running scripts are terminated, and the scripting engine will restart, and reload all scripts from the disk.
This is allowed to happen at all flight stages, even while the vehicle is armed and flying.

Examples
--------

See the `code examples folder  <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples>`__

Include Lua scripts in firmware directly
----------------------------------------

Lua scripts may be directly embedded in firmware binaries, rather than being placed on an external SD card. This is useful for manufacturer's to provide the scripts for included peripherals or when the autopilot does not support an SD card. Instructions to do so are on `this link <https://ardupilot.org/dev/docs/common-oem-customizations.html#>`__.


For more background on technical decisions made before this point you can reference the presentation from the 2019 ArduPilot Unconference.

..  youtube:: ZUNOZMxOwsI
    :width: 100%

..  youtube:: 3n80dYoJQ60
    :width: 100%
