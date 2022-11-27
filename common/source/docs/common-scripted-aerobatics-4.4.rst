.. _common-scripted-aerobatics-4.4:

[copywiki destination="plane,copter,rover,blimp"]

===================
Scripted Aerobatics
===================


.. note:: this capability is in ArduPlane 4.2 and higher

ArduPilot has the capability of executing aerobatics from a LUA script. Either via AUTO mode mission items, or via scripts executed in many normal plane flight modes, usually controlled via a transmitter switch and selection RC channels.

.. note:: this requires a vehicle capable of aerobatic flight, properly tuned pitch,roll, and yaw rate controllers, and the pilot should be capable of safely taking over control (by switch or mode change) the vehicle while in an unusual attitude, if the need arises. The ``ACRO_x_RATE`` parameters limit the maximum commanded rate for each axis, as in ACRO mode.

Mission NAV_SCRIPT_TIME 
=======================

The SCRIPT_TIME waypoint type allows for a :ref:`LUA<common-lua-scripts>` script to take control of the roll/pitch/yaw rates of the vehicle for a time. For protection against lack of an appropriate script being active (ie was not loaded in the APM/Scripts directory of the SD card) to assume control, or LUA script crash due to error, there is a mandatory parameter which provides a maximum time the script can takeover, before the next mission item is executed. An additional layer of protection is provided within the firmware execution of the NAV_SCRIPT_TIME command that requires the script to access it within every 200ms, so the command's timeout parameter (P2) can be set to 0 (unlimited time out) for longer running aerobatics sequences (> 255 seconds), IF they are known to be otherwise well-behaved and present on the SD card.

The format of the SCRIPT_TIME item is:

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Mission Planner Field</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>
   </td>
   <td>Command ID number, accessible from running script to identify requested maneuver</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td></td>
   <td>Maximum time in seconds (1-255) before next mission item is processed. A value of 0 is unlimited time and should be used only with verified scripts</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td>arg1</td>
   <td>Parameter which is accessible from running script for maneuver use a first variable.</td>
   </tr>
   <tr>
   <td><strong>param4</strong></td>
   <td></td>
   <td>Parameter which is accessible from running script for maneuver use as a second variable.</td>
   </tr>
   <tr>
   <td><strong>param5</strong></td>
   <td>arg3</td>
   <td>Parameter which is accessible from running script for maneuver use as a third variable.</td>
   </tr>
   <tr >
   <td><strong>param6</strong></td>
   <td>arg4</td>
   <td>Parameter which is accessible from running script for maneuver use as a fourth variable.</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param7</strong></td>
   <td>Alt</td>
   <td>Target altitude</td>
   </tr>
   </tbody>
   </table>

A LUA script would need to be running when the above mission command is encountered to recognize that it needs to take control, and start sending pitch/roll/yaw rate inputs into the flight controller to control its attitude and/or trajectory. It can use the four variable input parameters of the command to control aspects of the maneuver, such as rate of rolls/loops, repeat counts, time for certain segments of the maneuver, etc. An example is provided in the LUA Scripting Applet directory called `plane_aerobatics.lua <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets/Aerobatics/FixedWing>`__ for several types of tricks and full aerobatic schedules.

The execution of a script running during the NAV_SCRIPT_TIME item can easily be interrupted by switching modes out of AUTO.

.. note:: returning to AUTO mode would resume the AUTO mission at the last item before switching modes. If this was a NAV_SCRIPT_TIME command it would re-start from the beginning.

.. note:: be sure to have valid waypoints before and after a NAV_SCRIPT_TIME sequence of commands in order to establish path for the scripts to use (optionally) and to have sufficient altitude before trying this feature.

.. tip:: if you create your mission, and place the LUA script you intend to use in the /scripts subdirectory of a SITL simulation, you can test the script and the mission without risking your vehicle.

Trajectory Precise Plane Aerobatics LUA script (firmware 4.4 and higher)
========================================================================

A trajectory precise, plane aerobatics script example is provided in the LUA Scripting Applet directory called `plane_aerobatics.lua <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets/Aerobatics/FixedWing>`__ . This LUA applet has a multitude of maneuvers, as well as complete sequences of routines, and the ability to easily create and add new sequence of routines.

These tricks and sequences can be either run as part of a mission using the NAV_SCRIPT_TIME mission command above, or activated by a switch on the RC transmitter.

The script provides numerous individual tricks, and a table of their IDs is provided in the `README.md <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets/Aerobatics/FixedWing/README.md>`__ file. Below is a list, but future expansion is anticipated so check the README:

==== ========================  ======  ===========  ==========   ==========  ==========
 ID  Name                      Arg1    Arg2         Arg3         Arg4        Turnaround 
==== ========================  ======  ===========  ==========   ==========  ==========
 1   Figure Eight              radius  bank angle                            No
 2   Loop                      radius  bank angle   #   loops                No
 3   Horizontal Rectangle      length  width        radius       bank angle  No
 4   Climbing Circle           radius  height       bank angle               No
 5   vertical Box              length  height       radius                   No
 6   Immelmann (FastRoll)      radius                                        Yes
 7   Straight Roll             length  num rolls                             No
 8   Rolling Circle            radius  num rolls                             No
 9   Half Cuban Eight          radius                                        Yes
 10  Half Reverse Cuban Eight  radius                                        Yes
 11  Cuban Eight               radius                                        No         
 12  Humpty Bump               radius  height                                Yes        
 13  Straight Flight           length  bank angle                            No         
 14  Scale Figure Eight        radius  bank angle                            No         
 15  Immelmann Turn            radius                                        Yes        
 16  Split-S                   radius                                        Yes        
 17  Upline-45                 radius  height gain                           No         
 18  Downline-45               radius  height loss                           No         
 19  Stall Turn                radius  height       direction                Yes        
 20  Procedure Turn            radius  bank angle   step-out                 Yes        
 21  Derry Turn                radius  bank angle                            No         
 22  Two Point Roll            length                                        No         
 23  Half Climbing Circle      radius  height       bank angle               Yes        
 24  Crossbox Humpty           radius  height                                Yes
 25  Laydown Humpty            radius  height                                Yes        
 25  Barrell Roll              radius  length       # spirals                No
 26  Straight Hold             length  bank angle                            No
 29  Four Point Roll 	       length                          				 No
 30  Eight Point Roll 	       length                                  		 No
 31  Multi Point Roll 	       length  num points 			                 No  
 200 Test Suite (dont fly!)
 201 NZ Clubman Schedule
 202 FAI F3A P-23
 203 FAI F3C Scale Example
 204 AirShow 
==== ========================  ======  ===========  ==========   ==========  ==========

The IDs 200 and above are sequences of the tricks above.

An example is the Airshow Sequence, consisting of Loop/HalfReverseCubanEight/ScaleFigureEight/Immelmann/Roll/Split-S/RollingCircle/HumptyBump/HalfCubanEight/Upline45/Downline45/HalfReverseCubanEight:

.. image:: ../../../images/airshow.png


To use as part of a mission, you would insert a SCRIPT_TIME mission item as shown:

.. image:: ../../../images/nav_script_time.jpg


in the above example, once WP3 is reached the airshow will execute, and when finished, proceed to WP5.

Tricks on a Switch
------------------

This applet also provides a means to execute tricks or sequences via an RC Switch in normal modes like ACRO, LOITER, CRUISE, or FBWA.

Setup
-----

- Make sure you have Autotuned your plane and that its capable of doing aerobatics in ACRO mode well (Bixlers and flying wings only can do rolls and loops, generally, sine they have insufficient yaw authority and side fuselage area...ie must be able to knife edge for full capability). This includes Autotuning the new YAW rate controller. Enable :ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>` and set :ref:`ACRO_YAW_RATE<ACRO_YAW_RATE>` appropriately (90deg/s is a good start). When you Autotune, not only do pitch and roll, but also yaw by exercising the rudder, like pitch and roll.
- Enable scripting, on an autopilot that is capable (F7 or H7) with :ref:`SCR_ENABLE<SCR_ENABLE>` =1, reboot and set :ref:`SCR_HEAP_SIZE<SCR_HEAP_SIZE>` = 250000 and :ref:`SCR_VM_I_COUNT<SCR_VM_I_COUNT>` = 200000.
- Copy the `plane_aerobatic.lua <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets/FixedWing>`__ script into your SD card APM/scripts directory. Use the RAW view and copy to files on the SD card. Reboot.
- If you want to activate tricks and schedules from an RC switch, assign an RC channel to ``RCx_OPTION`` = 300 for the trick activation switch (low=disable,mid=show trick number selected, high= do it) and one for 301 which is trick selection. This normally a pot or slider, but can be any switch (which limits the number of tricks to the number of switch positions). Set the ``TRIK_ENABLE`` to 1 and reboot. Then set `TRIK_COUNT`` parameter to the number of tricks (11 maximum) to be selectable by the trick selection channel. Reboot.
- For each trick/schedule, set its corresponding ``TRIKn_ID/ARG1/ARG2/ARG3/ARG4`` parameters. For example, if you wish TRIK3 to be a Vertical Box, 50 meters tall, 100 meters long, with corner radii of 15 meters, set ``TRIK3_ID`` = 5, ``TRIK3_ARG1`` = 100, ``TRIK3_ARG2`` = 50, and ``TRIK3_ARG3`` = 15. Other TRIK3 argument values are ignored.
- Setting the activation switch to mid position will identify the trick number selected by the selection channel on the ground station or TX, if running :ref:`Yaapu <common-frsky-yaapu>` telemetry.
- You can abort out of the trick by putting activation switch low, change modes, or set selector to 0. A trick or sequence will also abort if the programmed trajectory is not being met with an error limit. This usually indicates a lack of sufficient vehicle tuning or power.

Remember: ALTITUDE IS YOUR FRIEND! dont attempt your first one below 200feet! and FPV is a good way to try it out first, if you have trouble with seeing the vehicle at that altitude.

Adding Custom Schedules
-----------------------

While adding new basic tricks requires modification of the plane_aerobatics.lua applet, adding combinations of the above included tricks is easily done via creation of a simple text file in the /scripts directory, or the root directory on the SD card where the plane_aerobatics.lua script is stored.

An example for a schedule similar to the AirShow schedule is included as `trick72.txt <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/Aerobatics/FixedWing/trick72.txt>`__ and would be executed as ``TRIKx_ID`` = 72 via switch or in an AUTO mission command. The schedule will display its "name" when started, and as each trick begins the "message" will sent to the GCS to indicate its start.

Note, that the "straight_align" command is not a trick, but rather a command as to when the next trick is to begin. Its parameter is meters from the schedules initial entry point. Positive numbers are meters away from that point in the entry direction on the ground track, while negative numbers are in the opposite direction on the track line. If the aircraft is already past that point in the desired direction along the track, the trick will begin immediately.

Tuning for Aerobatics
=====================

Normal ArduPilot Autotune provides a safe, stable PID tune for most vehicles. As such, its not optimized for precision aerobatics which require a tight tune. See below:

.. toctree::
   :maxdepth: 1
    
   Aerobatic Tuning <common-aerobatics-tuning>

Simulation
==========

Using SITL, especially in conjunction with :ref:`RealFlight<dev:sitl-with-realflight>` and a RealFlight model like the `AddictionX <https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Planes/Addiction_3D>`__, makes development and debugging much easier without risking a real vehicle. However, simulation can also be done without RealFlight using Mission Planner alone with ArduPilot's provided 3D plane model.

Below is a video showing simulation using Mission Planner and the built-in 3D plane simulation:

.. youtube:: soAGBNPhNQ

If you are using RealFlight for simulations, here is how to setup the Addiction 3D model to do scripted trajectory aerobatics:

.. youtube:: YB_zJ7laF7M

Script Calls into the firmware
==============================

Other scripts can be developed which allow control of the vehicle, either via NAV_SCRIPT_TIME mission items or from normal flight modes. The key calls are:

- Enabling scripting control override of roll/pitch/yaw rates and throttle with a specific call "vehicle:nav_scripting_enable(..)" which returns a boolean indicating  success or failure.

- Obtaining the arguments of a NAV_SCRIPT_TIME command using "vehicle:nav_script_time()" if running while in AUTO mode.

- Controlling the above rates and throttle with the "vehicle:set_target_throttle_rate_rpy(....)" function, which must be called regularly (at least every 200ms) to set the roll/pitch/yaw rates and throttle percentage. Failure to do so, will disable the control override and return control to the original flight mode. Changing flight modes also disables script control.


