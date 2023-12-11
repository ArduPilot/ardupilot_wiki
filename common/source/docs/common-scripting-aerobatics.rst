.. _common-scripted-aerobatics:

[copywiki destination="plane"]
===================
Scripted Aerobatics
===================


.. note:: this capability is in ArduPlane 4.2 and higher. It is rapidly evolving. Its suggested that users uses load the current "Latest" firmware and refer to its matching doucmentation ( :ref:`common-scripted-aerobatics-4.4` ) for best perfoemance.

ArduPilot has the capability of executing aerobatics from a LUA script. Either via AUTO mode mission items, or via scripts executed in many normal plane flight modes, usually controlled via a transmitter switch and selection RC channels.

.. note:: this requires a vehicle capable of aerobatic flight, properly tuned pitch,roll, and yaw rate controllers, and the pilot should be capable of safely taking over control (by switch or mode change) the vehicle while in an unusual attitude, if the need arises. The ``ACRO_x_RATE`` parameters limit the maximum commanded rate for each axis, as in ACRO mode.

Mission NAV_SCRIPT_TIME 
=======================

The SCRIPT_TIME waypoint type allows for a :ref:`LUA<common-lua-scripts>` script to take control of the roll/pitch/yaw rates of the vehicle for a time. For protection against lack of an appropriate script being active (ie was not loaded in the APM/Scripts directory of the SD card) to assume control, or LUA script crash due to error, there is a mandatory parameter which provides a maximum time the script can takeover, before the next mission item is executed.

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
   <td>Maximu time in seconds before next mission item is processed</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td></td>
   <td>Parameter which is accessible from running script for maneuver use a first variable.</td>
   </tr>
   <tr>
   <td><strong>param4</strong></td>
   <td></td>
   <td>Parameter which is accessible from running script for maneuver use as a second variable..</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param5</strong></td>
   <td>Lat</td>
   <td>Target latitude</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>Lon</td>
   <td>Target longitude</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param7</strong></td>
   <td>Alt</td>
   <td>Target altitude</td>
   </tr>
   </tbody>
   </table>

A LUA script would need to be running when the above mission command is encountered to recognize that it needs to take control, and start sending pitch/roll/yaw rate inputs into the autopilot to control its attitude and/or trajectory. It can use the two variable input parameters of the command to control aspects of the maneuver, such as rate of rolls/loops, repeat counts, time for certain segments of the maneuver, etc. An example is provided in the LUA Scripting examples directory called `plane_aerobatics.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/Aerobatics/FixedWing/plane_aerobatics.lua>`__ for several types of tricks.

The execution of a script running during the NAV_SCRIPT_TIME item can easily be interrupted by switching modes out of AUTO.

.. note:: returning to AUTO mode would resume the AUTO mission at the last item before switching modes. If this was a NAV_SCRIPT_TIME command it would re-start from the beginning.

.. note:: be sure to have valid waypoints before and after a NAV_SCRIPT_TIME sequence of commands in order to establish path for the scripts to use (optionally) and to have sufficient altitude before trying this feature.

.. tip:: if you create your mission, and place the LUA script you intend to use in the /scripts subdirectory of a SITL simulation, you can test the script and the mission without risking your vehicle.

Here is a video of a SITL simulation of the above mentioned example script:

.. youtube:: CG2DSdWQ0mI

Tricks on a Switch
==================

In addition to allowing LUA scripts to take over the vehicles rate controllers during AUTO missions as described above, they can also take control in normal modes like CRUISE or FBWA. This requires that the script:

- Enable the  feature with a specific call "vehicle:nav_scripting_enable(..)" which returns a boolean indicating  success or failure.

- Call the "vehicle:set_target_throttle_rpy(....)" function regularly (at least every 50ms) to set the roll/pitch/yaw rates and throttle percentage. Failure to do so, will disable the control overide and return control to the original flight mode. Changing flight modes also disables script control.

This allows the implementation of "Tricks on a Switch" where an activation switch driving an RC channel is used to enable an aerobatics trick script, which is selected by another RC channel. An example system is given `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples/Aerobatics/Via_Switch>`_.

This consists of a small control script to read the RC channels for trick selection and activation, and individual trick maneuver scripts for up to 10 different tricks. This allows the exchange of tricks between users, with, at most, the change in a text editor of a given trick's ID number to fit into the new user's list of trick scripts loaded on his SD card.

In order to create new tricks, a user needs to have some LUA script creation knowledge, but the task is simplified since they can use existing tricks as templates for the creation of new tricks.

As above, using SITL, especially in conjunction with :ref:`RealFlight<dev:sitl-with-realflight>` and a RealFlight model like the `AddictionX <https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Planes/Addiction_3D>`__, makes development and debugging much easier without risking a real vehicle.

Below is an introductory video to "Trick on a Switch":


.. youtube:: XD81TRRgDUM

Here is a video on showing the development of a new trick for Tricks on a Switch:

.. youtube:: cyDHaNWepfs

Setup Instructions for Example Tricks on a Switch Scripts
---------------------------------------------------------

- Make sure you have Autotuned your plane and that its capable of doing aerobatics in ACRO mode well (Bixlers and flying wings only can do rolls and loops, generally....no yaw authority...ie should be able to knife edge for full capability). This includes Autotuning the new YAW rate controller. Enable :ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>` and set :ref:`ACRO_YAW_RATE<ACRO_YAW_RATE>` appropriately (90deg/s is a good start). When you Autotune, not only do pitch and roll, but also yaw by exercising the rudder, like pitch and roll.
- Enable scripting, on an autopilot that is capable (F7 or H7) with :ref:`SCR_ENABLE<SCR_ENABLE>` =1 and :ref:`SCR_HEAP_SIZE<SCR_HEAP_SIZE>` = 200000 (which will allow four trick scripts + AUTO mission scripting in the future).
- Copy the https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples/Aerobatics/Via_Switch scripts into your SD card APM/scripts directory. Use the RAW view and copy to files on the SD card using the same names.
- Assign an RC channel to ``RCx_OPTION`` = 300 for the trick activation switch (low=disable,mid=show trick number selected, high= do it) and one for 301 which is trick selection. You can use a three position for 0/5/10 trick id selection setup to ``RCx_OPTION`` = 301. 0 is disable, 5 is the knife edge, 10 is the loop/immelman example using the above scripts.
- Upon boot you should see messages that show the trick 5(knife-edge) and 10(loop/immelman) are loaded. Setting the activation switch to mid position will identify the trick number selected by the selection channel on the ground station or TX, if running :ref:`Yaapu <common-frsky-yaapu>` telemetry.
- In the Ground Control Station's parameter lists, you should see a set of "AERO" params....the ``AERO_TRICK_RAT`` sets the loop pitch rate, try 90deg/s, ``AERO_RPT_COUNT`` sets the number of loops or if 0, an immelman. ``AERO_TRICK_ANG`` set the knife-edge trick angle 0 to 180 or -180 deg that will be executed as long as you have the activation switch high. Using the GCS or :ref:`Yaapu GCS<common-yaapu-gcs>` or :ref:`CRSF parameter editor<common-crsf-telemetry>`, you can change these in air to vary the trick characteristics, if you wish
- You can bail out of the trick by putting activation switch low, change modes, or set selector to 0

Remember: ALTITUDE IS YOUR FRIEND! dont attempt your first one below 200feet! and FPV is a good way to try it out first, if you have trouble with seeing the vehicle at that altitude.

Tuning for Aerobatics
=====================

Normal ArduPilot Autotune provides a safe, stable PID tune for most vehicles. As such, its not optimized for precision aerobatics which require a tight tune. See below:

.. toctree::
   :maxdepth: 1

   Aerobatic Tuning <common-aerobatics-tuning>
