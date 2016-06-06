.. _code-overview-adding-a-new-mavlink-message:

============================================
Adding a new MAVLink Message (Code Overview)
============================================

Data and commands are passed between the ground station (i.e mission
planner, Droid Planner, etc) using the `MAVLink protocol <https://en.wikipedia.org/wiki/MAVLink>`__ over a serial
interface. This page provides some high level advice for adding a new
MAVLink message.

These instructions have only been tested on Linux (to be precise a VM
running Ubuntu on a Windows machine). Instructions for setting up a VM
are on the :ref:`SITL page <setting-up-sitl-on-windows>`. If you can
run SITL, you should be able to follow the advice here. These
instructions will not run natively on Windows or a Mac.

**Step #1:** Ensure you have the latest ArduPilot code installed. Also
check mavproxy. Mavproxy can be updated by running this command in a
Terminal window:

::

    sudo pip install --upgrade mavproxy

**Step #2:** Decide what type of message you want to add and how it will
fit in with the existing `MAVLink messages <https://pixhawk.ethz.ch/mavlink/>`__.

For example you might want to send a new new navigation command to the
vehicle so that it can perform a trick (like a flip) in the middle of a
mission (i.e. in AUTO mode).  In this case you would need a new
MAV_CMD_NAV_TRICK similar to the MAV_CMD_NAV_WAYPOINT definition
(search for "MAV_CMD_NAV_WAYPOINT" in the \ `MAVLink messages <http://mavlink.org/messages/common>`__ page).

Alternatively you may want to send down a new type of sensor data from
the vehicle to the ground station.  Perhaps similar to the
`SCALED_PRESSURE <https://pixhawk.ethz.ch/mavlink/#SCALED_PRESSURE>`__
message.

**Step #3:** Add the new message definition to the
`common.xml <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml>`__
or
`ardupilotmega.xml <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml>`__
file in the mavlink submodule.

If this command will hopefully be added to the MAVLink protocol then it
should be added to the
../modules/mavlink/message_definitions/v1.0/common.xml
file. If it is only for your personal use or only for use with Copter,
Plane, Rover then it should be added to the ardupilotmega.xml file.

**Step #4:** Starting in Jan 2016 the source code is automatically generated when you compile the project but before that date you would cd to the ardupilot directory and then run this command to manually generate it.

``./libraries/GCS_MAVLink/generate.sh``

**Step #5:** Add functions to the main vehicle code to handle sending or receiving the command to/from the ground station. A compile will be needed (ie. make px4-v2) to generate the mavlink packet code so make sure to do that after editing the xml file. The mavlink generation happens first so it doesn't matter if the project compilation is successful or notdue to other source code changes.

The top level of this code will most likely be in the vehicle's
`GCS_MAVLink.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/GCS_Mavlink.cpp>`__
file or in the
`../libraries/GCS_MAVLink/GCS <https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS.h>`__
class.

In the first example where we want to add support for a new navigation
command (i.e. a trick) the following would be required:

-  Extend the
   `AP_Mission <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission>`__
   library's ``mission_cmd_to_mavlink()`` and
   ``mavlink_to_mission_cmd()`` functions to convert the MAVProxy
   command into an AP_Mission::Mission_Command structure.
-  Add a new case to the vehicle's
   `commands_logic.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/commands_logic.cpp>`__'s
   ``start_command()`` and ``verify_command()`` functions to check for
   the arrival of the new ``MAV_CMD_NAV_TRICK``. These should call two
   new functions that you create called ``do_trick()`` and
   ``verify_trick()`` (see below).
-  Create these two new functions,  do_trick() and verify_trick(),
   that somehow command the vehicle *to perform the trick* (perhaps by
   calling another function in
   `control_auto.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/control_auto.cpp>`__
   that sets the auto_mode variable and then calls a new
   ``auto_trick_start()`` function).  The ``do_trick()`` function will
   be called when the command is first invoked.  The ``verify_trick()``
   will be called at 10hz (or higher) repeatedly until the trick is
   complete.  The ``verify_trick()`` function should return true when
   the trick has been completed.

**Step #6:** Consider contributing your code back to the main code base.
Email the `drones-discuss email list <https://groups.google.com/forum/#!forum/drones-discuss>`__ and/or
:ref:`raise a pull request <submitting-patches-back-to-master>`. If
you raise a pull request it is best to separate the change into at least
two separate commits. One commit for the changes to the .xml files
(i.e Step #3) and another for the changes to the vehicle code.
