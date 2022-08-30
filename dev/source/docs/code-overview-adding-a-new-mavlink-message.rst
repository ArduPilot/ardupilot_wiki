.. _code-overview-adding-a-new-mavlink-message:

============================
Adding a new MAVLink Message
============================

Data and commands are passed between the ground station (i.e. Mission Planner, QGroundControl, MAVProxy, etc.) using the `MAVLink protocol <https://mavlink.io/en/>`__ over a serial
interface. This page provides some high-level advice for adding a new
MAVLink message.

These instructions have only been tested on Linux (to be precise a VM
running Ubuntu on a Windows machine). Instructions for setting up a VM
are on the :ref:`SITL page <setting-up-sitl-on-windows>`. If you can
run SITL, you should be able to follow the advice here. These
instructions will not run natively on Windows or a Mac.

**Step #1:** Ensure you have the latest ArduPilot code installed. Also,
check mavproxy. Mavproxy can be updated by running this command in a
Terminal window:

::

    sudo pip install --upgrade mavproxy

**Step #2:** Decide what type of message you want to add and how it will
fit in with the existing `MAVLink messages <https://mavlink.io/en/>`__.


For example, you might want to send a new navigation command to the
vehicle so that it can perform a trick (like a flip) in the middle of a
mission (i.e. in AUTO mode).  In this case, you would need a new
MAV_CMD_NAV_TRICK similar to the MAV_CMD_NAV_WAYPOINT definition
(search for "MAV_CMD_NAV_WAYPOINT" in the \ `MAVLink messages <https://mavlink.io/en/messages/common.html>`__ page).

Alternatively, you may want to send down a new type of sensor data from
the vehicle to the ground station.  Perhaps similar to the
`SCALED_PRESSURE <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE>`__
message.

**Step #3:** Add the new message definition to the
`common.xml <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml>`__
or
`ardupilotmega.xml <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml>`__
file in the MAVLink submodule.

If this command is generally useful, and will hopefully be added to the MAVLink protocol, then it
should be added to the
../modules/mavlink/message_definitions/v1.0/common.xml
file. If it is only for your personal use or only applicable to ArduPilot, then it should be added to the ardupilotmega.xml file.

**Step #4:** Add functions to the main vehicle code to handle sending or receiving the command to/from the ground station. A compile will be needed (ie. ./waf copter) to generate the MAVLink packet code so make sure to do that after editing the XML file. The MAVLink generation happens first, so it doesn't matter if the project compilation is successful or not due to other source code changes.

The top-level of this code will most likely be in the vehicle's
`GCS_MAVLink.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/GCS_Mavlink.cpp>`__
file or in the
`../libraries/GCS_MAVLink/GCS <https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS.h>`__
class.

In the first example where we want to add support for a new navigation
command (i.e., a trick) the following would be required:

-  Extend the
   `AP_Mission <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission>`__
   library's ``mission_cmd_to_mavlink()`` and
   ``mavlink_to_mission_cmd()`` functions to convert the MAVProxy
   command into an AP_Mission::Mission_Command structure.
-  Add a new case to the vehicle's
   commands_logic.cpp 's (`mode_auto.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_auto.cpp>`__ for ArduCopter) 
   ``start_command()`` and ``verify_command()`` functions to check for
   the arrival of the new ``MAV_CMD_NAV_TRICK``. These should call two
   new functions that you create called ``do_trick()`` and
   ``verify_trick()`` (see below).
-  Create these two new functions,  do_trick() and verify_trick(),
   that somehow command the vehicle *to perform the trick* (perhaps by
   calling another function in
   control_auto.cpp (`mode_auto.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_auto.cpp>`__ for ArduCopter) 
   that sets the auto_mode variable and then calls a new
   ``auto_trick_start()`` function).  The ``do_trick()`` function will
   be called when the command is first invoked.  The ``verify_trick()``
   will be called at 10hz (or higher) repeatedly until the trick is
   complete.  The ``verify_trick()`` function should return true when
   the trick has been completed.
   
**Step #5:** Decide how you are going to handle the message at the GCS. One of the
simplest ways is to use Mavproxy. MavProxy uses pymavlink to define the MAVLink messages,
so you will need to rebuild pymavlink to include your custom message. 
 
 - Remove the currently installed version of pymavlink. ``pip uninstall pymavlink``
 - Install the updated version. CD to ``ardupilot/modules/mavlink/pymavlink``
   and run ``python setup.py install --user``
 - Mavproxy is now capable of sending or receiving the new message. To ask it
   to print out or send your message you need to implement a module. Modules
   are python plugins that allow you to add functionality to Mavproxy. By default
   on Ubuntu they are located in ``/usr/local/lib/python2.7/dist-packages/MAVProxy/modules/``.
   Here is an example of a module that prints the contents of a MY_CUSTOM_PACKET message. Look
   at the other modules for examples on how to trigger sending of messages using the command
   line interface.
 
.. code-block:: python
 
     #!/usr/bin/env python
    '''Custom'''

    import time, os

    from MAVProxy.modules.lib import mp_module
    from pymavlink import mavutil
    import sys, traceback

    class CustomModule(mp_module.MPModule):
        def __init__(self, mpstate):
            super(CustomModule, self).__init__(mpstate, "Custom", "Custom module")
            '''initialisation code'''

        def mavlink_packet(self, m):
            'handle a MAVLink packet'''
            if m.get_type() == 'MY_CUSTOM_PACKET':
                print "My Int: %(x).2f" % \
                    {"x" : m.intField}

    def init(mpstate):
        '''initialise module'''
        return CustomModule(mpstate) 
    

.. warning::

   If the message you added has an ID greater that 255 you will need to enable MAVLink 2 support. 
   This can be done by setting the relevant ``SERIALn_PROTOCOL`` parameters (e.g. ``SERIAL1_PROTOCOL``) to 2 and starting Mavproxy with the ``--mav20`` argument.

**Step #6:** Consider contributing your code back to the main code base.
Discuss this with other developers on `Discord <https://ardupilot.org/discord>`__ and/or
:ref:`raise a pull request <submitting-patches-back-to-master>`. If
you raise a pull request it is best to separate the change into at least
two separate commits. One commit for the changes to the .xml files
(i.e Step #3) and another for the changes to the vehicle code.
