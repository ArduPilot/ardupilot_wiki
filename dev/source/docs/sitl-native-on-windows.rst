.. _sitl-native-on-windows:

==========================
Setting up SITL on Windows
==========================

SITL was originally developed on Linux, but can also be built and run *natively* Windows. It can also be run on a virtual machine (Linux) hosted on Windows, Mac OSX, or Linux.

This article shows how to build and run :ref:`SITL (Software In The Loop) <sitl-simulator-software-in-the-loop>` *natively* on Windows using Cygwin. These commands were tested on Windows 10 with `MAVProxy 1.5.0 <http://ardupilot.github.io/MAVProxy/>`__ and Copter-4.0.0-dev.

.. figure:: ../images/MAVProxy_Map_GuidedCopter.jpg
   :target: ../_images/MAVProxy_Map_GuidedCopter.jpg
   :width: 450px

   MAVProxy Map: Guiding a SimulatedCopter

Installation steps
==================

- Install Cygwin as described :ref:`here <building-setup-windows-cygwin>`
- `Download MAVProxy for Windows <https://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe>`__ (latest build), install the executable including accepting the license and all default installation options

.. tip::

   Makes sure that Cygwin is in your PATH. As for troubleshooting, you may add a Windows environment variable called CYGWIN_HOME in your path pointing to your c:\cygwin\bin (if you installed at "C:"). You may check it by doing a Right-click on "My Computer" -> Properties -> Advanced -> Environment Variables.

Running SITL
============
      
#. Open a Cygwin64 terminal from the start menu

#. Navigate to one of the vehicle directories (in this case Copter) and call ``sim_vehicle.py`` to start SITL.  If you get a windows security alert for the the firewall, allow the connection. 

   ::

       cd ~/ardupilot/ArduCopter
       ../Tools/autotest/sim_vehicle.py --map --console

#. SITL and *MAVProxy* will start. *MAVProxy* displays three windows:

   -  A command prompt in which you enter commands to SITL
   -  A Console which displays current status and messages
   -  A map that shows the current position of the vehicle and can be
      used (via right-click) to control vehicle movement and missions.

   .. image:: ../images/mavproxy_sitl_console_and_map.jpg
       :target: ../_images/mavproxy_sitl_console_and_map.jpg
       :width: 450px

   .. tip::

      It is useful to arrange the windows as shown above, so you can
      observe the status and send commands at the same time.

#. Send commands to SITL from the command prompt and observe the results on the map.

   -  Change to GUIDED mode, arm the throttle, and then takeoff:

      ::

          mode guided 
          arm throttle
          takeoff 40

      Watch the altitude increase on the console.

      .. note::

         Takeoff must start within 15 seconds of arming, or the
         motors will disarm.

   -  Change to CIRCLE mode and set the radius to 2000cm

      ::

          rc 3 1500
          mode circle
          param set circle_radius 2000

      Watch the copter circle on the map.

   -  When you're ready to land you can set the mode to RTL (or LAND):

      ::

          mode rtl

This is a very basic example. For links to more information on what you
can do with SITL and MAVProxy see the section: :ref:`Next Steps <sitl-native-on-windows_next_steps>`.

.. tip::

    Use batch files to simplify the running of SITL down to a single double-click. See `here <https://github.com/ArduPilot/ardupilot/tree/master/Tools/autotest/win_sitl>`__ for some examples.

FlightGear 3D View (Optional)
-----------------------------

`FlightGear Flight Simulator <http://www.flightgear.org/>`__ can also be used as a viewer
to display a 3D simulation of the vehicle and its surroundings. This
provides a much better visualization than the 2D maps and HUD flight
displays provided by *MAVProxy* and *Mission Planner*.

.. figure:: ../images/flightgear_copter_windows.jpg
   :target: ../_images/flightgear_copter_windows.jpg
   :width: 450px

   FlightGear:Simulated Copter at KSFO (click for larger view).

SITL outputs *FlightGear* compatible state information on UDP port 5503.
We highly recommend you start *FlightGear* before starting SITL
(although this is not a requirement, it has been found to improve
stability in some systems).

The main steps are:

#. Download `FlightGear 3.4.0 <http://www.flightgear.org/download/>`__

   .. warning::

      At time
         of writing `version 3.4.0 is required on Windows <https://github.com/ArduPilot/ardupilot/issues/3422>`__.

#. Open a new command prompt and run the appropriate batch file for your
   vehicle in **/ardupilot/Tools/autotest/**:
   `fg_plane_view.bat <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/fg_plane_view.bat>`__
   (Plane) and
   `fg_quad_view.bat <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/fg_quad_view.bat>`__
   (Copter).

   This will start FlightGear.
#. Start SITL in Cygwin in the normal way. In this case we're specifying
   the start location as San Francisco airport (KSFO) as this is an
   interesting airport with lots to see:

   ::

       sim_vehicle.py -L KSFO

   .. note::

      *FlightGear* will always initially start by loading scenery at
         KSFO (this is hard-coded into the batch file) but will switch to the
         scenery for the simulated location once SITL is started.

.. tip::

   If the vehicle appear to be hovering in space (no
      scenery) then *FlightGear* does not have any scenery files for the
      selected location. Choose a new location!

You can now takeoff and fly the vehicle as normal for
:ref:`Copter <copter-sitl-mavproxy-tutorial>` or
:ref:`Plane <plane-sitlmavproxy-tutorial>`, observing the vehicle movement
including pitch, yaw and roll.

.. _sitl-native-on-windows_running_sitl_andmavproxy:

Connecting with another GCS
===========================

You can attach multiple additional ground control stations to SITL from
*MAVProxy*. The simulated vehicle can then be controlled and viewed
through any attached GCS.

First use the ``output`` command on the *MAVProxy command prompt* to
determine where *MAVProxy* is sending packets:

::

    GUIDED> output
    GUIDED> 2 outputs
    0: 127.0.0.1:14550
    1: 127.0.0.1:14551

This tells us that we can connect *Mission Planner* to either UDP port
14550 or 14551, as shown on the dialog below.

.. figure:: ../images/MissionPlanner_Connect_UDP.jpg
   :target: ../_images/MissionPlanner_Connect_UDP.jpg

   Mission Planner: Connecting to a UDPPort

.. tip::

   We could connect *APM Planner 2* to the remaining port. If we
   needed a third port, we could add it as shown:

   ::

       GUIDED> output add 127.0.0.1:14553

*Mission Planner* can then be used to control the simulated vehicle in
exactly the same way as though it were a real vehicle. We can reproduce
the previous "takeoff-circle-land" example as shown below:

#. Change to GUIDED mode, arm the throttle, and then takeoff

   -  Open the *FLIGHT DATA* screen and select the *Actions* tab on the
      bottom left. This is where we can change the mode and set
      commands.

      .. figure:: ../images/MissionPlanner_Actions_GuidedModeArm.png
         :target: ../_images/MissionPlanner_Actions_GuidedModeArm.png

         Mission Planner: Actions Tab (Set Mode, Arm/Disarm)

   -  Select **Guided** in the *Mode selection list* and then press the
      **Set Mode** button.
   -  Select the **Arm/Disarm** button
   -  Right-click on the map and select Takeoff. Then enter the desired
      takeoff altitude

      .. figure:: ../images/MissionPlanner_Map_takeoff.jpg
         :target: ../_images/MissionPlanner_Map_takeoff.jpg

         Mission Planner Map: Takeoff Command

   .. note::

      Takeoff must start within 15 seconds of arming, or the motors
      will disarm.

#. Change to CIRCLE mode on the *Action* tab and watch the copter circle
   on the map.
#. You can change the circle radius in the *CONFIG/TUNING* screen.
   Select *Full Parameters List*, then the **Find** button and search
   for ``CIRCLE_MODE``. When you've changed the value press the **Write
   Params** button to save them to the vehicle.
#. When you're ready to land you can set the mode to RTL.

Running SITL with a GCS without MAVProxy
========================================

It is also possible to interact with SITL without using *MAVProxy* at
all using **ArduCopter.elf** (in the **ArduCopter** directory).

Run the file in the *Cygwin Terminal*, specifying a home position and
vehicle model as shown below:

::

    hamis_000@XPS12ultra ~/ardupilot/ArduCopter
    $ ./ArduCopter.elf --home -35,149,584,270 --model quad
    Started model quad at -35,149,584,270 at speed 1.0
    Starting sketch 'ArduCopter'
    Starting SITL input
    bind port 5760 for 0
    Serial port 0 on TCP port 5760
    Waiting for connection ....

The command output shows that you can connect to SITL using TCP/IP at
port 5760.

In *Mission Planner* we first change the link type to TCP and then press
the **Connect** button. Click through the \ *remote host* and *remote
Port* prompts as these default to the correct values.

.. figure:: ../images/MissionPlanner_ConnectTCP.jpg
   :target: ../_images/MissionPlanner_ConnectTCP.jpg

   Mission Planner: Connecting toSITL using TCP

Mission Planner will then connect and can be used just as before.

.. tip::

   **ArduCopter.elf** has other startup options, which you can use
   using the -h command line parameter:

   ::

       ./ArduCopter.elf -h

Updating ArduPilot
==================

See advice on :ref:`this wiki page <git-rebase>` regarding how to "Rebase" on ArduPilot's master branch.

Updating MAVProxy
=================

.. warning::

   The *MAVProxy 1.4.19 *\ installer does not properly remove all
   parts of preceding installations. Before installing a new version you
   must first delete the old directory: C\ **:\\Program Files
   (x86)\\MAVProxy\\**.

`Download and Install MAVProxy for Windows <https://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe>`__ (this link always points to the latest version!)


.. _sitl-native-on-windows_next_steps:

Next steps
==========

SITL and MAVProxy can do a whole lot more than shown here, including
manually guiding the vehicle, and creating and running missions. To find
out more:

-  Read the `MAVProxy documentation <http://ardupilot.github.io/MAVProxy/>`__.
-  See :ref:`Using SITL for ArduPilot Testing <using-sitl-for-ardupilot-testing>` for guidance on flying and testing with SITL.

Troubleshooting
---------------

Some users have reported build errors related to Windows not setting paths correctly. For more information see `this issue <https://github.com/ArduPilot/ardupilot-wiki-issue-tracker/issues/156>`__.
