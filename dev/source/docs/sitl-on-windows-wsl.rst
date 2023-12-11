.. _sitl-on-windows-wsl:

=========================
SITL on Windows using WSL
=========================

Before following these instructions please following :ref:`Setting up the Build Environment using WSL <building-setup-windows>` for Windows 10 and 11.  For Windows10 users the sections detailing :ref:`installing XWindows or VcXsrv <building-setup-windows10_XWindows>` are necessary for installation.

Once the above have been completed, using SITL under Windows WSL is nearly the same as :ref:`Setting up SITL on Linux <setting-up-sitl-on-linux>`

.. image:: ../images/MAVProxy_Map_GuidedCopter.jpg
   :target: ../_images/MAVProxy_Map_GuidedCopter.jpg
   :width: 450px

Running SITL
============

#. Open an Ubuntu terminal under WSL, or use the :ref:`VSCode <editing-the-code-with-vscode>` Terminal

#. Navigate to one of the vehicle directories (in this case Copter) and call ``sim_vehicle.py`` to start SITL.

   ::

       cd ~/ardupilot/ArduCopter
       ../Tools/autotest/sim_vehicle.py --map --console

   .. image:: ../images/sitl-on-windows-wsl-vscode.png
      :target: ../_images/sitl-on-windows-wsl-vscode.png
      :width: 450px

#. SITL and *MAVProxy* will start. *MAVProxy* displays three windows:

   -  command prompt in which you enter commands to SITL
   -  console which displays current status and messages
   -  map that shows the current position of the vehicle and can be
      used (via right-click) to control vehicle movement and missions.

   .. image:: ../images/mavproxy_sitl_console_and_map.jpg
       :target: ../_images/mavproxy_sitl_console_and_map.jpg
       :width: 450px

   .. tip::

      It is useful to arrange the windows as shown above, so you can
      observe the status and send commands at the same time.

#. Send commands to SITL from the command prompt and observe the results on the map.

   -  For example, change to GUIDED mode, arm the throttle, and then takeoff:

      ::

          mode guided 
          arm throttle
          takeoff 40

      Watch the altitude increase on the console.

      .. note::

         Takeoff must start within 15 seconds of arming, or the motors will disarm.

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
can do with SITL and MAVProxy see the section: :ref:`Next Steps <sitl-on-windows-wsl_next_steps>`.

Connecting with another GCS
===========================

Mission Planner will often automatically connect to SITL if started (or re-started) after the MAVProxy console and map have appeared.

If the above does not work you can attach multiple additional ground control stations to SITL from
*MAVProxy*. The simulated vehicle can then be controlled and viewed through any attached GCS.

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

*Mission Planner* can then be used to control the simulated vehicle in
exactly the same way as though it were a real vehicle. We can reproduce
the previous "takeoff-circle-land" example as shown below:

.. _sitl-on-windows-wsl_next_steps:

Next Steps
==========

SITL and MAVProxy can do a whole lot more than shown here, including
manually guiding the vehicle, and creating and running missions. To find
out more:

-  Read the :ref:`MAVProxy documentation <mavproxy:home>`.
-  See :ref:`Using SITL for ArduPilot Testing <using-sitl-for-ardupilot-testing>` for guidance on flying and testing with SITL.
