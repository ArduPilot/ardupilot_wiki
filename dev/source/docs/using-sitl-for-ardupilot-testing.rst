.. _using-sitl-for-ardupilot-testing:

==========
Using SITL
==========

This article describes how :ref:`SITL <sitl-simulator-software-in-the-loop>`
and :ref:`MAVProxy <mavproxy-developer-gcs>` can be used to change the environment,
simulate failure modes, and configure the vehicle with optional components.
It also explains how to :ref:`connect to different GCSs <using-sitl-for-ardupilot-testing_connecting_otheradditional_ground_stations>`.

.. tip::

   If you're just getting started with MAVProxy and SITL you may wish
   to start by reading the :ref:`Copter SITL/MAVProxy Tutorial <copter-sitl-mavproxy-tutorial>`
   (or equivalent tutorials for the other vehicles).

.. note::

   These instructions generally use
   :ref:`MAVProxy <mavproxy-developer-gcs>` to
   describe operations (e.g. setting parameters) because it presents a
   simple and consistent command-line interface (removing the need to
   describe a GCS-specific UI layout). Many of these operations can also
   be performed in *Mission Planner* (through the *Full Parameters List*) or any other GCS.

..  youtube:: Ewh0fKGEJL4
    :width: 100%



Selecting a vehicle/frame type
==============================

The simulation will default to the vehicle type in the directory in which it is started. You can select the vehicle type if starting from the ardupilot directory by starting the simulator calling **sim_vehicle.py** with the ``-v`` parameter.

::

     sim_vehicle.py -v ArduPlane --console --map

The frame type can also be changed with the ``-f`` parameter.

::

    sim_vehicle.py -v ArduPlane -f quadplane --console --map
    

Frame Types:
------------

A partial listing of frame types is show below. For a current list, just type:

::

    sim_vehicle.py --help


+--------------------------+------------------------+
+          Vehicle         +       Frame Type       +
+--------------------------+------------------------+
+                          + plane (default if -f   +
+                          + is not used)           +
+                          +------------------------+
+ Plane                    + quadplane              +
+                          + firefly plane-dspoilers+
+                          + plane-elevon plane-jet +
+                          + plane-tailsitter plane-+
+                          + vtail quadplane-cl84   +
+                          + quadplane-tilthvec     +
+                          + quadplane-tilttri      +
+                          + quadplane-tilttrivec   +
+                          + quadplane-tri          +
+--------------------------+------------------------+
+                          + quad (default if -f    +
+                          + is not used)           +
+                          +------------------------+
+                          + coaxcopter  dodeca-hexa+
+ Copter                   + heli heli-compound     +
+                          + heli-dual hexa hexa-cwx+
+                          + hexa-dji octa octa-cwx +
+                          + octa-dji octa-quad     +
+                          + octaquad-cwx  tri  cwx +
+                          + singlecopter  y6 djix  +
+--------------------------+------------------------+
+                          + rover (default if -f   +
+  Rover                   + is not used)           +
+                          +------------------------+
+                          + balancebot rover-skid  +
+                          + sailboat sailboat-motor+
+--------------------------+------------------------+


Setting vehicle start location
==============================

You can start the simulator with the vehicle at a particular location by
calling **sim_vehicle.py** with the ``-L`` parameter and a named
location in the
`ardupilot/Tools/autotest/locations.txt <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/locations.txt>`__
file.

For example, to start Copter in *Ballarat* (a named location in
**locations.txt**) call:

::

    cd ArduCopter 
    sim_vehicle.py -L Ballarat --console --map

.. note::

   You can add your own locations to the file. The order is Lat,Lng,Alt,Heading where alt is MSL and in meters, and heading is degrees.
   If the flying location is well-used then consider adding it to the project via a pull request.

.. note::

   You can add your own private locations to a local locations.txt
   file, in the same format as the main file.  On linux the file is
   located in ``$HOME/.config/ardupilot/locations.txt`` - you will
   need to create this file using your favourite text editor.

Simulating On-Board OSD
=======================

When starting SITL, you can have it display a simulation of the integated OSD, if the autopilot includes one. Add the ``--osd`` option when starting SITL:

::

    sim_vehicle.py -v ArduPlane --console --map --osd


.. _using-sitl-for-ardupilot-testing_loading_a_parameter_set:

Loading a parameter set
=======================

When starting SITL the first time, the device may be configured with
"unforgiving" parameters. Typically you will want to replace these with
values that simulate more realistic vehicle and environment conditions.
Useful parameter sets are provided in the autotest source for
`Copter <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/default_params/copter.parm>`__,
`Plane <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/default_params/plane.parm>`__,
and
`Rover <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/default_params/rover.parm>`__.

.. tip::

   This only needs to be done once, after loading, the parameters are
   stored in the simulated EEPROM.

The MAVProxy commands to load the parameters for Copter, Rover and Plane
(assuming the present working directory is a vehicle directory like
**/ardupilot/ArduCopter/**) are shown below:

::

    param load ..\Tools\autotest\default_params\copter.parm

::

    param load ..\Tools\autotest\default_params\plane.parm

::

    param load ..\Tools\autotest\default_params\rover.parm

You can re-load the parameters later if you choose, or revert to the
default parameters by starting SITL (**sim_vehicle.py**) with the
``-w`` flag.

Parameters can also be saved. For example, to save the parameters into
the present working directory you might do:

::

    param save ./myparams.parm

Setting parameters
==================

Many of the following tasks involve setting parameter values over
MAVLink, which you do using the ``param set`` command as shown:

::

    param set PARAMETERNAME VALUE

All available parameters can be listed using ``param show``. The
SITL-specific parameters start with ``SIM_``, and can be obtained using:

::

    param show SIM_*

.. tip::

   A table of user settable SITL-specific parameters can be found in `SITL.cpp <https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SITL.cpp>`__
   When you change a parameter the value remains in the virtual EEPROM
   after you restart SITL. Remember to change it back if you don't want it
   any more (or :ref:`reload/reset the parameters <using-sitl-for-ardupilot-testing_loading_a_parameter_set>`). 

Testing RC failsafe
===================

To test the behaviour of ArduPilot when you lose remote control (RC),
set the parameter ``SIM_RC_FAIL=1``, as shown:

::

    param set SIM_RC_FAIL 1

This simulates the complete loss of RC input. If you just want to
simulate low throttle (below throttle failsafe level) then you can do
that with the RC command:

::

    rc 3 900

Testing GPS failure
===================

To test losing GPS lock, use ``SIM_GPS_DISABLE``:

::

    param set SIM_GPS_DISABLE 1

You can also enable/disable a 2nd GPS using ``SIM_GPS2_ENABLE``.

Testing the effects of vibration
================================

To test the vehicle's reaction to vibration, use ``SIM_ACC_RND``. The
example below adds 3 m/s/s acceleration noise:

::

    param set SIM_ACC_RND 3

Testing the effects of wind
===========================

The wind direction, speed and turbulence can be changed to test their
effect on flight behaviour. The following settings changes the wind so
that it blows towards the South at a speed of 10 m/s.

::

    param set SIM_WIND_DIR 180
    param set SIM_WIND_SPD 10

To see other wind parameters do:

::

    param show sim_wind*


Adding a virtual gimbal
=======================

SITL can simulate a virtual gimbal.

.. note::

   Gimbal simulation causes SITL to start sending
   `MOUNT_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_STATUS>`__
   messages. These messages contain the orientation according to the last
   commands sent to the gimbal, not actual measured values. As a result, it
   is possible that the true gimbal position will not match - i.e. a
   command might be ignored or the gimbal might be moved manually. Changes
   are not visible in Mission Planner.

First start the simulator and use the following commands to set up the
gimbal mount:

::

    # Specify a servo-based mount:
    param set MNT_TYPE 1

    # Set RC output 6 as pan servo:
    param set RC6_FUNCTION 6

    # Set RC output 8 as roll servo:
    param set RC7_FUNCTION 8

Then stop and re-launch SITL with the ``-M`` flag:

::

    sim_vehicle.py -M

Adding a virtual rangefinder
============================

SITL can simulate an analog rangefinder, which is very useful for
developing flight modes that can use a rangefinder. To set it up use the
following commands:

::

    param set SIM_SONAR_SCALE 10
    param set RNGFND_TYPE 1
    param set RNGFND_SCALING 10
    param set RNGFND_PIN 0
    param set RNGFND_MAX_CM 5000
    param set RNGFND_MIN_CM 0

    # Enable rangefinder for landing (Plane only!)
    param set RNGFND_LANDING 1

The above commands will set up an analog rangefinder with a maximum range
of 50 meters (the 50m comes from an analog voltage range of 0 to 5V, and
a scaling of 10). After making the above changes you need to restart
SITL.

Then to test it try this:

::

    module load graph
    graph RANGEFINDER.distance

Then try a flight and see if the graph shows you the rangefinder
distance.

.. tip::

   You can also use the following commands to graph rangefinder
   information (defined as *MAVProxy* aliases):

   -  ``grangealt`` - graph rangefinder distance and relative altitude.
   -  ``grangev`` - rangefinder voltage
   -  ``grange`` - graph "rangefinder_roll"

Adding a virtual optical flow sensor
====================================

You can add a virtual optical flow sensor like this:

::

    param set SIM_FLOW_ENABLE 1
    param set FLOW_TYPE 10

Then restart SITL. After setting it up try this:

::

    module load graph
    graph OPTICAL_FLOW.flow_x OPTICAL_FLOW.flow_y

Go for a flight and see if you get reasonable data.

Accessing log files
===================

SITL supports both blogs and DF logs (same as other types of ArduPilot
ports). The DF logs are stored in a "logs" subdirectory in the directory
where you start SITL. You can also access the DF logs via MAVLink using
a GCS, but directly accessing them in the logs/ directory is usually
more convenient.

To keep your blogs organised it is recommended you start SITL using the
"--aircraft NAME" option. That will create a subdirectory called NAME
which will have flight logs organised by date. Each flight will get its
own directory, and will include the parameters for the flight plus any
downloaded waypoints and rally points.

Graphing vehicle state
======================

MAVProxy allows you to create graphs of inputs, outputs, internal variables, etc. by loading the ``graph`` module.  You can then create graphs of vehicle state using the graph command. For example, to graph the RC channel 3 input during the simulation:

::

    graph RC_CHANNELS.channel3_raw

As with most commands, you can type ``graph`` and then double tab to see available completions. In the example above the `RC_CHANNELS` group contains the 16 rc channels, so typing ``graph RC_CHANNELS``, then double tab would show those.

Since these are rather long to type, MAVProxy allows for the creation of shorter aliases.  There have been many aliases created in a file present in the ArduPilot source tree in the Tools/vagrant sub-directory called **mavinit.scr**. If this file is copied and placed in your home directory and renamed to **.mavinit.scr**, it will be used upon *MAVProxy's* initialization.  Common ones are: *g* for graph, *grc* to graph the RC inputs, *gservo8* to graph the first 8 output channels instead of typing long item names eight times, etc. Using this alias initialization file also has the advantage of automatically loading the graph module upon startup, so you will not have to load it.


Using a joystick
================

It can be useful to use a joystick for input in SITL. The joystick can
be a real RC transmitter with a USB dongle for the trainer port, or
something like the RealFlight interlink controller or a wide range of
other joystick devices.

Before you use the joystick support you may need to remove debug
statements from the python-pygame joystick driver on Linux. If you don't
then you may see lots of debug output like this:

::

    SDL_JoystickGetAxis value:-32768:

To remove this debug line run this command:

::

    sudo sed -i 's/SDL_JoystickGetAxis value/\x00DL_JoystickGetAxis value/g' /usr/lib/python2.7/dist-packages/pygame/joystick.so

note that this needs to be one long command line. Ignore the line
wrapping in the wiki. If you have installed the joystick support using the instructions on setting up the ArduPilot code building environment, then this will probably NOT be required.

Then to use the joystick run:

::

    module load joystick

If you want to add support for a new joystick type then you need to add a file for it following these `instructions <https://github.com/ArduPilot/MAVProxy/blob/master/docs/JOYSTICKS.md>`__ . Note that you can also use this information to customize the operation of your joystick if it already supported. Just modify it file appropriately.

Using real serial devices
=========================

Sometimes it is useful to use a real serial device in SITL. This makes
it possible to connect SITL to a real GPS for GPS device driver
development, or connect it to a real OSD device for testing an OSD.

To use a real serial device you can use a command like this:

::

    sim_vehicle.py -A "--uartB=uart:/dev/ttyUSB0" --console --map

what that does it pass the --uartB argument to the ardupilot code,
telling it to use /dev/ttyUSB0 instead of the normal internal simulated
GPS for the 2nd UART.

Any of the 5 UARTs can be configured in this way, using uartA to uartE.

Similar to this if you were running a vehicle in SITL via Cygwin on
Microsoft Windows and you wanted to send the mavlink output through a
connected radio on COM16 to AntennaTracker you can use a command like
this - note under Cygwin comm ports are ttyS and they start at 0 so 15
is equivalent to COM16:

::

    sim_vehicle.py -A "--uartC=uart:/dev/ttyS15" --console --map

.. _using-sitl-for-ardupilot-testing_connecting_otheradditional_ground_stations:


Changing the speed of the simulation
====================================

Most of the simulator backends support changing the speed while
running. Just set the SIM_SPEEDUP parameter as needed. A value of 1
means normal wall-clock time. A value of 5 means 5x realtime. A value
of 0.1 means 1/10th of real time.

Testing Compass Calibration
===========================

A quick way to test compass calibration in SITL is with the
"calibration" vehicle model. To use this with plane do this:

::

   sim_vehicle.py -D -f plane --model calibration --console --map

then do:

:: 

   servo set 5 1250

This will start the vehicle moving through a "compass dance". You can
start a compass calibration to test changes to the calibrator
code. Using this in combination with the SIM_SPEEDUP parameter can be useful.

The calibration vehicle module has a lot of other features too. See
`http://guludo.me/posts/2016/05/27/compass-calibration-progress-with-geodesic-sections-in-ardupilot/
<http://guludo.me/posts/2016/05/27/compass-calibration-progress-with-geodesic-sections-in-ardupilot/>`__
for details.

Connecting other/additional ground stations
===========================================

SITL can connect to multiple ground stations by using *MAVProxy* to
forward UDP packets to the GCSs network address. Alternatively SITL can
connect to a GCS over TCP/IP without using *MAVProxy*.

.. _using-sitl-for-ardupilot-testing_sitl_with_mavproxy_udp:

SITL with MAVProxy (UDP)
------------------------

SITL can connect to multiple ground stations by using *MAVProxy* to
forward UDP packets to the GCSs network address (for example, forwarding
to another Windows box or Android tablet on your local network). The
simulated vehicle can then be controlled and viewed through any attached
GCS.

First find the IP address of the machine running the GCS. How you get
the address is platform dependent (on Windows you can use the 'ipconfig'
command to find the computer's address).

Assuming the IP address of the GCS is 192.168.14.82, you would add this
address/port as a *MAVProxy* output using:

::

    output add 192.168.14.82:14550

The GCS would then connect to SITL by listening on that UDP port. The
method for connecting will be GCS specific (we show :ref:`how to connect for Mission Planner <using-sitl-for-ardupilot-testing_connecting_mission_planner_udp>` below).

.. tip::

   If you're running the GCS on the **same machine** as SITL then an
   appropriate output may already exist. Check this by calling ``output``
   on the *MAVProxy command prompt*:

   ::

       GUIDED> output
       GUIDED> 2 outputs
       0: 127.0.0.1:14550
       1: 127.0.0.1:14551

   In this case we can connect a GCS running on the same machine to UDP
   port 14550 or 14551. We can choose to connect another GCS to the
   remaining port, and add more ports if needed. 

   
.. _using-sitl-for-ardupilot-testing_sitl_without_mavproxy_tcp:

SITL without MAVProxy (TCP)
---------------------------

It is also possible to interact with SITL over TCP/IP by starting it
using \ *vehicle_name*.\ **elf** (e.g. **/ArduCopter/ArduCopter.elf**).
*MAVProxy* is not needed when using this method.

Run the file in the *Cygwin Terminal*, specifying a home position and
vehicle model as shown below:

::

    $ ./ArduCopter.elf --home -35,149,584,270 --model quad
    Started model quad at -35,149,584,270 at speed 1.0
    Starting sketch 'ArduCopter'
    Starting SITL input
    bind port 5760 for 0
    Serial port 0 on TCP port 5760
    Waiting for connection ....

The command output shows that you can connect to SITL using TCP/IP at
the network address of the **machine SITL is running on** at port 5760.

.. tip::

   **ArduCopter.elf** has other startup options, which you can use
   using the -h command line parameter:

   ::

       ./ArduCopter.elf -h

.. _using-sitl-for-ardupilot-testing_connecting_mission_planner_udp:

Connecting Mission Planner (UDP)
--------------------------------

First set up SITL to :ref:`output UDP packets to the address/port of the computer running *Mission Planner* <using-sitl-for-ardupilot-testing_sitl_with_mavproxy_udp>`.

In *Mission Planner* listen to the specific UDP port by selecting
**UDP** and then the **Connect** button. Enter the port to listen on
(the default port number of 14550 should be correct if SITL is running
on the same computer).

.. figure:: ../images/MissionPlanner_Connect_UDP.jpg
   :target: ../_images/MissionPlanner_Connect_UDP.jpg

   Mission Planner: Connecting to a UDPPort

Connecting to Mission Planner (TCP)
-----------------------------------

First set up SITL :ref:`for use with TCP <using-sitl-for-ardupilot-testing_sitl_without_mavproxy_tcp>`.

In *Mission Planner* connect to SITL by selecting **TCP** and then the
**Connect** button. Enter the \ *remote host* and *remote Port* of the
machine running SITL. *Mission Planner* will then connect and can be
used just as before.

.. tip::

   If SITL is running on the same machine as *Mission Planner* you can
   click through the \ *remote host* and *remote Port* prompts as these
   default to the correct values.

.. figure:: ../images/MissionPlanner_ConnectTCP.jpg
   :target: ../_images/MissionPlanner_ConnectTCP.jpg

   Mission Planner: Connecting toSITL using TCP


Testing Precision Landing
-------------------------

.. note::

   These instructions are written assuming ArduCopter

Enable Precision Landing, and set the precision landing backend type to SITL:

::

   param set PLND_ENABLED 1
   param fetch
   param set PLND_TYPE 4

A rangefinder is currently required for precision landing.  Enable a simulated rangefinder:

::

   param set RNGFND_TYPE 1
   param set RNGFND_MIN_CM 0
   param set RNGFND_MAX_CM 4000
   param set RNGFND_PIN 0
   param set RNGFND_SCALING 12.12

Restart the simulation.
   
Takeoff and fly a bit, then switch into land:

::

   arm throttle
   rc 3 1800
   mode land
   rc 3 1500

Check the logs for precision landing messages:

::

   ls -lt logs

Choose the youngest, then:

::

   mavlogdump --type PL logs/<youngest>



Testing Visual Positioning
--------------------------

Start SITL, wiping parameters:

::

   ./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug -w

Disable GPS, indicate to ArduPilot that instead of a GPS on SERIAL3 it should expect MAVLink (e.g. simulating a 900MHz radio):

::

   param set GPS_TYPE 0
   param set EK2_GPS_TYPE 3
   param set SERIAL3_PROTOCOL 1
   param set DISARM_DELAY 60

Restart the simulation, attaching a simulated VICON system to uartB (which corresponds to ``SERIAL3``:

::

   ./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug -A "--uartB=sim:vicon:" --map --console

The console should indicate no GPS is present:

::

   GPS: 0 (0)

Vision position estimates should now be being fed into ArduCopter:

::

   STABILIZE> status VICON_POSITION_ESTIMATE
   STABILIZE> 43371: VICON_POSITION_ESTIMATE {usec : 38380000, x : 0.0, y : 0.0, z : -0.0999755859375, roll : 0.0, pitch : 0.0, yaw : -0.122173137963}


You should also receive a startup message from the EKF:

::

   APM: EKF2 IMU0 is using external nav data
   APM: EKF2 IMU0 initial pos NED = 0.0,0.0,-0.1 (m)
   APM: EKF2 IMU1 is using external nav data
   APM: EKF2 IMU1 initial pos NED = 0.0,0.0,-0.1 (m)

Use MAVProxy's right-click context menu item to ``Set Origin (with alt)``

Use MAVProxy's right-click context menu item to ``Set Home (with alt)``

Arm in stabilize, switch to loiter:

::

   mode stabilize
   arm throttle
   mode loiter

Take off, then fly somewhere:

::

   rc 3 1800
   rc 2 1400


Wait a while, note vehicle moving on map.

Now RTL:

::

   rc 3 1500
   rc 2 1500
   mode rtl

Note vehicle returning to home
