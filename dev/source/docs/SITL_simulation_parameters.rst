.. _SITL_simulation_parameters:

Using Simulation Parameters to Control the Simulation
=====================================================

SITL has a large set of parameters to control the simulation. Wind speed/direction, Sensor malfunctions and parameters, RC failures, etc. can be emulated and controlled. They are visible and present when a simulation is running and prefixed by ``SIM_``. A few are detailed below.

Setting parameters
------------------

.. note:: If the attached GCS is Mission Planner, you would use the CONFIG/Full Parameter List or Tree to change the parameters. The following gives examples using MAVProxy commands to change the parameter values.

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


Changing the speed of the simulation
------------------------------------

Most of the simulator backends (native, X-plane, etc.) support changing the speed while
running (NOT Realflight). Just set the ``SIM_SPEEDUP`` parameter as needed. A value of 1
means normal wall-clock time. A value of 5 means 5x realtime. A value
of 0.1 means 1/10th of real time.

Running with high-levels of speedup can result in enough network traffic that MAVProxy can’t keep up. An error such as “Set RC override timeout” or the vehicle entering GCS failsafe are typical of these failures. Reducing the ``SIM_SPEEDUP`` factor is typically sufficient to avoid this problem. Also, using the ``--udp`` options will use UDP instead of the slower TCP communication protocol and can help avoid this issue at higher speedups.

Loading a parameter set
-----------------------

When starting SITL the first time, the device will be configured with
default parameters for the vehicle/frame type. Typically, you will want to replace these with
values that simulate the desired vehicle and environment conditions.

::

    param load <path to parameter file>

the working directory is the directory in which MAVProxy starts, not the simulation

You can re-load the parameters later if you choose, or revert to the
default parameters by starting SITL (**sim_vehicle.py**) with the
``-w`` flag.

.. note:: You may have to load them again, after typing ``param fetch``, in order to load parameters that require enabling before presenting their parameter set.  And, in some cases, you may even need to restart SITL in order for some new parameters, such as output function changes, to take effect. This can be avoided if you add the parameter file during the start of SITL with the "--add-param-file=*pathtofile* "....ie: sim_vehicle.py -f flightaxis:192.168.x.x - -map - -console --add-param-file=*pathtofile* -w. This adds the param file as a default and then wipes any previous param changes that may exist back to the defaults. 

Parameters can also be saved. For example, to save the parameters in a file in the present working directory:

::

    param save ./myparams.parm

.. note:: the "default" parameter set emulates the parameter default values contained in the firmware. These are used at startup for parameters, UNLESS the user has previously changed a parameter(s) which are stored in the autopilot's flash, in which case those changed values are used for those changed parameters. In SITL, these changes are stored in a file in the simulation startup directory named ``eeprom.bin``. The above param load and save commands deal with these user set values for the parameters, not the firmware default values created at simulation start, which can be changed using the ``--add-param-file`` startup option.

Other options can be listed using:

::

   param help


Testing RC failsafe
-------------------

To test the behaviour of ArduPilot when you lose remote control (RC),
set the parameter ``SIM_RC_FAIL=1``, as shown:

::

    param set SIM_RC_FAIL 1

This simulates the complete loss of RC input. If you just want to
simulate low throttle (below throttle failsafe level) then you can do
that with the RC command:

::

    rc 3 900

or use ``SIM_RC_FAIL=2`` which sets all channels at neutral  (1500us) and the throttle channel at 950us.

Testing the effects of vibration
--------------------------------

To test the vehicle's reaction to vibration, use ``SIM_ACC_RND``. The
example below adds 3 m/s/s acceleration noise:

::

    param set SIM_ACC_RND 3

Testing the effects of wind
---------------------------

The wind direction, speed and turbulence can be changed to test their
effect on flight behaviour. The following settings changes the wind so
that it blows from the South at a speed of 10 m/s.

::

    param set SIM_WIND_DIR 180
    param set SIM_WIND_SPD 10

To see other wind parameters do:

::

    param show sim_wind*

Testing GPS failure
-------------------

To test losing GPS lock, use ``SIM_GPS_DISABLE``:

::

    param set SIM_GPS_DISABLE 1

You can also enable/disable a 2nd GPS using ``SIM_GPS2_DISABLE``.

Testing GPS-for-Yaw
-------------------

To test GPS-for-Yaw using two simulated UBlox GPSs:

::

    param set EK3_SRC1_YAW 2
    param set GPS_AUTO_CONFIG 0
    param set GPS_TYPE 17
    param set GPS_TYPE2 18
    param set GPS_POS1_Y -0.2
    param set GPS_POS2_Y 0.2
    param set SIM_GPS_POS_Y -0.2
    param set SIM_GPS2_POS_Y 0.2
    param set SIM_GPS2_DISABLE 0
    param set SIM_GPS2_HDG 1

Reboot SITL and the heading should be visible in the GPS2_RAW message's yaw field:

::

    status GPS2_RAW

To test using a single NMEA GPS:

::

    param set EK3_SRC1_YAW 2
    param set GPS_TYPE 5
    param set SIM_GPS_TYPE 5
    param set SIM_GPS_HDG 1

Reboot SITL and the heading should be visible in the GPS_RAW_INT message's yaw field:

::

    status GPS_RAW_INT

Testing Compass Calibration
---------------------------

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
