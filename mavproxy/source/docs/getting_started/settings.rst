========
Settings
========

MAVProxy has a number of internal settings, which can be used to change
the behaviour of MAVProxy.

The settings take the following form in the MAVProxy environment:

.. code:: bash

    set <settingname> <value>
    
If a ``<value>`` is not specified, MAVProxy will output the current value 
of the setting.

The current settings in MAVProxy are:

==================   ==========================================  ===============================
Setting              Description                                 Value Range
==================   ==========================================  ===============================
link                 Primary Link number                         -1 to 4
streamrate           Data stream rate on link 1                  -1 to 100
streamrate2          Data stream rate on link 2                  -1 to 100
heartbeat            GCS to UAV hearbeat rate (Hz)               1 to 5
mavfwd               Allow control from --out clients            True or False
mavfwd_rate          Allow --out clients to control streamrate   True or False
shownoise            Show any malformed or bad data              True or False
baudrate             Baudrate for any --out serial links         0 to 10000000
rtscts               Enable flow control for serial links        True or False
select_timeout       Timeout (sec) when trying to read packets   <any float>
altreadout           Readout altitude                            0 to 100
distreadout          Distance readout                            0 to 10000
moddebug             Debug verbosity                             0 to 3
compdebug            Computation debug mask                      0 to 4
flushlogs            Flush (write) logfile on every packet       True or False
requireexit          Require ``exit`` command to exit MAVProxy   True or False
wpupdates            Announce waypoint updates                   True or False
basealt              Base altitude (AGL offset)                  0 to 30000
wpalt                Default waypoint altitude (m)               0 to 10000
rallyalt             Default rally point altitude (m)            0 to 10000
terrainalt           Use terrain altitude in waypoints           True, False, Auto
rally_breakalt       Default rally point break altitude (m)      0 to 10000
rally_flags          Default rally flags                         0 to 10000
source_system        MAVLink Source system                       0 to 255
source_component     MAVLink Source component                    0 to 255
target_system        MAVLink Target system                       0 to 255
target_component     MAVLink Target component                    0 to 255
state_basedir        Base directory for logfiles                 <string>
allow_unsigned       Accept unsigned packets                     True or False
dist_unit            Distance unit                               m, nm, miles
height_unit          Height unit                                 m, feet
speed_unit           Speed unit                                  m/s, knots, mph
vehicle_name         Vehicle name in MAVProxy prompt             <string>
==================   ==========================================  ===============================

