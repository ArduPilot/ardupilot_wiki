.. Dynamically generated list of documented parameters
.. This page was generated using Tools\/autotest\/param\_metadata\/param\_parse\.py

.. DO NOT EDIT


.. _parameters:

Complete Parameter List
=======================

This is a complete list of the parameters which can be set \(e\.g\. via the MAVLink protocol\) to control vehicle behaviour\. They are stored in persistent storage on the vehicle\.

This list is automatically generated from the latest ardupilot source code\, and so may contain parameters which are not yet in the stable released versions of the code\.




.. _parameters_ArduPlane:

ArduPlane Parameters
--------------------


.. _FORMAT_VERSION:

FORMAT\_VERSION: Eeprom format version number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This value is incremented when changes are made to the eeprom format


.. _SYSID_SW_TYPE:

SYSID\_SW\_TYPE: Software Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is used by the ground station to recognise the software type \(eg ArduPlane vs ArduCopter\)


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _SYSID_THISMAV:

SYSID\_THISMAV: MAVLink system ID of this vehicle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network


+---------+
| Range   |
+=========+
| 1 - 255 |
+---------+




.. _SYSID_MYGCS:

SYSID\_MYGCS: Ground station MAVLink system ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The identifier of the ground station in the MAVLink protocol\. Don\'t change this unless you also modify the ground station to match\.


+---------+
| Range   |
+=========+
| 1 - 255 |
+---------+




.. _CLI_ENABLED:

CLI\_ENABLED: CLI Enable
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables\/disables the checking for three carriage returns on telemetry links on startup to enter the diagnostics command line interface


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _AUTOTUNE_LEVEL:

AUTOTUNE\_LEVEL: Autotune level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Level of aggressiveness for autotune\. When autotune is run a lower AUTOTUNE\_LEVEL will result in a \'softer\' tune\, with less aggressive gains\. For most users a level of 6 is recommended\.


+--------+-----------+
| Range  | Increment |
+========+===========+
| 1 - 10 | 1         |
+--------+-----------+




.. _TELEM_DELAY:

TELEM\_DELAY: Telemetry startup delay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The amount of time \(in seconds\) to delay radio telemetry to prevent an Xbee bricking on power up


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 10 | 1         | seconds |
+--------+-----------+---------+




.. _GCS_PID_MASK:

GCS\_PID\_MASK: GCS PID tuning mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

bitmask of PIDs to send MAVLink PID\_TUNING messages for


+-------------------+---------------------+
| Bitmask           | Values              |
+===================+=====================+
| +-----+---------+ | +-------+---------+ |
| | Bit | Meaning | | | Value | Meaning | |
| +=====+=========+ | +=======+=========+ |
| | 0   | Roll    | | | 0     | None    | |
| +-----+---------+ | +-------+---------+ |
| | 1   | Pitch   | | | 1     | Roll    | |
| +-----+---------+ | +-------+---------+ |
| | 2   | Yaw     | | | 2     | Pitch   | |
| +-----+---------+ | +-------+---------+ |
|                   | | 4     | Yaw     | |
|                   | +-------+---------+ |
|                   |                     |
+-------------------+---------------------+




.. _KFF_RDDRMIX:

KFF\_RDDRMIX: Rudder Mix
~~~~~~~~~~~~~~~~~~~~~~~~


The amount of rudder mix to apply during aileron movement 0 \= 0 \%\, 1 \= 100\%


+-------+-----------+
| Range | Increment |
+=======+===========+
| 0 - 1 | 0.01      |
+-------+-----------+




.. _KFF_THR2PTCH:

KFF\_THR2PTCH: Throttle to Pitch Mix
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Throttle to pitch feed\-forward gain\.


+-------+-----------+
| Range | Increment |
+=======+===========+
| 0 - 5 | 0.01      |
+-------+-----------+




.. _STAB_PITCH_DOWN:

STAB\_PITCH\_DOWN: Low throttle pitch down trim
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the amount of down pitch to add in FBWA and AUTOTUNE modes when at low throttle\. No down trim is added when throttle is above TRIM\_THROTTLE\. Below TRIM\_THROTTLE downtrim is added in proportion to the amount the throttle is below TRIM\_THROTTLE\. At zero throttle the full downpitch specified in this parameter is added\. This parameter is meant to help keep airspeed up when flying in FBWA mode with low throttle\, such as when on a landing approach\, without relying on an airspeed sensor\. A value of 2 degrees is good for many planes\, although a higher value may be needed for high drag aircraft\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 15 | 0.1       | Degrees |
+--------+-----------+---------+




.. _GLIDE_SLOPE_MIN:

GLIDE\_SLOPE\_MIN: Glide slope minimum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the minimum altitude change for a waypoint before a glide slope will be used instead of an immediate altitude change\. The default value is 15 meters\, which helps to smooth out waypoint missions where small altitude changes happen near waypoints\. If you don\'t want glide slopes to be used in missions then you can set this to zero\, which will disable glide slope calculations\. Otherwise you can set it to a minimum number of meters of altitude error to the destination waypoint before a glide slope will be used to change altitude\.


+----------+-----------+--------+
| Range    | Increment | Units  |
+==========+===========+========+
| 0 - 1000 | 1         | meters |
+----------+-----------+--------+




.. _GLIDE_SLOPE_THR:

GLIDE\_SLOPE\_THR: Glide slope threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the height above the glide slope the plane may be before rebuilding a glide slope\. This is useful for smoothing out an autotakeoff


+---------+-----------+--------+
| Range   | Increment | Units  |
+=========+===========+========+
| 0 - 100 | 1         | meters |
+---------+-----------+--------+




.. _STICK_MIXING:

STICK\_MIXING: Stick Mixing
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

When enabled\, this adds user stick input to the control surfaces in auto modes\, allowing the user to have some degree of flight control without changing modes\.  There are two types of stick mixing available\. If you set STICK\_MIXING to 1 then it will use \"fly by wire\" mixing\, which controls the roll and pitch in the same way that the FBWA mode does\. This is the safest option if you usually fly ArduPlane in FBWA or FBWB mode\. If you set STICK\_MIXING to 2 then it will enable direct mixing mode\, which is what the STABILIZE mode uses\. That will allow for much more extreme maneuvers while in AUTO mode\.


+--------------------------+
| Values                   |
+==========================+
| +-------+--------------+ |
| | Value | Meaning      | |
| +=======+==============+ |
| | 0     | Disabled     | |
| +-------+--------------+ |
| | 1     | FBWMixing    | |
| +-------+--------------+ |
| | 2     | DirectMixing | |
| +-------+--------------+ |
|                          |
+--------------------------+




.. _AUTO_FBW_STEER:

AUTO\_FBW\_STEER: Use FBWA steering in AUTO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

When enabled this option gives FBWA navigation and steering in AUTO mode\. This can be used to allow manual stabilised piloting with waypoint logic for triggering payloads\. With this enabled the pilot has the same control over the plane as in FBWA mode\, and the normal AUTO navigation is completely disabled\. THIS OPTION IS NOT RECOMMENDED FOR NORMAL USE\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 42    | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _TKOFF_THR_MINSPD:

TKOFF\_THR\_MINSPD: Takeoff throttle min speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum GPS ground speed in m\/s used by the speed check that un\-suppresses throttle in auto\-takeoff\. This can be be used for catapult launches where you want the motor to engage only after the plane leaves the catapult\, but it is preferable to use the TKOFF\_THR\_MINACC and TKOFF\_THR\_DELAY parameters for catapult launches due to the errors associated with GPS measurements\. For hand launches with a pusher prop it is strongly advised that this parameter be set to a value no less than 4 m\/s to provide additional protection against premature motor start\. Note that the GPS velocity will lag the real velocity by about 0\.5 seconds\. The ground speed check is delayed by the TKOFF\_THR\_DELAY parameter\.


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 30 | 0.1       | m/s   |
+--------+-----------+-------+




.. _TKOFF_THR_MINACC:

TKOFF\_THR\_MINACC: Takeoff throttle min acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum forward acceleration in m\/s\/s before arming the ground speed check in auto\-takeoff\. This is meant to be used for hand launches\. Setting this value to 0 disables the acceleration test which means the ground speed check will always be armed which could allow GPS velocity jumps to start the engine\. For hand launches and bungee launches this should be set to around 15\.


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 30 | 0.1       | m/s/s |
+--------+-----------+-------+




.. _TKOFF_THR_DELAY:

TKOFF\_THR\_DELAY: Takeoff throttle delay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the time delay \(in 1\/10ths of a second\) that the ground speed check is delayed after the forward acceleration check controlled by TKOFF\_THR\_MINACC has passed\. For hand launches with pusher propellers it is essential that this is set to a value of no less than 2 \(0\.2 seconds\) to ensure that the aircraft is safely clear of the throwers arm before the motor can start\. For bungee launches a larger value can be used \(such as 30\) to give time for the bungee to release from the aircraft before the motor is started\.


+---------+-----------+-------------+
| Range   | Increment | Units       |
+=========+===========+=============+
| 0 - 127 | 1         | 0.1 seconds |
+---------+-----------+-------------+




.. _TKOFF_TDRAG_ELEV:

TKOFF\_TDRAG\_ELEV: Takeoff tail dragger elevator
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the amount of elevator to apply during the initial stage of a takeoff\. It is used to hold the tail wheel of a taildragger on the ground during the initial takeoff stage to give maximum steering\. This option should be combined with the TKOFF\_TDRAG\_SPD1 option and the GROUND\_STEER\_ALT option along with tuning of the ground steering controller\. A value of zero means to bypass the initial \"tail hold\" stage of takeoff\. Set to zero for hand and catapult launch\. For tail\-draggers you should normally set this to 100\, meaning full up elevator during the initial stage of takeoff\. For most tricycle undercarriage aircraft a value of zero will work well\, but for some tricycle aircraft a small negative value \(say around \-20 to \-30\) will apply down elevator which will hold the nose wheel firmly on the ground during initial acceleration\. Only use a negative value if you find that the nosewheel doesn\'t grip well during takeoff\. Too much down elevator on a tricycle undercarriage may cause instability in steering as the plane pivots around the nosewheel\. Add down elevator 10 percent at a time\.


+------------+-----------+---------+
| Range      | Increment | Units   |
+============+===========+=========+
| -100 - 100 | 1         | Percent |
+------------+-----------+---------+




.. _TKOFF_TDRAG_SPD1:

TKOFF\_TDRAG\_SPD1: Takeoff tail dragger speed1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the airspeed at which to stop holding the tail down and transition to rudder control of steering on the ground\. When TKOFF\_TDRAG\_SPD1 is reached the pitch of the aircraft will be held level until TKOFF\_ROTATE\_SPD is reached\, at which point the takeoff pitch specified in the mission will be used to \"rotate\" the pitch for takeoff climb\. Set TKOFF\_TDRAG\_SPD1 to zero to go straight to rotation\. This should be set to zero for hand launch and catapult launch\. It should also be set to zero for tricycle undercarriages unless you are using the method above to genetly hold the nose wheel down\. For tail dragger aircraft it should be set just below the stall speed\.


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 30 | 0.1       | m/s   |
+--------+-----------+-------+




.. _TKOFF_ROTATE_SPD:

TKOFF\_ROTATE\_SPD: Takeoff rotate speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the airspeed at which the aircraft will \"rotate\"\, setting climb pitch specified in the mission\. If TKOFF\_ROTATE\_SPD is zero then the climb pitch will be used as soon as takeoff is started\. For hand launch and catapult launches a TKOFF\_ROTATE\_SPD of zero should be set\. For all ground launches TKOFF\_ROTATE\_SPD should be set above the stall speed\, usually by about 10 to 30 percent


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 30 | 0.1       | m/s   |
+--------+-----------+-------+




.. _TKOFF_THR_SLEW:

TKOFF\_THR\_SLEW: Takeoff throttle slew rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the slew rate for the throttle during auto takeoff\. When this is zero the THR\_SLEWRATE parameter is used during takeoff\. For rolling takeoffs it can be a good idea to set a lower slewrate for takeoff to give a slower acceleration which can improve ground steering control\. The value is a percentage throttle change per second\, so a value of 20 means to advance the throttle over 5 seconds on takeoff\. Values below 20 are not recommended as they may cause the plane to try to climb out with too little throttle\.


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 127 | 1         | percent |
+---------+-----------+---------+




.. _TKOFF_PLIM_SEC:

TKOFF\_PLIM\_SEC: Takeoff pitch limit reduction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter reduces the pitch minimum limit of an auto\-takeoff just a few seconds before it reaches the target altitude\. This reduces overshoot by allowing the flight controller to start leveling off a few seconds before reaching the target height\. When set to zero\, the mission pitch min is enforced all the way to and through the target altitude\, otherwise the pitch min slowly reduces to zero in the final segment\. This is the pitch\_min\, not the demand\. The flight controller should still be commanding to gain altitude to finish the takeoff but with this param it is not forcing it higher than it wants to be\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 10 | 0.5       | seconds |
+--------+-----------+---------+




.. _LAND_THR_SLEW:

LAND\_THR\_SLEW: Landing throttle slew rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the slew rate for the throttle during auto landing\. When this is zero the THR\_SLEWRATE parameter is used during landing\. The value is a percentage throttle change per second\, so a value of 20 means to advance the throttle over 5 seconds on landing\. Values below 50 are not recommended as it may cause a stall when airspeed is low and you can not throttle up fast enough\.


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 127 | 1         | percent |
+---------+-----------+---------+




.. _TKOFF_FLAP_PCNT:

TKOFF\_FLAP\_PCNT: Takeoff flap percentage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The amount of flaps \(as a percentage\) to apply in automatic takeoff


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 100 | Percent |
+---------+---------+




.. _FBWA_TDRAG_CHAN:

FBWA\_TDRAG\_CHAN: FBWA taildragger channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is a RC input channel which when it goes above 1700 enables FBWA taildragger takeoff mode\. It should be assigned to a momentary switch\. Once this feature is enabled it will stay enabled until the aircraft goes above TKOFF\_TDRAG\_SPD1 airspeed\, changes mode\, or the pitch goes above the initial pitch when this is engaged or goes below 0 pitch\. When enabled the elevator will be forced to TKOFF\_TDRAG\_ELEV\. This option allows for easier takeoffs on taildraggers in FBWA mode\, and also makes it easier to test auto\-takeoff steering handling in FBWA\. Setting it to 0 disables this option\.


.. _LEVEL_ROLL_LIMIT:

LEVEL\_ROLL\_LIMIT: Level flight roll limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls the maximum bank angle in degrees during flight modes where level flight is desired\, such as in the final stages of landing\, and during auto takeoff\. This should be a small angle \(such as 5 degrees\) to prevent a wing hitting the runway during takeoff or landing\. Setting this to zero will completely disable heading hold on auto takeoff and final landing approach\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 45 | 1         | degrees |
+--------+-----------+---------+




.. _LAND_SLOPE_RCALC:

LAND\_SLOPE\_RCALC: Landing slope re\-calc threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter is used when using a rangefinder during landing for altitude correction from baro drift \(RNGFND\_LANDING\=1\) and the altitude correction indicates your altitude is lower than the intended slope path\. This value is the threshold of the correction to re\-calculate the landing approach slope\. Set to zero to keep the original slope all the way down and any detected baro drift will be corrected by pitching\/throttling up to snap back to resume the original slope path\. Otherwise\, when a rangefinder altitude correction exceeds this threshold it will trigger a slope re\-calculate to give a shallower slope\. This also smoothes out the approach when flying over objects such as trees\. Recommend a value of 2m\.


+-------+-----------+--------+
| Range | Increment | Units  |
+=======+===========+========+
| 0 - 5 | 0.5       | meters |
+-------+-----------+--------+




.. _LAND_ABORT_DEG:

LAND\_ABORT\_DEG: Landing auto\-abort slope threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter is used when using a rangefinder during landing for altitude correction from baro drift \(RNGFND\_LANDING\=1\) and the altitude correction indicates your actual altitude is higher than the intended slope path\. Normally it would pitch down steeply but that can result in a crash with high airspeed so this allows remembering the baro offset and self\-abort the landing and come around for another landing with the correct baro offset applied for a perfect slope\. An auto\-abort go\-around will only happen once\, next attempt will not auto\-abort again\. This operation happens entirely automatically in AUTO mode\. This value is the delta degrees threshold to trigger the go\-around compared to the origional slope\. Example\: if set to 5 deg and the mission planned slope is 15 deg then if the new slope is 21 then it will go\-around\. Set to 0 to disable\. Requires LAND\_SLOPE\_RCALC \> 0\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 90 | 0.1       | degrees |
+--------+-----------+---------+




.. _LAND_PITCH_CD:

LAND\_PITCH\_CD: Landing Pitch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used in autoland to give the minimum pitch in the final stage of landing \(after the flare\)\. This parameter can be used to ensure that the final landing attitude is appropriate for the type of undercarriage on the aircraft\. Note that it is a minimum pitch only \- the landing code will control pitch above this value to try to achieve the configured landing sink rate\.


+---------------+
| Units         |
+===============+
| centi-Degrees |
+---------------+




.. _LAND_FLARE_ALT:

LAND\_FLARE\_ALT: Landing flare altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Altitude in autoland at which to lock heading and flare to the LAND\_PITCH\_CD pitch\. Note that this option is secondary to LAND\_FLARE\_SEC\. For a good landing it preferable that the flare is triggered by LAND\_FLARE\_SEC\. 


+-----------+--------+
| Increment | Units  |
+===========+========+
| 0.1       | meters |
+-----------+--------+




.. _LAND_FLARE_SEC:

LAND\_FLARE\_SEC: Landing flare time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Vertical time before landing point at which to lock heading and flare with the motor stopped\. This is vertical time\, and is calculated based solely on the current height above the ground and the current descent rate\.  Set to 0 if you only wish to flare based on altitude \(see LAND\_FLARE\_ALT\)\.


+-----------+---------+
| Increment | Units   |
+===========+=========+
| 0.1       | seconds |
+-----------+---------+




.. _LAND_PF_ALT:

LAND\_PF\_ALT: Landing pre\-flare altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Altitude to trigger pre\-flare flight stage where LAND\_PF\_ARSPD controls airspeed\. The pre\-flare flight stage trigger works just like LAND\_FLARE\_ALT but higher\. Disabled when LAND\_PF\_ARSPD is 0\.


+--------+-----------+--------+
| Range  | Increment | Units  |
+========+===========+========+
| 0 - 30 | 0.1       | meters |
+--------+-----------+--------+




.. _LAND_PF_SEC:

LAND\_PF\_SEC: Landing pre\-flare time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Vertical time to ground to trigger pre\-flare flight stage where LAND\_PF\_ARSPD controls airspeed\. This pre\-flare flight stage trigger works just like LAND\_FLARE\_SEC but earlier\. Disabled when LAND\_PF\_ARSPD is 0\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 10 | 0.1       | seconds |
+--------+-----------+---------+




.. _LAND_PF_ARSPD:

LAND\_PF\_ARSPD: Landing pre\-flare airspeed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Desired airspeed during pre\-flare flight stage\. This is useful to reduce airspeed just before the flare\. Use 0 to disable\.


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 30 | 0.1       | m/s   |
+--------+-----------+-------+




.. _USE_REV_THRUST:

USE\_REV\_THRUST: Bitmask for when to allow negative reverse thrust
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Typically THR\_MIN will be clipped to zero unless reverse thrust is available\. Since you may not want negative thrust available at all times this bitmask allows THR\_MIN to go below 0 while executing certain auto\-mission commands\.


+------------------------------+---------------------------------+
| Bitmask                      | Values                          |
+==============================+=================================+
| +-----+--------------------+ | +-------+---------------------+ |
| | Bit | Meaning            | | | Value | Meaning             | |
| +=====+====================+ | +=======+=====================+ |
| | 0   | AUTO_ALWAYS        | | | 0     | Disabled            | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 1   | AUTO_LAND          | | | 1     | AlwaysAllowedInAuto | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 2   | AUTO_LOITER_TO_ALT | | | 2     | Auto_LandApproach   | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 3   | AUTO_LOITER_ALL    | | | 4     | Auto_LoiterToAlt    | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 4   | AUTO_WAYPOINTS     | | | 8     | Auto_Loiter         | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 5   | LOITER             | | | 16    | Auto_Waypoint       | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 6   | RTL                | | | 32    | Loiter              | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 7   | CIRCLE             | | | 64    | RTL                 | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 8   | CRUISE             | | | 128   | Circle              | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 9   | FBWB               | | | 256   | Cruise              | |
| +-----+--------------------+ | +-------+---------------------+ |
| | 10  | GUIDED             | | | 512   | FBWB                | |
| +-----+--------------------+ | +-------+---------------------+ |
|                              | | 1024  | Guided              | |
|                              | +-------+---------------------+ |
|                              |                                 |
+------------------------------+---------------------------------+




.. _LAND_DISARMDELAY:

LAND\_DISARMDELAY: Landing disarm delay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

After a landing has completed using a LAND waypoint\, automatically disarm after this many seconds have passed\. Use 0 to not disarm\.


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 127 | 1         | seconds |
+---------+-----------+---------+




.. _LAND_THEN_NEUTRL:

LAND\_THEN\_NEUTRL: Set servos to neutral after landing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

When enabled\, after an autoland and auto\-disarm via LAND\_DISARMDELAY happens then set all servos to neutral\. This is helpful when an aircraft has a rough landing upside down or a crazy angle causing the servos to strain\.


+--------------------------------+
| Values                         |
+================================+
| +-------+--------------------+ |
| | Value | Meaning            | |
| +=======+====================+ |
| | 0     | Disabled           | |
| +-------+--------------------+ |
| | 1     | Servos to Neutral  | |
| +-------+--------------------+ |
| | 2     | Servos to Zero PWM | |
| +-------+--------------------+ |
|                                |
+--------------------------------+




.. _LAND_ABORT_THR:

LAND\_ABORT\_THR: Landing abort using throttle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Allow a landing abort to trigger with a throttle \> 95\%


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _NAV_CONTROLLER:

NAV\_CONTROLLER: Navigation controller selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Which navigation controller to enable\. Currently the only navigation controller available is L1\. From time to time other experimental controllers will be added which are selected using this parameter\.


+--------------------------+
| Values                   |
+==========================+
| +-------+--------------+ |
| | Value | Meaning      | |
| +=======+==============+ |
| | 0     | Default      | |
| +-------+--------------+ |
| | 1     | L1Controller | |
| +-------+--------------+ |
|                          |
+--------------------------+




.. _ALT_MIX:

ALT\_MIX: GPS to Baro Mix
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The percent of mixing between GPS altitude and baro altitude\. 0 \= 100\% gps\, 1 \= 100\% baro\. It is highly recommend that you not change this from the default of 1\, as GPS altitude is notoriously unreliable\. The only time I would recommend changing this is if you have a high altitude enabled GPS\, and you are dropping a plane from a high altitude balloon many kilometers off the ground\.


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 1 | 0.1       | Percent |
+-------+-----------+---------+




.. _ALT_CTRL_ALG:

ALT\_CTRL\_ALG: Altitude control algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets what algorithm will be used for altitude control\. The default is zero\, which selects the most appropriate algorithm for your airframe\. Currently the default is to use TECS \(total energy control system\)\. From time to time we will add other experimental altitude control algorithms which will be selected using this parameter\.


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Automatic | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _ALT_OFFSET:

ALT\_OFFSET: Altitude offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is added to the target altitude in automatic flight\. It can be used to add a global altitude offset to a mission


+----------------+-----------+--------+
| Range          | Increment | Units  |
+================+===========+========+
| -32767 - 32767 | 1         | Meters |
+----------------+-----------+--------+




.. _WP_RADIUS:

WP\_RADIUS: Waypoint Radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the maximum distance from a waypoint that when crossed indicates the waypoint may be complete\. To avoid the aircraft looping around the waypoint in case it misses by more than the WP\_RADIUS an additional check is made to see if the aircraft has crossed a \"finish line\" passing through the waypoint and perpendicular to the flight path from the previous waypoint\. If that finish line is crossed then the waypoint is considered complete\. Note that the navigation controller may decide to turn later than WP\_RADIUS before a waypoint\, based on how sharp the turn is and the speed of the aircraft\. It is safe to set WP\_RADIUS much larger than the usual turn radius of your aircraft and the navigation controller will work out when to turn\. If you set WP\_RADIUS too small then you will tend to overshoot the turns\.


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 1 - 32767 | 1         | Meters |
+-----------+-----------+--------+




.. _WP_MAX_RADIUS:

WP\_MAX\_RADIUS: Waypoint Maximum Radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the maximum distance to a waypoint for the waypoint to be considered complete\. This overrides the \"cross the finish line\" logic that is normally used to consider a waypoint complete\. For normal AUTO behaviour this parameter should be set to zero\. Using a non\-zero value is only recommended when it is critical that the aircraft does approach within the given radius\, and should loop around until it has done so\. This can cause the aircraft to loop forever if its turn radius is greater than the maximum radius set\.


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 0 - 32767 | 1         | Meters |
+-----------+-----------+--------+




.. _WP_LOITER_RAD:

WP\_LOITER\_RAD: Waypoint Loiter Radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the distance from the waypoint center\, the plane will maintain during a loiter\. If you set this value to a negative number then the default loiter direction will be counter\-clockwise instead of clockwise\.


+----------------+-----------+--------+
| Range          | Increment | Units  |
+================+===========+========+
| -32767 - 32767 | 1         | Meters |
+----------------+-----------+--------+




.. _RTL_RADIUS:

RTL\_RADIUS: RTL loiter radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the radius of the loiter circle when in RTL mode\. If this is zero then WP\_LOITER\_RAD is used\. If the radius is negative then a counter\-clockwise is used\. If positive then a clockwise loiter is used\.


+----------------+-----------+--------+
| Range          | Increment | Units  |
+================+===========+========+
| -32767 - 32767 | 1         | Meters |
+----------------+-----------+--------+




.. _FENCE_ACTION:

FENCE\_ACTION: Action on geofence breach
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


What to do on fence breach\. If this is set to 0 then no action is taken\, and geofencing is disabled\. If this is set to 1 then the plane will enter GUIDED mode\, with the target waypoint as the fence return point\. If this is set to 2 then the fence breach is reported to the ground station\, but no other action is taken\. If set to 3 then the plane enters guided mode but the pilot retains manual throttle control\.


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | None              | |
| +-------+-------------------+ |
| | 1     | GuidedMode        | |
| +-------+-------------------+ |
| | 2     | ReportOnly        | |
| +-------+-------------------+ |
| | 3     | GuidedModeThrPass | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _FENCE_TOTAL:

FENCE\_TOTAL: Fence Total
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Number of geofence points currently loaded


.. _FENCE_CHANNEL:

FENCE\_CHANNEL: Fence Channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RC Channel to use to enable geofence\. PWM input above 1750 enables the geofence


.. _FENCE_MINALT:

FENCE\_MINALT: Fence Minimum Altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum altitude allowed before geofence triggers


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 0 - 32767 | 1         | meters |
+-----------+-----------+--------+




.. _FENCE_MAXALT:

FENCE\_MAXALT: Fence Maximum Altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum altitude allowed before geofence triggers


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 0 - 32767 | 1         | meters |
+-----------+-----------+--------+




.. _FENCE_RETALT:

FENCE\_RETALT: Fence Return Altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Altitude the aircraft will transit to when a fence breach occurs\.  If FENCE\_RETALT is \<\= 0 then the midpoint between FENCE\_MAXALT and FENCE\_MINALT is used\, unless FENCE\_MAXALT \< FENCE\_MINALT\.  If FENCE\_MAXALT \< FENCE\_MINALT AND FENCE\_RETALT is \<\= 0 then ALT\_HOLD\_RTL is the altitude used on a fence breach\.


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 0 - 32767 | 1         | meters |
+-----------+-----------+--------+




.. _FENCE_AUTOENABLE:

FENCE\_AUTOENABLE: Fence automatic enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When set to 1\, geofence automatically enables after an auto takeoff and automatically disables at the beginning of an auto landing\.  When on the ground before takeoff the fence is disabled\.  When set to 2\, the fence autoenables after an auto takeoff\, but only disables the fence floor during landing\. It is highly recommended to not use this option for line of sight flying and use a fence enable channel instead\.


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | NoAutoEnable               | |
| +-------+----------------------------+ |
| | 1     | AutoEnable                 | |
| +-------+----------------------------+ |
| | 2     | AutoEnableDisableFloorOnly | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _FENCE_RET_RALLY:

FENCE\_RET\_RALLY: Fence Return to Rally
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When set to 1\: on fence breach the plane will return to the nearest rally point rather than the fence return point\.  If no rally points have been defined the plane will return to the home point\.  


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | FenceReturnPoint  | |
| +-------+-------------------+ |
| | 1     | NearestRallyPoint | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _STALL_PREVENTION:

STALL\_PREVENTION: Enable stall prevention
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls the use of stall prevention techniques\, including roll limits at low speed and raising the minimum airspeed in turns\. The limits are based on the aerodynamic load factor of a banked turn\. This option relies on the correct ARSPD\_FBW\_MIN value being set correctly\. Note that if you don\'t have an airspeed sensor then stall prevention will use an airspeed estimate based on the ground speed plus a wind estimate taken from the response of the autopilot banked turns\. That synthetic airspeed estimate may be inaccurate\, so you should not assume that stall prevention with no airspeed sensor will be effective\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _ARSPD_FBW_MIN:

ARSPD\_FBW\_MIN: Minimum Airspeed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the minimum airspeed you want to fly at in modes where the autopilot controls the airspeed\. This should be set to a value around 20\% higher than the level flight stall speed for the airframe\. This value is also used in the STALL\_PREVENTION code\.


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 5 - 100 | 1         | m/s   |
+---------+-----------+-------+




.. _ARSPD_FBW_MAX:

ARSPD\_FBW\_MAX: Maximum Airspeed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the maximum airspeed that you want to allow for your airframe in auto\-throttle modes\. You should ensure that this value is sufficiently above the ARSPD\_FBW\_MIN value to allow for a sufficient flight envelope to accurately control altitude using airspeed\. A value at least 50\% above ARSPD\_FBW\_MIN is recommended\.


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 5 - 100 | 1         | m/s   |
+---------+-----------+-------+




.. _FBWB_ELEV_REV:

FBWB\_ELEV\_REV: Fly By Wire elevator reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse sense of elevator in FBWB and CRUISE modes\. When set to 0 up elevator \(pulling back on the stick\) means to lower altitude\. When set to 1\, up elevator means to raise altitude\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _TERRAIN_FOLLOW:

TERRAIN\_FOLLOW: Use terrain following
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This enables terrain following for CRUISE mode\, FBWB mode\, RTL and for rally points\. To use this option you also need to set TERRAIN\_ENABLE to 1\, which enables terrain data fetching from the GCS\, and you need to have a GCS that supports sending terrain data to the aircraft\. When terrain following is enabled then CRUISE and FBWB mode will hold height above terrain rather than height above home\. In RTL the return to launch altitude will be considered to be a height above the terrain\. Rally point altitudes will be taken as height above the terrain\. This option does not affect mission items\, which have a per\-waypoint flag for whether they are height above home or height above the terrain\. To use terrain following missions you need a ground station which can set the waypoint type to be a terrain height waypoint when creating the mission\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _TERRAIN_LOOKAHD:

TERRAIN\_LOOKAHD: Terrain lookahead
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how far ahead the terrain following code looks to ensure it stays above upcoming terrain\. A value of zero means no lookahead\, so the controller will track only the terrain directly below the aircraft\. The lookahead will never extend beyond the next waypoint when in AUTO mode\.


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 10000 | meters |
+-----------+--------+




.. _FBWB_CLIMB_RATE:

FBWB\_CLIMB\_RATE: Fly By Wire B altitude change rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the rate in m\/s at which FBWB and CRUISE modes will change its target altitude for full elevator deflection\. Note that the actual climb rate of the aircraft can be lower than this\, depending on your airspeed and throttle control settings\. If you have this parameter set to the default value of 2\.0\, then holding the elevator at maximum deflection for 10 seconds would change the target altitude by 20 meters\.


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 1 - 10 | 0.1       | m/s   |
+--------+-----------+-------+




.. _THR_MIN:

THR\_MIN: Minimum Throttle
~~~~~~~~~~~~~~~~~~~~~~~~~~


The minimum throttle setting \(as a percentage\) which the autopilot will apply\. For the final stage of an automatic landing this is always zero\. If your ESC supports reverse\, use a negative value to configure for reverse thrust\.


+------------+-----------+---------+
| Range      | Increment | Units   |
+============+===========+=========+
| -100 - 100 | 1         | Percent |
+------------+-----------+---------+




.. _THR_MAX:

THR\_MAX: Maximum Throttle
~~~~~~~~~~~~~~~~~~~~~~~~~~


The maximum throttle setting \(as a percentage\) which the autopilot will apply\.


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 100 | 1         | Percent |
+---------+-----------+---------+




.. _TKOFF_THR_MAX:

TKOFF\_THR\_MAX: Maximum Throttle for takeoff
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The maximum throttle setting during automatic takeoff\. If this is zero then THR\_MAX is used for takeoff as well\.


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 100 | 1         | Percent |
+---------+-----------+---------+




.. _THR_SLEWRATE:

THR\_SLEWRATE: Throttle slew rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


maximum percentage change in throttle per second\. A setting of 10 means to not change the throttle by more than 10\% of the full throttle range in one second\.


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 127 | 1         | Percent |
+---------+-----------+---------+




.. _FLAP_SLEWRATE:

FLAP\_SLEWRATE: Flap slew rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

maximum percentage change in flap output per second\. A setting of 25 means to not change the flap by more than 25\% of the full flap range in one second\. A value of 0 means no rate limiting\.


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 100 | 1         | Percent |
+---------+-----------+---------+




.. _THR_SUPP_MAN:

THR\_SUPP\_MAN: Throttle suppress manual passthru
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

When throttle is suppressed in auto mode it is normally forced to zero\. If you enable this option\, then while suppressed it will be manual throttle\. This is useful on petrol engines to hold the idle throttle manually while waiting for takeoff


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _THR_PASS_STAB:

THR\_PASS\_STAB: Throttle passthru in stabilize
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If this is set then when in STABILIZE\, FBWA or ACRO modes the throttle is a direct passthru from the transmitter\. This means the THR\_MIN and THR\_MAX settings are not used in these modes\. This is useful for petrol engines where you setup a throttle cut switch that suppresses the throttle below the normal minimum\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _THR_FAILSAFE:

THR\_FAILSAFE: Throttle Failsafe Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _THR_FS_VALUE:

THR\_FS\_VALUE: Throttle Failsafe Value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The PWM level on channel 3 below which throttle failsafe triggers


+------------+-----------+
| Range      | Increment |
+============+===========+
| 925 - 2200 | 1         |
+------------+-----------+




.. _TRIM_THROTTLE:

TRIM\_THROTTLE: Throttle cruise percentage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The target percentage of throttle to apply for normal flight


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 100 | 1         | Percent |
+---------+-----------+---------+




.. _THROTTLE_NUDGE:

THROTTLE\_NUDGE: Throttle nudge enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When enabled\, this uses the throttle input in auto\-throttle modes to \'nudge\' the throttle or airspeed to higher or lower values\. When you have an airspeed sensor the nudge affects the target airspeed\, so that throttle inputs above 50\% will increase the target airspeed from TRIM\_ARSPD\_CM up to a maximum of ARSPD\_FBW\_MAX\. When no airspeed sensor is enabled the throttle nudge will push up the target throttle for throttle inputs above 50\%\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _FS_SHORT_ACTN:

FS\_SHORT\_ACTN: Short failsafe action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The action to take on a short \(FS\_SHORT\_TIMEOUT\) failsafe event\. A short failsafe even can be triggered either by loss of RC control \(see THR\_FS\_VALUE\) or by loss of GCS control \(see FS\_GCS\_ENABL\)\. If in CIRCLE or RTL mode this parameter is ignored\. A short failsafe event in stabilization and manual modes will cause an change to CIRCLE mode if FS\_SHORT\_ACTN is 0 or 1\, and a change to FBWA mode if FS\_SHORT\_ACTN is 2\. In all other modes \(AUTO\, GUIDED and LOITER\) a short failsafe event will cause no mode change is FS\_SHORT\_ACTN is set to 0\, will cause a change to CIRCLE mode if set to 1 and will change to FBWA mode if set to 2\. Please see the documentation for FS\_LONG\_ACTN for the behaviour after FS\_LONG\_TIMEOUT seconds of failsafe\.


+----------------------------------------------------------------+
| Values                                                         |
+================================================================+
| +-------+----------------------------------------------------+ |
| | Value | Meaning                                            | |
| +=======+====================================================+ |
| | 0     | CIRCLE/no change(if already in AUTO|GUIDED|LOITER) | |
| +-------+----------------------------------------------------+ |
| | 1     | CIRCLE                                             | |
| +-------+----------------------------------------------------+ |
| | 2     | FBWA                                               | |
| +-------+----------------------------------------------------+ |
|                                                                |
+----------------------------------------------------------------+




.. _FS_SHORT_TIMEOUT:

FS\_SHORT\_TIMEOUT: Short failsafe timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The time in seconds that a failsafe condition has to persist before a short failsafe event will occur\. This defaults to 1\.5 seconds


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 1 - 100 | 0.5       | seconds |
+---------+-----------+---------+




.. _FS_LONG_ACTN:

FS\_LONG\_ACTN: Long failsafe action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The action to take on a long \(FS\_LONG\_TIMEOUT seconds\) failsafe event\. If the aircraft was in a stabilization or manual mode when failsafe started and a long failsafe occurs then it will change to RTL mode if FS\_LONG\_ACTN is 0 or 1\, and will change to FBWA if FS\_LONG\_ACTN is set to 2\. If the aircraft was in an auto mode \(such as AUTO or GUIDED\) when the failsafe started then it will continue in the auto mode if FS\_LONG\_ACTN is set to 0\, will change to RTL mode if FS\_LONG\_ACTN is set to 1 and will change to FBWA mode if FS\_LONG\_ACTN is set to 2\. If FS\_LONG\_ACTION is set to 3\, the parachute will be deployed \(make sure the chute is configured and enabled\)\. 


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | Continue         | |
| +-------+------------------+ |
| | 1     | ReturnToLaunch   | |
| +-------+------------------+ |
| | 2     | Glide            | |
| +-------+------------------+ |
| | 3     | Deploy Parachute | |
| +-------+------------------+ |
|                              |
+------------------------------+




.. _FS_LONG_TIMEOUT:

FS\_LONG\_TIMEOUT: Long failsafe timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The time in seconds that a failsafe condition has to persist before a long failsafe event will occur\. This defaults to 5 seconds\.


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 1 - 300 | 0.5       | seconds |
+---------+-----------+---------+




.. _FS_BATT_VOLTAGE:

FS\_BATT\_VOLTAGE: Failsafe battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage to trigger failsafe\. Set to 0 to disable battery voltage failsafe\. If the battery voltage drops below this voltage continuously for 10 seconds then the plane will switch to RTL mode\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | Volts |
+-----------+-------+




.. _FS_BATT_MAH:

FS\_BATT\_MAH: Failsafe battery milliAmpHours
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity remaining to trigger failsafe\. Set to 0 to disable battery remaining failsafe\. If the battery remaining drops below this level then the plane will switch to RTL mode immediately\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 50        | mAh   |
+-----------+-------+




.. _FS_GCS_ENABL:

FS\_GCS\_ENABL: GCS failsafe enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable ground control station telemetry failsafe\. Failsafe will trigger after FS\_LONG\_TIMEOUT seconds of no MAVLink heartbeat messages\. There are two possible enabled settings\. Seeing FS\_GCS\_ENABL to 1 means that GCS failsafe will be triggered when the aircraft has not received a MAVLink HEARTBEAT message\. Setting FS\_GCS\_ENABL to 2 means that GCS failsafe will be triggered on either a loss of HEARTBEAT messages\, or a RADIO\_STATUS message from a MAVLink enabled 3DR radio indicating that the ground station is not receiving status updates from the aircraft\, which is indicated by the RADIO\_STATUS\.remrssi field being zero \(this may happen if you have a one way link due to asymmetric noise on the ground station and aircraft radios\)\.Setting FS\_GCS\_ENABL to 3 means that GCS failsafe will be triggered by Heartbeat\(like option one\)\, but only in AUTO mode\. WARNING\: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground it it loses contact with your ground station\. If this option is enabled on an electric plane then you should enable ARMING\_REQUIRED\.


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | Disabled            | |
| +-------+---------------------+ |
| | 1     | Heartbeat           | |
| +-------+---------------------+ |
| | 2     | HeartbeatAndREMRSSI | |
| +-------+---------------------+ |
| | 3     | HeartbeatAndAUTO    | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _FLTMODE_CH:

FLTMODE\_CH: Flightmode channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC Channel to use for flight mode control


.. _FLTMODE1:

FLTMODE1: FlightMode1
~~~~~~~~~~~~~~~~~~~~~


Flight mode for switch position 1 \(910 to 1230 and above 2049\)


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Manual    | |
| +-------+-----------+ |
| | 1     | CIRCLE    | |
| +-------+-----------+ |
| | 2     | STABILIZE | |
| +-------+-----------+ |
| | 3     | TRAINING  | |
| +-------+-----------+ |
| | 4     | ACRO      | |
| +-------+-----------+ |
| | 5     | FBWA      | |
| +-------+-----------+ |
| | 6     | FBWB      | |
| +-------+-----------+ |
| | 7     | CRUISE    | |
| +-------+-----------+ |
| | 8     | AUTOTUNE  | |
| +-------+-----------+ |
| | 10    | Auto      | |
| +-------+-----------+ |
| | 11    | RTL       | |
| +-------+-----------+ |
| | 12    | Loiter    | |
| +-------+-----------+ |
| | 15    | Guided    | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE2:

FLTMODE2: FlightMode2
~~~~~~~~~~~~~~~~~~~~~


Flight mode for switch position 2 \(1231 to 1360\)


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Manual    | |
| +-------+-----------+ |
| | 1     | CIRCLE    | |
| +-------+-----------+ |
| | 2     | STABILIZE | |
| +-------+-----------+ |
| | 3     | TRAINING  | |
| +-------+-----------+ |
| | 4     | ACRO      | |
| +-------+-----------+ |
| | 5     | FBWA      | |
| +-------+-----------+ |
| | 6     | FBWB      | |
| +-------+-----------+ |
| | 7     | CRUISE    | |
| +-------+-----------+ |
| | 8     | AUTOTUNE  | |
| +-------+-----------+ |
| | 10    | Auto      | |
| +-------+-----------+ |
| | 11    | RTL       | |
| +-------+-----------+ |
| | 12    | Loiter    | |
| +-------+-----------+ |
| | 15    | Guided    | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE3:

FLTMODE3: FlightMode3
~~~~~~~~~~~~~~~~~~~~~


Flight mode for switch position 3 \(1361 to 1490\)


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Manual    | |
| +-------+-----------+ |
| | 1     | CIRCLE    | |
| +-------+-----------+ |
| | 2     | STABILIZE | |
| +-------+-----------+ |
| | 3     | TRAINING  | |
| +-------+-----------+ |
| | 4     | ACRO      | |
| +-------+-----------+ |
| | 5     | FBWA      | |
| +-------+-----------+ |
| | 6     | FBWB      | |
| +-------+-----------+ |
| | 7     | CRUISE    | |
| +-------+-----------+ |
| | 8     | AUTOTUNE  | |
| +-------+-----------+ |
| | 10    | Auto      | |
| +-------+-----------+ |
| | 11    | RTL       | |
| +-------+-----------+ |
| | 12    | Loiter    | |
| +-------+-----------+ |
| | 15    | Guided    | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE4:

FLTMODE4: FlightMode4
~~~~~~~~~~~~~~~~~~~~~


Flight mode for switch position 4 \(1491 to 1620\)


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Manual    | |
| +-------+-----------+ |
| | 1     | CIRCLE    | |
| +-------+-----------+ |
| | 2     | STABILIZE | |
| +-------+-----------+ |
| | 3     | TRAINING  | |
| +-------+-----------+ |
| | 4     | ACRO      | |
| +-------+-----------+ |
| | 5     | FBWA      | |
| +-------+-----------+ |
| | 6     | FBWB      | |
| +-------+-----------+ |
| | 7     | CRUISE    | |
| +-------+-----------+ |
| | 8     | AUTOTUNE  | |
| +-------+-----------+ |
| | 10    | Auto      | |
| +-------+-----------+ |
| | 11    | RTL       | |
| +-------+-----------+ |
| | 12    | Loiter    | |
| +-------+-----------+ |
| | 15    | Guided    | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE5:

FLTMODE5: FlightMode5
~~~~~~~~~~~~~~~~~~~~~


Flight mode for switch position 5 \(1621 to 1749\)


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Manual    | |
| +-------+-----------+ |
| | 1     | CIRCLE    | |
| +-------+-----------+ |
| | 2     | STABILIZE | |
| +-------+-----------+ |
| | 3     | TRAINING  | |
| +-------+-----------+ |
| | 4     | ACRO      | |
| +-------+-----------+ |
| | 5     | FBWA      | |
| +-------+-----------+ |
| | 6     | FBWB      | |
| +-------+-----------+ |
| | 7     | CRUISE    | |
| +-------+-----------+ |
| | 8     | AUTOTUNE  | |
| +-------+-----------+ |
| | 10    | Auto      | |
| +-------+-----------+ |
| | 11    | RTL       | |
| +-------+-----------+ |
| | 12    | Loiter    | |
| +-------+-----------+ |
| | 15    | Guided    | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE6:

FLTMODE6: FlightMode6
~~~~~~~~~~~~~~~~~~~~~


Flight mode for switch position 6 \(1750 to 2049\)


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Manual    | |
| +-------+-----------+ |
| | 1     | CIRCLE    | |
| +-------+-----------+ |
| | 2     | STABILIZE | |
| +-------+-----------+ |
| | 3     | TRAINING  | |
| +-------+-----------+ |
| | 4     | ACRO      | |
| +-------+-----------+ |
| | 5     | FBWA      | |
| +-------+-----------+ |
| | 6     | FBWB      | |
| +-------+-----------+ |
| | 7     | CRUISE    | |
| +-------+-----------+ |
| | 8     | AUTOTUNE  | |
| +-------+-----------+ |
| | 10    | Auto      | |
| +-------+-----------+ |
| | 11    | RTL       | |
| +-------+-----------+ |
| | 12    | Loiter    | |
| +-------+-----------+ |
| | 15    | Guided    | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _INITIAL_MODE:

INITIAL\_MODE: Initial flight mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This selects the mode to start in on boot\. This is useful for when you want to start in AUTO mode on boot without a receiver\.


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Manual    | |
| +-------+-----------+ |
| | 1     | CIRCLE    | |
| +-------+-----------+ |
| | 2     | STABILIZE | |
| +-------+-----------+ |
| | 3     | TRAINING  | |
| +-------+-----------+ |
| | 4     | ACRO      | |
| +-------+-----------+ |
| | 5     | FBWA      | |
| +-------+-----------+ |
| | 6     | FBWB      | |
| +-------+-----------+ |
| | 7     | CRUISE    | |
| +-------+-----------+ |
| | 8     | AUTOTUNE  | |
| +-------+-----------+ |
| | 10    | Auto      | |
| +-------+-----------+ |
| | 11    | RTL       | |
| +-------+-----------+ |
| | 12    | Loiter    | |
| +-------+-----------+ |
| | 15    | Guided    | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _LIM_ROLL_CD:

LIM\_ROLL\_CD: Maximum Bank Angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The maximum commanded bank angle in either direction


+----------+-----------+---------------+
| Range    | Increment | Units         |
+==========+===========+===============+
| 0 - 9000 | 1         | centi-Degrees |
+----------+-----------+---------------+




.. _LIM_PITCH_MAX:

LIM\_PITCH\_MAX: Maximum Pitch Angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The maximum commanded pitch up angle


+----------+-----------+---------------+
| Range    | Increment | Units         |
+==========+===========+===============+
| 0 - 9000 | 1         | centi-Degrees |
+----------+-----------+---------------+




.. _LIM_PITCH_MIN:

LIM\_PITCH\_MIN: Minimum Pitch Angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The minimum commanded pitch down angle


+-----------+-----------+---------------+
| Range     | Increment | Units         |
+===========+===========+===============+
| -9000 - 0 | 1         | centi-Degrees |
+-----------+-----------+---------------+




.. _ACRO_ROLL_RATE:

ACRO\_ROLL\_RATE: ACRO mode roll rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The maximum roll rate at full stick deflection in ACRO mode


+----------+-----------+----------------+
| Range    | Increment | Units          |
+==========+===========+================+
| 10 - 500 | 1         | degrees/second |
+----------+-----------+----------------+




.. _ACRO_PITCH_RATE:

ACRO\_PITCH\_RATE: ACRO mode pitch rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The maximum pitch rate at full stick deflection in ACRO mode


+----------+-----------+----------------+
| Range    | Increment | Units          |
+==========+===========+================+
| 10 - 500 | 1         | degrees/second |
+----------+-----------+----------------+




.. _ACRO_LOCKING:

ACRO\_LOCKING: ACRO mode attitude locking
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable attitude locking when sticks are released


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _GROUND_STEER_ALT:

GROUND\_STEER\_ALT: Ground steer altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Altitude at which to use the ground steering controller on the rudder\. If non\-zero then the STEER2SRV controller will be used to control the rudder for altitudes within this limit of the home altitude\.


+------------+-----------+--------+
| Range      | Increment | Units  |
+============+===========+========+
| -100 - 100 | 0.1       | Meters |
+------------+-----------+--------+




.. _GROUND_STEER_DPS:

GROUND\_STEER\_DPS: Ground steer rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Ground steering rate in degrees per second for full rudder stick deflection


+----------+-----------+----------------+
| Range    | Increment | Units          |
+==========+===========+================+
| 10 - 360 | 1         | degrees/second |
+----------+-----------+----------------+




.. _TRIM_AUTO:

TRIM\_AUTO: Automatic trim adjustment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Set RC trim PWM levels to current levels when switching away from manual mode\. When this option is enabled and you change from MANUAL to any other mode then the APM will take the current position of the control sticks as the trim values for aileron\, elevator and rudder\. It will use those to set RC1\_TRIM\, RC2\_TRIM and RC4\_TRIM\. This option is disabled by default as if a pilot is not aware of this option and changes from MANUAL to another mode while control inputs are not centered then the trim could be changed to a dangerously bad value\. You can enable this option to assist with trimming your plane\, by enabling it before takeoff then switching briefly to MANUAL in flight\, and seeing how the plane reacts\. You can then switch back to FBWA\, trim the surfaces then again test MANUAL mode\. Each time you switch from MANUAL the APM will take your control inputs as the new trim\. After you have good trim on your aircraft you can disable TRIM\_AUTO for future flights\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _ELEVON_MIXING:

ELEVON\_MIXING: Elevon mixing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This enables an older form of elevon mixing which is now deprecated\. Please see the ELEVON\_OUTPUT option for setting up elevons\. The ELEVON\_MIXING option should be set to 0 for elevon planes except for backwards compatibility with older setups\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _ELEVON_REVERSE:

ELEVON\_REVERSE: Elevon reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse elevon mixing


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _ELEVON_CH1_REV:

ELEVON\_CH1\_REV: Elevon reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse elevon channel 1


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _ELEVON_CH2_REV:

ELEVON\_CH2\_REV: Elevon reverse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reverse elevon channel 2


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _VTAIL_OUTPUT:

VTAIL\_OUTPUT: VTail output
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable VTail output in software\. If enabled then the APM will provide software VTail mixing on the elevator and rudder channels\. There are 4 different mixing modes available\, which refer to the 4 ways the elevator can be mapped to the two VTail servos\. Note that you must not use VTail output mixing with hardware pass\-through of RC values\, such as with channel 8 manual control on an APM1\. So if you use an APM1 then set FLTMODE\_CH to something other than 8 before you enable VTAIL\_OUTPUT\. Please also see the MIXING\_GAIN parameter for the output gain of the mixer\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | UpUp     | |
| +-------+----------+ |
| | 2     | UpDown   | |
| +-------+----------+ |
| | 3     | DownUp   | |
| +-------+----------+ |
| | 4     | DownDown | |
| +-------+----------+ |
|                      |
+----------------------+




.. _ELEVON_OUTPUT:

ELEVON\_OUTPUT: Elevon output
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable software elevon output mixer\. If enabled then the APM will provide software elevon mixing on the aileron and elevator channels\. There are 4 different mixing modes available\, which refer to the 4 ways the elevator can be mapped to the two elevon servos\. Note that you must not use elevon output mixing with hardware pass\-through of RC values\, such as with channel 8 manual control on an APM1\. So if you use an APM1 then set FLTMODE\_CH to something other than 8 before you enable ELEVON\_OUTPUT\. Please also see the MIXING\_GAIN parameter for the output gain of the mixer\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | UpUp     | |
| +-------+----------+ |
| | 2     | UpDown   | |
| +-------+----------+ |
| | 3     | DownUp   | |
| +-------+----------+ |
| | 4     | DownDown | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MIXING_GAIN:

MIXING\_GAIN: Mixing Gain
~~~~~~~~~~~~~~~~~~~~~~~~~


The gain for the Vtail and elevon output mixers\. The default is 0\.5\, which ensures that the mixer doesn\'t saturate\, allowing both input channels to go to extremes while retaining control over the output\. Hardware mixers often have a 1\.0 gain\, which gives more servo throw\, but can saturate\. If you don\'t have enough throw on your servos with VTAIL\_OUTPUT or ELEVON\_OUTPUT enabled then you can raise the gain using MIXING\_GAIN\. The mixer allows outputs in the range 900 to 2100 microseconds\.


+-----------+
| Range     |
+===========+
| 0.5 - 1.2 |
+-----------+




.. _RUDDER_ONLY:

RUDDER\_ONLY: Rudder only aircraft
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable rudder only mode\. The rudder will control attitude in attitude controlled modes \(such as FBWA\)\. You should setup your transmitter to send roll stick inputs to the RCMAP\_YAW channel \(normally channel 4\)\. The rudder servo should be attached to the RCMAP\_YAW channel as well\. Note that automatic ground steering will be disabled for rudder only aircraft\. You should also set KFF\_RDDRMIX to 1\.0\. You will also need to setup the YAW2SRV\_DAMP yaw damping appropriately for your aircraft\. A value of 0\.5 for YAW2SRV\_DAMP is a good starting point\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _SYS_NUM_RESETS:

SYS\_NUM\_RESETS: Num Resets
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Number of APM board resets


.. _LOG_BITMASK:

LOG\_BITMASK: Log bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmap of what log types to enable in dataflash\. This values is made up of the sum of each of the log types you want to be saved on dataflash\. On a PX4 or Pixhawk the large storage size of a microSD card means it is usually best just to enable all log types by setting this to 65535\. On APM2 the smaller 4 MByte dataflash means you need to be more selective in your logging or you may run out of log space while flying \(in which case it will wrap and overwrite the start of the log\)\. The individual bits are ATTITUDE\_FAST\=1\, ATTITUDE\_MEDIUM\=2\, GPS\=4\, PerformanceMonitoring\=8\, ControlTuning\=16\, NavigationTuning\=32\, Mode\=64\, IMU\=128\, Commands\=256\, Battery\=512\, Compass\=1024\, TECS\=2048\, Camera\=4096\, RCandServo\=8192\, Sonar\=16384\, Arming\=32768\, FullLogs\=65535


+-------------------------+---------------------------------+
| Bitmask                 | Values                          |
+=========================+=================================+
| +-----+---------------+ | +-------+---------------------+ |
| | Bit | Meaning       | | | Value | Meaning             | |
| +=====+===============+ | +=======+=====================+ |
| | 0   | ATTITUDE_FAST | | | 0     | Disabled            | |
| +-----+---------------+ | +-------+---------------------+ |
| | 1   | ATTITUDE_MED  | | | 5190  | APM2-Default        | |
| +-----+---------------+ | +-------+---------------------+ |
| | 2   | GPS           | | | 65535 | PX4/Pixhawk-Default | |
| +-----+---------------+ | +-------+---------------------+ |
| | 3   | PM            | |                                 |
| +-----+---------------+ |                                 |
| | 4   | CTUN          | |                                 |
| +-----+---------------+ |                                 |
| | 5   | NTUN          | |                                 |
| +-----+---------------+ |                                 |
| | 6   | MODE          | |                                 |
| +-----+---------------+ |                                 |
| | 7   | IMU           | |                                 |
| +-----+---------------+ |                                 |
| | 8   | CMD           | |                                 |
| +-----+---------------+ |                                 |
| | 9   | CURRENT       | |                                 |
| +-----+---------------+ |                                 |
| | 10  | COMPASS       | |                                 |
| +-----+---------------+ |                                 |
| | 11  | TECS          | |                                 |
| +-----+---------------+ |                                 |
| | 12  | CAMERA        | |                                 |
| +-----+---------------+ |                                 |
| | 13  | RC            | |                                 |
| +-----+---------------+ |                                 |
| | 14  | SONAR         | |                                 |
| +-----+---------------+ |                                 |
| | 15  | ARM/DISARM    | |                                 |
| +-----+---------------+ |                                 |
| | 19  | IMU_RAW       | |                                 |
| +-----+---------------+ |                                 |
|                         |                                 |
+-------------------------+---------------------------------+




.. _RST_SWITCH_CH:

RST\_SWITCH\_CH: Reset Switch Channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC channel to use to reset to last flight mode	after geofence takeover\.


.. _RST_MISSION_CH:

RST\_MISSION\_CH: Reset Mission Channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC channel to use to reset the mission to the first waypoint\. When this channel goes above 1750 the mission is reset\. Set RST\_MISSION\_CH to 0 to disable\.


.. _TRIM_ARSPD_CM:

TRIM\_ARSPD\_CM: Target airspeed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Airspeed in cm\/s to aim for when airspeed is enabled in auto mode\. This is a calibrated \(apparent\) airspeed\.


+-------+
| Units |
+=======+
| cm/s  |
+-------+




.. _SCALING_SPEED:

SCALING\_SPEED: speed used for speed scaling calculations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed in m\/s to use when calculating surface speed scaling\. Note that changing this value will affect all PID values


+-------+
| Units |
+=======+
| m/s   |
+-------+




.. _MIN_GNDSPD_CM:

MIN\_GNDSPD\_CM: Minimum ground speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum ground speed in cm\/s when under airspeed control


+-------+
| Units |
+=======+
| cm/s  |
+-------+




.. _TRIM_PITCH_CD:

TRIM\_PITCH\_CD: Pitch angle offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

offset to add to pitch \- used for in\-flight pitch trimming\. It is recommended that instead of using this parameter you level your plane correctly on the ground for good flight attitude\.


+---------------+
| Units         |
+===============+
| centi-Degrees |
+---------------+




.. _ALT_HOLD_RTL:

ALT\_HOLD\_RTL: RTL altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Return to launch target altitude\. This is the relative altitude the plane will aim for and loiter at when returning home\. If this is negative \(usually \-1\) then the plane will use the current altitude at the time of entering RTL\. Note that when transiting to a Rally Point the altitude of the Rally Point is used instead of ALT\_HOLD\_RTL\.


+-------------+
| Units       |
+=============+
| centimeters |
+-------------+




.. _ALT_HOLD_FBWCM:

ALT\_HOLD\_FBWCM: Minimum altitude for FBWB mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the minimum altitude in centimeters that FBWB and CRUISE modes will allow\. If you attempt to descend below this altitude then the plane will level off\. A value of zero means no limit\.


+-------------+
| Units       |
+=============+
| centimeters |
+-------------+




.. _MAG_ENABLE:

MAG\_ENABLE: Enable Compass
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Enabled\(1\) will enable the compass\. Setting this to Disabled\(0\) will disable the compass\. Note that this is separate from COMPASS\_USE\. This will enable the low level senor\, and will enable logging of magnetometer data\. To use the compass for navigation you must also set COMPASS\_USE to 1\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _FLAP_IN_CHANNEL:

FLAP\_IN\_CHANNEL: Flap input channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


An RC input channel to use for flaps control\. If this is set to a RC channel number then that channel will be used for manual flaps control\. When enabled\, the percentage of flaps is taken as the percentage travel from the TRIM value of the channel to the MIN value of the channel\. A value above the TRIM values will give inverse flaps \(spoilers\)\. This option needs to be enabled in conjunction with a FUNCTION setting on an output channel to one of the flap functions\. When a FLAP\_IN\_CHANNEL is combined with auto\-flaps the higher of the two flap percentages is taken\. You must also enable a FLAPERON\_OUTPUT flaperon mixer setting if using flaperons\.


.. _FLAPERON_OUTPUT:

FLAPERON\_OUTPUT: Flaperon output
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable flaperon output in software\. If enabled then the APM will provide software flaperon mixing on the FLAPERON1 and FLAPERON2 output channels specified using the FUNCTION on two auxiliary channels\. There are 4 different mixing modes available\, which refer to the 4 ways the flap and aileron outputs can be mapped to the two flaperon servos\. Note that you must not use flaperon output mixing with hardware pass\-through of RC values\, such as with channel 8 manual control on an APM1\. So if you use an APM1 then set FLTMODE\_CH to something other than 8 before you enable FLAPERON\_OUTPUT\. Please also see the MIXING\_GAIN parameter for the output gain of the mixer\. FLAPERON\_OUTPUT cannot be combined with ELEVON\_OUTPUT or ELEVON\_MIXING\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | UpUp     | |
| +-------+----------+ |
| | 2     | UpDown   | |
| +-------+----------+ |
| | 3     | DownUp   | |
| +-------+----------+ |
| | 4     | DownDown | |
| +-------+----------+ |
|                      |
+----------------------+




.. _FLAP_1_PERCNT:

FLAP\_1\_PERCNT: Flap 1 percentage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The percentage change in flap position when FLAP\_1\_SPEED is reached\. Use zero to disable flaps


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 100 | Percent |
+---------+---------+




.. _FLAP_1_SPEED:

FLAP\_1\_SPEED: Flap 1 speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The speed in meters per second at which to engage FLAP\_1\_PERCENT of flaps\. Note that FLAP\_1\_SPEED should be greater than or equal to FLAP\_2\_SPEED


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 100 | 1         | m/s   |
+---------+-----------+-------+




.. _FLAP_2_PERCNT:

FLAP\_2\_PERCNT: Flap 2 percentage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The percentage change in flap position when FLAP\_2\_SPEED is reached\. Use zero to disable flaps


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 100 | Percent |
+---------+---------+




.. _FLAP_2_SPEED:

FLAP\_2\_SPEED: Flap 2 speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The speed in meters per second at which to engage FLAP\_2\_PERCENT of flaps\. Note that FLAP\_1\_SPEED should be greater than or equal to FLAP\_2\_SPEED


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 100 | 1         | m/s   |
+---------+-----------+-------+




.. _LAND_FLAP_PERCNT:

LAND\_FLAP\_PERCNT: Landing flap percentage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The amount of flaps \(as a percentage\) to apply in the landing approach and flare of an automatic landing


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 100 | Percent |
+---------+---------+




.. _OVERRIDE_CHAN:

OVERRIDE\_CHAN: PX4IO override channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If set to a non\-zero value then this is an RC input channel number to use for giving PX4IO manual control in case the main FMU microcontroller on a PX4 or Pixhawk fails\. When this RC input channel goes above 1750 the FMU microcontroller will no longer be involved in controlling the servos and instead the PX4IO microcontroller will directly control the servos\. Note that PX4IO manual control will be automatically activated if the FMU crashes for any reason\. This parameter allows you to test for correct manual behaviour without actually crashing the FMU\. This parameter is can be set to a non\-zero value either for ground testing purposes or for giving the effect of an external override control board\. Please also see the docs on OVERRIDE\_SAFETY\. Note that you may set OVERRIDE\_CHAN to the same channel as FLTMODE\_CH to get PX4IO based override when in flight mode 6\. Note that when override is triggered due to a FMU crash the 6 auxiliary output channels on Pixhawk will no longer be updated\, so all the flight controls you need must be assigned to the first 8 channels\.


.. _OVERRIDE_SAFETY:

OVERRIDE\_SAFETY: PX4IO override safety switch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls whether the safety switch is turned off when you activate override with OVERRIDE\_CHAN\. When set to 1 the safety switch is de\-activated \(activating the servos\) then a PX4IO override is triggered\. In that case the safety remains de\-activated after override is disabled\. If OVERRIDE\_SAFETTY is set to 0 then the safety switch state does not change\. Note that regardless of the value of this parameter the servos will be active while override is active\.


.. _INVERTEDFLT_CH:

INVERTEDFLT\_CH: Inverted flight channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


A RC input channel number to enable inverted flight\. If this is non\-zero then the APM will monitor the corresponding RC input channel and will enable inverted flight when the channel goes above 1750\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Channel1 | |
| +-------+----------+ |
| | 2     | Channel2 | |
| +-------+----------+ |
| | 3     | Channel3 | |
| +-------+----------+ |
| | 4     | Channel4 | |
| +-------+----------+ |
| | 5     | Channel5 | |
| +-------+----------+ |
| | 6     | Channel6 | |
| +-------+----------+ |
| | 7     | Channel7 | |
| +-------+----------+ |
| | 8     | Channel8 | |
| +-------+----------+ |
|                      |
+----------------------+




.. _HIL_MODE:

HIL\_MODE: HIL mode enable
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables and disables hardware in the loop mode\. If HIL\_MODE is 1 then on the next reboot all sensors are replaced with HIL sensors which come from the GCS\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _HIL_SERVOS:

HIL\_SERVOS: HIL Servos enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls whether real servo controls are used in HIL mode\. If you enable this then the APM will control the real servos in HIL mode\. If disabled it will report servo values\, but will not output to the real servos\. Be careful that your motor and propeller are not connected if you enable this option\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _HIL_ERR_LIMIT:

HIL\_ERR\_LIMIT: Limit of error in HIL attitude before reset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the maximum error in degrees on any axis before HIL will reset the DCM attitude to match the HIL\_STATE attitude\. This limit will prevent poor timing on HIL from causing a major attitude error\. If the value is zero then no limit applies\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 90 | 0.1       | degrees |
+--------+-----------+---------+




.. _RTL_AUTOLAND:

RTL\_AUTOLAND: RTL auto land
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Automatically begin landing sequence after arriving at RTL location\. This requires the addition of a DO\_LAND\_START mission item\, which acts as a marker for the start of a landing sequence\. The closest landing sequence will be chosen to the current location\. 


+------------------------------------------------------+
| Values                                               |
+======================================================+
| +-------+------------------------------------------+ |
| | Value | Meaning                                  | |
| +=======+==========================================+ |
| | 0     | Disable                                  | |
| +-------+------------------------------------------+ |
| | 1     | Enable - go HOME then land               | |
| +-------+------------------------------------------+ |
| | 2     | Enable - go directly to landing sequence | |
| +-------+------------------------------------------+ |
|                                                      |
+------------------------------------------------------+




.. _RC_TRIM_AT_START:

RC\_TRIM\_AT\_START: RC Trims auto set at start\.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Automatically set roll\/pitch trim from Tx at ground start\. This makes the assumption that the RC transmitter has not been altered since trims were last captured\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _CRASH_ACC_THRESH:

CRASH\_ACC\_THRESH: Crash Deceleration Threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X\-Axis deceleration threshold to notify the crash detector that there was a possible impact which helps disarm the motor quickly after a crash\. This value should be much higher than normal negative x\-axis forces during normal flight\, check flight log files to determine the average IMU\.x values for your aircraft and motor type\. Higher value means less sensative \(triggers on higher impact\)\. For electric planes that don\'t vibrate much during fight a value of 25 is good \(that\'s about 2\.5G\)\. For petrol\/nitro planes you\'ll want a higher value\. Set to 0 to disable the collision detector\.


+--------+-------+
| Values | Units |
+========+=======+
| 10 127 | m/s/s |
+--------+-------+




.. _CRASH_DETECT:

CRASH\_DETECT: Crash Detection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Automatically detect a crash during AUTO flight and perform the bitmask selected action\(s\)\. Disarm will turn off motor for safety and to help against burning out ESC and motor\. Setting the mode to manual will help save the servos from burning out by overexerting if the aircraft crashed in an odd orientation such as upsidedown\.


+-------------------+----------------------+
| Bitmask           | Values               |
+===================+======================+
| +-----+---------+ | +-------+----------+ |
| | Bit | Meaning | | | Value | Meaning  | |
| +=====+=========+ | +=======+==========+ |
| | 0   | Disarm  | | | 0     | Disabled | |
| +-----+---------+ | +-------+----------+ |
|                   | | 1     | Disarm   | |
|                   | +-------+----------+ |
|                   |                      |
+-------------------+----------------------+




.. _CHUTE_CHAN:

CHUTE\_CHAN: Parachute release channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If set to a non\-zero value then this is an RC input channel number to use for manually releasing the parachute\. When this channel goes above 1700 the parachute will be released


.. _RNGFND_LANDING:

RNGFND\_LANDING: Enable rangefinder for landing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This enables the use of a rangefinder for automatic landing\. The rangefinder will be used both on the landing approach and for final flare


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+





.. _parameters_SERIAL:

SERIAL Parameters
-----------------


.. _SERIAL0_BAUD:

SERIAL0\_BAUD: Serial0 baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used on the USB console\. The APM2 can support all baudrates up to 115\, and also can support 500\. The PX4 can support rates of up to 1500\. If you setup a rate you cannot support on APM2 and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL0_PROTOCOL:

SERIAL0\_PROTOCOL: Console protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control what protocol to use on the console\. 


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 1     | MAVlink1 | |
| +-------+----------+ |
| | 2     | MAVLink2 | |
| +-------+----------+ |
|                      |
+----------------------+




.. _SERIAL1_PROTOCOL:

SERIAL1\_PROTOCOL: Telem1 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control what protocol to use on the Telem1 port\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | None                  | |
| +-------+-----------------------+ |
| | 1     | MAVlink1              | |
| +-------+-----------------------+ |
| | 2     | MAVLink2              | |
| +-------+-----------------------+ |
| | 3     | Frsky D-PORT          | |
| +-------+-----------------------+ |
| | 4     | Frsky S-PORT          | |
| +-------+-----------------------+ |
| | 5     | GPS                   | |
| +-------+-----------------------+ |
| | 7     | Alexmos Gimbal Serial | |
| +-------+-----------------------+ |
| | 8     | SToRM32 Gimbal Serial | |
| +-------+-----------------------+ |
| | 9     | Lidar                 | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _SERIAL1_BAUD:

SERIAL1\_BAUD: Telem1 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used on the Telem1 port\. The APM2 can support all baudrates up to 115\, and also can support 500\. The PX4 can support rates of up to 1500\. If you setup a rate you cannot support on APM2 and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL2_PROTOCOL:

SERIAL2\_PROTOCOL: Telemetry 2 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control what protocol to use on the Telem2 port\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | None                  | |
| +-------+-----------------------+ |
| | 1     | MAVlink1              | |
| +-------+-----------------------+ |
| | 2     | MAVLink2              | |
| +-------+-----------------------+ |
| | 3     | Frsky D-PORT          | |
| +-------+-----------------------+ |
| | 4     | Frsky S-PORT          | |
| +-------+-----------------------+ |
| | 5     | GPS                   | |
| +-------+-----------------------+ |
| | 7     | Alexmos Gimbal Serial | |
| +-------+-----------------------+ |
| | 8     | SToRM32 Gimbal Serial | |
| +-------+-----------------------+ |
| | 9     | Lidar                 | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _SERIAL2_BAUD:

SERIAL2\_BAUD: Telemetry 2 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate of the Telem2 port\. The APM2 can support all baudrates up to 115\, and also can support 500\. The PX4 can support rates of up to 1500\. If you setup a rate you cannot support on APM2 and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL3_PROTOCOL:

SERIAL3\_PROTOCOL: Serial 3 \(GPS\) protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control what protocol Serial 3 \(GPS\) should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | None                  | |
| +-------+-----------------------+ |
| | 1     | MAVlink1              | |
| +-------+-----------------------+ |
| | 2     | MAVLink2              | |
| +-------+-----------------------+ |
| | 3     | Frsky D-PORT          | |
| +-------+-----------------------+ |
| | 4     | Frsky S-PORT          | |
| +-------+-----------------------+ |
| | 5     | GPS                   | |
| +-------+-----------------------+ |
| | 7     | Alexmos Gimbal Serial | |
| +-------+-----------------------+ |
| | 8     | SToRM32 Gimbal Serial | |
| +-------+-----------------------+ |
| | 9     | Lidar                 | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _SERIAL3_BAUD:

SERIAL3\_BAUD: Serial 3 \(GPS\) Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for the Serial 3 \(GPS\)\. The APM2 can support all baudrates up to 115\, and also can support 500\. The PX4 can support rates of up to 1500\. If you setup a rate you cannot support on APM2 and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL4_PROTOCOL:

SERIAL4\_PROTOCOL: Serial4 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control what protocol Serial4 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | None                  | |
| +-------+-----------------------+ |
| | 1     | MAVlink1              | |
| +-------+-----------------------+ |
| | 2     | MAVLink2              | |
| +-------+-----------------------+ |
| | 3     | Frsky D-PORT          | |
| +-------+-----------------------+ |
| | 4     | Frsky S-PORT          | |
| +-------+-----------------------+ |
| | 5     | GPS                   | |
| +-------+-----------------------+ |
| | 7     | Alexmos Gimbal Serial | |
| +-------+-----------------------+ |
| | 8     | SToRM32 Gimbal Serial | |
| +-------+-----------------------+ |
| | 9     | Lidar                 | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _SERIAL4_BAUD:

SERIAL4\_BAUD: Serial 4 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial4\. The APM2 can support all baudrates up to 115\, and also can support 500\. The PX4 can support rates of up to 1500\. If you setup a rate you cannot support on APM2 and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+




.. _SERIAL5_PROTOCOL:

SERIAL5\_PROTOCOL: Serial5 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control what protocol Serial5 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | None                  | |
| +-------+-----------------------+ |
| | 1     | MAVlink1              | |
| +-------+-----------------------+ |
| | 2     | MAVLink2              | |
| +-------+-----------------------+ |
| | 3     | Frsky D-PORT          | |
| +-------+-----------------------+ |
| | 4     | Frsky S-PORT          | |
| +-------+-----------------------+ |
| | 5     | GPS                   | |
| +-------+-----------------------+ |
| | 7     | Alexmos Gimbal Serial | |
| +-------+-----------------------+ |
| | 8     | SToRM32 Gimbal Serial | |
| +-------+-----------------------+ |
| | 9     | Lidar                 | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _SERIAL5_BAUD:

SERIAL5\_BAUD: Serial 5 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial5\. The APM2 can support all baudrates up to 115\, and also can support 500\. The PX4 can support rates of up to 1500\. If you setup a rate you cannot support on APM2 and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | 1200    | |
| +-------+---------+ |
| | 2     | 2400    | |
| +-------+---------+ |
| | 4     | 4800    | |
| +-------+---------+ |
| | 9     | 9600    | |
| +-------+---------+ |
| | 19    | 19200   | |
| +-------+---------+ |
| | 38    | 38400   | |
| +-------+---------+ |
| | 57    | 57600   | |
| +-------+---------+ |
| | 111   | 111100  | |
| +-------+---------+ |
| | 115   | 115200  | |
| +-------+---------+ |
| | 500   | 500000  | |
| +-------+---------+ |
| | 921   | 921600  | |
| +-------+---------+ |
| | 1500  | 1500000 | |
| +-------+---------+ |
|                     |
+---------------------+





.. _parameters_GND_:

GND\_ Parameters
----------------


.. _GND_ABS_PRESS:

GND\_ABS\_PRESS: Absolute Pressure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


calibrated ground pressure in Pascals


+----------+----------+-----------+---------+
| ReadOnly | Volatile | Increment | Units   |
+==========+==========+===========+=========+
| True     | True     | 1         | pascals |
+----------+----------+-----------+---------+




.. _GND_TEMP:

GND\_TEMP: ground temperature
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


calibrated ground temperature in degrees Celsius


+----------+----------+-----------+-----------------+
| ReadOnly | Volatile | Increment | Units           |
+==========+==========+===========+=================+
| True     | True     | 1         | degrees celsius |
+----------+----------+-----------+-----------------+




.. _GND_ALT_OFFSET:

GND\_ALT\_OFFSET: altitude offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


altitude offset in meters added to barometric altitude\. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer\. The value is added to the barometric altitude read by the aircraft\. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed\.


+--------+-----------+
| Units  | Increment |
+========+===========+
| meters | 0.1       |
+--------+-----------+




.. _GND_PRIMARY:

GND\_PRIMARY: Primary barometer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This selects which barometer will be the primary if multiple barometers are found


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | FirstBaro | |
| +-------+-----------+ |
| | 1     | 2ndBaro   | |
| +-------+-----------+ |
| | 2     | 3rdBaro   | |
| +-------+-----------+ |
|                       |
+-----------------------+





.. _parameters_GPS_:

GPS\_ Parameters
----------------


.. _GPS_TYPE:

GPS\_TYPE: GPS type
~~~~~~~~~~~~~~~~~~~


GPS type


+------------------------+----------------+
| Values                 | RebootRequired |
+========================+================+
| +-------+------------+ | True           |
| | Value | Meaning    | |                |
| +=======+============+ |                |
| | 0     | None       | |                |
| +-------+------------+ |                |
| | 1     | AUTO       | |                |
| +-------+------------+ |                |
| | 2     | uBlox      | |                |
| +-------+------------+ |                |
| | 3     | MTK        | |                |
| +-------+------------+ |                |
| | 4     | MTK19      | |                |
| +-------+------------+ |                |
| | 5     | NMEA       | |                |
| +-------+------------+ |                |
| | 6     | SiRF       | |                |
| +-------+------------+ |                |
| | 7     | HIL        | |                |
| +-------+------------+ |                |
| | 8     | SwiftNav   | |                |
| +-------+------------+ |                |
| | 9     | PX4-UAVCAN | |                |
| +-------+------------+ |                |
| | 10    | SBF        | |                |
| +-------+------------+ |                |
| | 11    | GSOF       | |                |
| +-------+------------+ |                |
|                        |                |
+------------------------+----------------+




.. _GPS_TYPE2:

GPS\_TYPE2: 2nd GPS type
~~~~~~~~~~~~~~~~~~~~~~~~


GPS type of 2nd GPS


+------------------------+----------------+
| Values                 | RebootRequired |
+========================+================+
| +-------+------------+ | True           |
| | Value | Meaning    | |                |
| +=======+============+ |                |
| | 0     | None       | |                |
| +-------+------------+ |                |
| | 1     | AUTO       | |                |
| +-------+------------+ |                |
| | 2     | uBlox      | |                |
| +-------+------------+ |                |
| | 3     | MTK        | |                |
| +-------+------------+ |                |
| | 4     | MTK19      | |                |
| +-------+------------+ |                |
| | 5     | NMEA       | |                |
| +-------+------------+ |                |
| | 6     | SiRF       | |                |
| +-------+------------+ |                |
| | 7     | HIL        | |                |
| +-------+------------+ |                |
| | 8     | SwiftNav   | |                |
| +-------+------------+ |                |
| | 9     | PX4-UAVCAN | |                |
| +-------+------------+ |                |
| | 10    | SBF        | |                |
| +-------+------------+ |                |
| | 11    | GSOF       | |                |
| +-------+------------+ |                |
|                        |                |
+------------------------+----------------+




.. _GPS_NAVFILTER:

GPS\_NAVFILTER: Navigation filter setting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Navigation filter engine setting


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Portable   | |
| +-------+------------+ |
| | 2     | Stationary | |
| +-------+------------+ |
| | 3     | Pedestrian | |
| +-------+------------+ |
| | 4     | Automotive | |
| +-------+------------+ |
| | 5     | Sea        | |
| +-------+------------+ |
| | 6     | Airborne1G | |
| +-------+------------+ |
| | 7     | Airborne2G | |
| +-------+------------+ |
| | 8     | Airborne4G | |
| +-------+------------+ |
|                        |
+------------------------+




.. _GPS_AUTO_SWITCH:

GPS\_AUTO\_SWITCH: Automatic Switchover Setting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Automatic switchover to GPS reporting best lock


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _GPS_MIN_DGPS:

GPS\_MIN\_DGPS: Minimum Lock Type Accepted for DGPS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Sets the minimum type of differential GPS corrections required before allowing to switch into DGPS mode\.


+------------------------+----------------+
| Values                 | RebootRequired |
+========================+================+
| +-------+------------+ | True           |
| | Value | Meaning    | |                |
| +=======+============+ |                |
| | 0     | Any        | |                |
| +-------+------------+ |                |
| | 50    | FloatRTK   | |                |
| +-------+------------+ |                |
| | 100   | IntegerRTK | |                |
| +-------+------------+ |                |
|                        |                |
+------------------------+----------------+




.. _GPS_SBAS_MODE:

GPS\_SBAS\_MODE: SBAS Mode
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the SBAS \(satellite based augmentation system\) mode if available on this GPS\. If set to 2 then the SBAS mode is not changed in the GPS\. Otherwise the GPS will be reconfigured to enable\/disable SBAS\. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
| | 2     | NoChange | |
| +-------+----------+ |
|                      |
+----------------------+




.. _GPS_MIN_ELEV:

GPS\_MIN\_ELEV: Minimum elevation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the minimum elevation of satellites above the horizon for them to be used for navigation\. Setting this to \-100 leaves the minimum elevation set to the GPS modules default\.


+-----------+---------+
| Range     | Units   |
+===========+=========+
| -100 - 90 | Degrees |
+-----------+---------+




.. _GPS_INJECT_TO:

GPS\_INJECT\_TO: Destination for GPS\_INJECT\_DATA MAVLink packets
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The GGS can send raw serial packets to inject data to multiple GPSes\.


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | send to first GPS | |
| +-------+-------------------+ |
| | 1     | send to 2nd GPS   | |
| +-------+-------------------+ |
| | 127   | send to all       | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _GPS_SBP_LOGMASK:

GPS\_SBP\_LOGMASK: Swift Binary Protocol Logging Mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Masked with the SBP msg\_type field to determine whether SBR1\/SBR2 data is logged


+----------------------------+
| Values                     |
+============================+
| +--------+---------------+ |
| | Value  | Meaning       | |
| +========+===============+ |
| | 0x0000 | None          | |
| +--------+---------------+ |
| | 0xFFFF | All           | |
| +--------+---------------+ |
| | 0xFF00 | External only | |
| +--------+---------------+ |
|                            |
+----------------------------+




.. _GPS_RAW_DATA:

GPS\_RAW\_DATA: Raw data logging
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable logging of RXM raw data from uBlox which includes carrier phase and pseudo range information\. This allows for post processing of dataflash logs for more precise positioning\. Note that this requires a raw capable uBlox such as the 6P or 6T\.


+---------------------------------+----------------+
| Values                          | RebootRequired |
+=================================+================+
| +-------+---------------------+ | True           |
| | Value | Meaning             | |                |
| +=======+=====================+ |                |
| | 0     | Disabled            | |                |
| +-------+---------------------+ |                |
| | 1     | log every sample    | |                |
| +-------+---------------------+ |                |
| | 5     | log every 5 samples | |                |
| +-------+---------------------+ |                |
|                                 |                |
+---------------------------------+----------------+




.. _GPS_GNSS_MODE:

GPS\_GNSS\_MODE: GNSS system configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask for what GNSS system to use on the first GPS \(all unchecked or zero to leave GPS as configured\)


+--------------------+---------------------------------------------+
| Bitmask            | Values                                      |
+====================+=============================================+
| +-----+----------+ | +-------+---------------------------------+ |
| | Bit | Meaning  | | | Value | Meaning                         | |
| +=====+==========+ | +=======+=================================+ |
| | 0   | GPS      | | | 0     | Leave as currently configured   | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 1   | SBAS     | | | 1     | GPS-NoSBAS                      | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 2   | Galileo  | | | 3     | GPS+SBAS                        | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 3   | Beidou   | | | 4     | Galileo-NoSBAS                  | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 4   | IMES     | | | 6     | Galileo+SBAS                    | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 5   | QZSS     | | | 8     | Beidou                          | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 6   | GLOSNASS | | | 51    | GPS+IMES+QZSS+SBAS (Japan Only) | |
| +-----+----------+ | +-------+---------------------------------+ |
|                    | | 64    | GLONASS                         | |
|                    | +-------+---------------------------------+ |
|                    | | 66    | GLONASS+SBAS                    | |
|                    | +-------+---------------------------------+ |
|                    | | 67    | GPS+GLONASS+SBAS                | |
|                    | +-------+---------------------------------+ |
|                    |                                             |
+--------------------+---------------------------------------------+




.. _GPS_SAVE_CFG:

GPS\_SAVE\_CFG: Save GPS configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Determines whether the configuration for this GPS should be written to non\-volatile memory on the GPS\. Currently working for UBlox 6 series and above\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | 0     | Do not save config    | |
| +-------+-----------------------+ |
| | 1     | Save config           | |
| +-------+-----------------------+ |
| | 2     | Save only when needed | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _GPS_GNSS_MODE2:

GPS\_GNSS\_MODE2: GNSS system configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask for what GNSS system to use on the second GPS \(all unchecked or zero to leave GPS as configured\)


+--------------------+---------------------------------------------+
| Bitmask            | Values                                      |
+====================+=============================================+
| +-----+----------+ | +-------+---------------------------------+ |
| | Bit | Meaning  | | | Value | Meaning                         | |
| +=====+==========+ | +=======+=================================+ |
| | 0   | GPS      | | | 0     | Leave as currently configured   | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 1   | SBAS     | | | 1     | GPS-NoSBAS                      | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 2   | Galileo  | | | 3     | GPS+SBAS                        | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 3   | Beidou   | | | 4     | Galileo-NoSBAS                  | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 4   | IMES     | | | 6     | Galileo+SBAS                    | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 5   | QZSS     | | | 8     | Beidou                          | |
| +-----+----------+ | +-------+---------------------------------+ |
| | 6   | GLOSNASS | | | 51    | GPS+IMES+QZSS+SBAS (Japan Only) | |
| +-----+----------+ | +-------+---------------------------------+ |
|                    | | 64    | GLONASS                         | |
|                    | +-------+---------------------------------+ |
|                    | | 66    | GLONASS+SBAS                    | |
|                    | +-------+---------------------------------+ |
|                    | | 67    | GPS+GLONASS+SBAS                | |
|                    | +-------+---------------------------------+ |
|                    |                                             |
+--------------------+---------------------------------------------+




.. _GPS_AUTO_CONFIG:

GPS\_AUTO\_CONFIG: Automatic GPS configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls if the autopilot should automatically configure the GPS based on the parameters and default settings


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | 0     | Disables automatic configuration | |
| +-------+----------------------------------+ |
| | 1     | Enable automatic configuration   | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+





.. _parameters_CAM_:

CAM\_ Parameters
----------------


.. _CAM_TRIGG_TYPE:

CAM\_TRIGG\_TYPE: Camera shutter \(trigger\) type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


how to trigger the camera to take a picture


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Servo   | |
| +-------+---------+ |
| | 1     | Relay   | |
| +-------+---------+ |
|                     |
+---------------------+




.. _CAM_DURATION:

CAM\_DURATION: Duration that shutter is held open
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


How long the shutter will be held open in 10ths of a second \(i\.e\. enter 10 for 1second\, 50 for 5seconds\)


+--------+---------+
| Range  | Units   |
+========+=========+
| 0 - 50 | seconds |
+--------+---------+




.. _CAM_SERVO_ON:

CAM\_SERVO\_ON: Servo ON PWM value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


PWM value to move servo to when shutter is activated


+-------------+-------+
| Range       | Units |
+=============+=======+
| 1000 - 2000 | pwm   |
+-------------+-------+




.. _CAM_SERVO_OFF:

CAM\_SERVO\_OFF: Servo OFF PWM value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


PWM value to move servo to when shutter is deactivated


+-------------+-------+
| Range       | Units |
+=============+=======+
| 1000 - 2000 | pwm   |
+-------------+-------+




.. _CAM_TRIGG_DIST:

CAM\_TRIGG\_DIST: Camera trigger distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Distance in meters between camera triggers\. If this value is non\-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in\. Note that this parameter can also be set in an auto mission using the DO\_SET\_CAM\_TRIGG\_DIST command\, allowing you to enable\/disable the triggering of the camera during the flight\.


+----------+--------+
| Range    | Units  |
+==========+========+
| 0 - 1000 | meters |
+----------+--------+




.. _CAM_RELAY_ON:

CAM\_RELAY\_ON: Relay ON value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets whether the relay goes high or low when it triggers\. Note that you should also set RELAY\_DEFAULT appropriately for your camera


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Low     | |
| +-------+---------+ |
| | 1     | High    | |
| +-------+---------+ |
|                     |
+---------------------+




.. _CAM_MIN_INTERVAL:

CAM\_MIN\_INTERVAL: Minimum time between photos
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Postpone shooting if previous picture was taken less than preset time\(ms\) ago\.


+-----------+--------------+
| Range     | Units        |
+===========+==============+
| 0 - 10000 | milliseconds |
+-----------+--------------+




.. _CAM_MAX_ROLL:

CAM\_MAX\_ROLL: Maximum photo roll angle\.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Postpone shooting if roll is greater than limit\. \(0\=Disable\, will shoot regardless of roll\)\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 180 | Degrees |
+---------+---------+




.. _CAM_FEEDBACK_PIN:

CAM\_FEEDBACK\_PIN: Camera feedback pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


pin number to use for save accurate camera feedback messages\. If set to \-1 then don\'t use a pin flag for this\, otherwise this is a pin number which if held high after a picture trigger order\, will save camera messages when camera really takes a picture\. A universal camera hot shoe is needed\. The pin should be held high for at least 2 milliseconds for reliable trigger detection\. See also the CAM\_FEEDBACK\_POL option\. If using AUX4 pin on a Pixhawk then a fast capture method is used that allows for the trigger time to be as short as one microsecond\.


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | -1    | Disabled               | |
| +-------+------------------------+ |
| | 50    | PX4 AUX1               | |
| +-------+------------------------+ |
| | 51    | PX4 AUX2               | |
| +-------+------------------------+ |
| | 52    | PX4 AUX3               | |
| +-------+------------------------+ |
| | 53    | PX4 AUX4(fast capture) | |
| +-------+------------------------+ |
| | 54    | PX4 AUX5               | |
| +-------+------------------------+ |
| | 55    | PX4 AUX6               | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _CAM_FEEDBACK_POL:

CAM\_FEEDBACK\_POL: Camera feedback pin polarity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Polarity for feedback pin\. If this is 1 then the feedback pin should go high on trigger\. If set to 0 then it should go low


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | TriggerLow  | |
| +-------+-------------+ |
| | 1     | TriggerHigh | |
| +-------+-------------+ |
|                         |
+-------------------------+





.. _parameters_ARMING_:

ARMING\_ Parameters
-------------------


.. _ARMING_RUDDER:

ARMING\_RUDDER: Rudder Arming
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Control arm\/disarm by rudder input\. When enabled arming is done with right rudder\, disarming with left rudder\. Rudder arming only works in manual throttle modes with throttle at zero \+\- deadzone \(RCx\_DZ\)


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | Disabled    | |
| +-------+-------------+ |
| | 1     | ArmingOnly  | |
| +-------+-------------+ |
| | 2     | ArmOrDisarm | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _ARMING_REQUIRE:

ARMING\_REQUIRE: Require Arming Motors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Arming disabled until some requirements are met\. If 0\, there are no requirements \(arm immediately\)\.  If 1\, require rudder stick or GCS arming before arming motors and send THR\_MIN PWM to throttle channel when disarmed\.  If 2\, require rudder stick or GCS arming and send 0 PWM to throttle channel when disarmed\. See the ARMING\_CHECK\_\* parameters to see what checks are done before arming\. Note\, if setting this parameter to 0 a reboot is required to arm the plane\.  Also note\, even with this parameter at 0\, if ARMING\_CHECK parameter is not also zero the plane may fail to arm throttle at boot due to a pre\-arm check failure\.


+---------------------------------------+
| Values                                |
+=======================================+
| +-------+---------------------------+ |
| | Value | Meaning                   | |
| +=======+===========================+ |
| | 0     | Disabled                  | |
| +-------+---------------------------+ |
| | 1     | THR_MIN PWM when disarmed | |
| +-------+---------------------------+ |
| | 2     | 0 PWM when disarmed       | |
| +-------+---------------------------+ |
|                                       |
+---------------------------------------+




.. _ARMING_CHECK:

ARMING\_CHECK: Arm Checks to Peform \(bitmask\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Checks prior to arming motor\. This is a bitmask of checks that will be performed befor allowing arming\. The default is no checks\, allowing arming at any time\. You can select whatever checks you prefer by adding together the values of each check type to set this parameter\. For example\, to only allow arming when you have GPS lock and no RC failsafe you would set ARMING\_CHECK to 72\. For most users it is recommended that you set this to 1 to enable all checks\.


+----------------------------------+----------------------------------------------------+
| Bitmask                          | Values                                             |
+==================================+====================================================+
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | Bit | Meaning                | | | Value | Meaning                                | |
| +=====+========================+ | +=======+========================================+ |
| | 0   | All                    | | | 0     | None                                   | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 1   | Barometer              | | | 1     | All                                    | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 2   | Compass                | | | 2     | Barometer                              | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 3   | GPS lock               | | | 4     | Compass                                | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 4   | INS                    | | | 8     | GPS Lock                               | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 5   | Parameters             | | | 16    | INS(INertial Sensors - accels & gyros) | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 6   | RC                     | | | 32    | Parameters(unused)                     | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 7   | Board voltage          | | | 64    | RC Failsafe                            | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 8   | Battery Level          | | | 128   | Board voltage                          | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 9   | Airspeed               | | | 256   | Battery Level                          | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 10  | Logging Available      | | | 512   | Airspeed                               | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 11  | Hardware safety switch | | | 1024  | LoggingAvailable                       | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
| | 12  | GPS Configuration      | | | 2048  | Hardware safety switch                 | |
| +-----+------------------------+ | +-------+----------------------------------------+ |
|                                  | | 4096  | GPS configuration                      | |
|                                  | +-------+----------------------------------------+ |
|                                  |                                                    |
+----------------------------------+----------------------------------------------------+




.. _ARMING_ACCTHRESH:

ARMING\_ACCTHRESH: Accelerometer error threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer error threshold used to determine inconsistent accelerometers\. Compares this error range to other accelerometers to detect a hardware or calibration error\. Lower value means tighter check and harder to pass arming check\. Not all accelerometers are created equal\.


+------------+-------+
| Range      | Units |
+============+=======+
| 0.25 - 3.0 | m/s/s |
+------------+-------+




.. _ARMING_MIN_VOLT:

ARMING\_MIN\_VOLT: Minimum arming voltage on the first battery
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The minimum voltage on the first battery to arm\, 0 disabes the check


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | Volts |
+-----------+-------+




.. _ARMING_MIN_VOLT2:

ARMING\_MIN\_VOLT2: Minimum arming voltage on the second battery
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The minimum voltage on the first battery to arm\, 0 disabes the check


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | Volts |
+-----------+-------+





.. _parameters_RELAY_:

RELAY\_ Parameters
------------------


.. _RELAY_PIN:

RELAY\_PIN: First Relay Pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin number for first relay control\. This is the pin used for camera control\.


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | -1    | Disabled        | |
| +-------+-----------------+ |
| | 13    | APM2 A9 pin     | |
| +-------+-----------------+ |
| | 47    | APM1 relay      | |
| +-------+-----------------+ |
| | 50    | Pixhawk AUXOUT1 | |
| +-------+-----------------+ |
| | 51    | Pixhawk AUXOUT2 | |
| +-------+-----------------+ |
| | 52    | Pixhawk AUXOUT3 | |
| +-------+-----------------+ |
| | 53    | Pixhawk AUXOUT4 | |
| +-------+-----------------+ |
| | 54    | Pixhawk AUXOUT5 | |
| +-------+-----------------+ |
| | 55    | Pixhawk AUXOUT6 | |
| +-------+-----------------+ |
| | 111   | PX4 FMU Relay1  | |
| +-------+-----------------+ |
| | 112   | PX4 FMU Relay2  | |
| +-------+-----------------+ |
| | 113   | PX4IO Relay1    | |
| +-------+-----------------+ |
| | 114   | PX4IO Relay2    | |
| +-------+-----------------+ |
| | 115   | PX4IO ACC1      | |
| +-------+-----------------+ |
| | 116   | PX4IO ACC2      | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RELAY_PIN2:

RELAY\_PIN2: Second Relay Pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin number for 2nd relay control\.


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | -1    | Disabled        | |
| +-------+-----------------+ |
| | 13    | APM2 A9 pin     | |
| +-------+-----------------+ |
| | 47    | APM1 relay      | |
| +-------+-----------------+ |
| | 50    | Pixhawk AUXOUT1 | |
| +-------+-----------------+ |
| | 51    | Pixhawk AUXOUT2 | |
| +-------+-----------------+ |
| | 52    | Pixhawk AUXOUT3 | |
| +-------+-----------------+ |
| | 53    | Pixhawk AUXOUT4 | |
| +-------+-----------------+ |
| | 54    | Pixhawk AUXOUT5 | |
| +-------+-----------------+ |
| | 55    | Pixhawk AUXOUT6 | |
| +-------+-----------------+ |
| | 111   | PX4 FMU Relay1  | |
| +-------+-----------------+ |
| | 112   | PX4 FMU Relay2  | |
| +-------+-----------------+ |
| | 113   | PX4IO Relay1    | |
| +-------+-----------------+ |
| | 114   | PX4IO Relay2    | |
| +-------+-----------------+ |
| | 115   | PX4IO ACC1      | |
| +-------+-----------------+ |
| | 116   | PX4IO ACC2      | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RELAY_PIN3:

RELAY\_PIN3: Third Relay Pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin number for 3rd relay control\.


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | -1    | Disabled        | |
| +-------+-----------------+ |
| | 13    | APM2 A9 pin     | |
| +-------+-----------------+ |
| | 47    | APM1 relay      | |
| +-------+-----------------+ |
| | 50    | Pixhawk AUXOUT1 | |
| +-------+-----------------+ |
| | 51    | Pixhawk AUXOUT2 | |
| +-------+-----------------+ |
| | 52    | Pixhawk AUXOUT3 | |
| +-------+-----------------+ |
| | 53    | Pixhawk AUXOUT4 | |
| +-------+-----------------+ |
| | 54    | Pixhawk AUXOUT5 | |
| +-------+-----------------+ |
| | 55    | Pixhawk AUXOUT6 | |
| +-------+-----------------+ |
| | 111   | PX4 FMU Relay1  | |
| +-------+-----------------+ |
| | 112   | PX4 FMU Relay2  | |
| +-------+-----------------+ |
| | 113   | PX4IO Relay1    | |
| +-------+-----------------+ |
| | 114   | PX4IO Relay2    | |
| +-------+-----------------+ |
| | 115   | PX4IO ACC1      | |
| +-------+-----------------+ |
| | 116   | PX4IO ACC2      | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RELAY_PIN4:

RELAY\_PIN4: Fourth Relay Pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin number for 4th relay control\.


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | -1    | Disabled        | |
| +-------+-----------------+ |
| | 13    | APM2 A9 pin     | |
| +-------+-----------------+ |
| | 47    | APM1 relay      | |
| +-------+-----------------+ |
| | 50    | Pixhawk AUXOUT1 | |
| +-------+-----------------+ |
| | 51    | Pixhawk AUXOUT2 | |
| +-------+-----------------+ |
| | 52    | Pixhawk AUXOUT3 | |
| +-------+-----------------+ |
| | 53    | Pixhawk AUXOUT4 | |
| +-------+-----------------+ |
| | 54    | Pixhawk AUXOUT5 | |
| +-------+-----------------+ |
| | 55    | Pixhawk AUXOUT6 | |
| +-------+-----------------+ |
| | 111   | PX4 FMU Relay1  | |
| +-------+-----------------+ |
| | 112   | PX4 FMU Relay2  | |
| +-------+-----------------+ |
| | 113   | PX4IO Relay1    | |
| +-------+-----------------+ |
| | 114   | PX4IO Relay2    | |
| +-------+-----------------+ |
| | 115   | PX4IO ACC1      | |
| +-------+-----------------+ |
| | 116   | PX4IO ACC2      | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RELAY_DEFAULT:

RELAY\_DEFAULT: Default relay state
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The state of the relay on boot\. 


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Off      | |
| +-------+----------+ |
| | 1     | On       | |
| +-------+----------+ |
| | 2     | NoChange | |
| +-------+----------+ |
|                      |
+----------------------+





.. _parameters_CHUTE_:

CHUTE\_ Parameters
------------------


.. _CHUTE_ENABLED:

CHUTE\_ENABLED: Parachute release enabled or disabled
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Parachute release enabled or disabled


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _CHUTE_TYPE:

CHUTE\_TYPE: Parachute release mechanism type \(relay or servo\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Parachute release mechanism type \(relay or servo\)


+--------------------------+
| Values                   |
+==========================+
| +-------+--------------+ |
| | Value | Meaning      | |
| +=======+==============+ |
| | 0     | First Relay  | |
| +-------+--------------+ |
| | 1     | Second Relay | |
| +-------+--------------+ |
| | 2     | Third Relay  | |
| +-------+--------------+ |
| | 3     | Fourth Relay | |
| +-------+--------------+ |
| | 10    | Servo        | |
| +-------+--------------+ |
|                          |
+--------------------------+




.. _CHUTE_SERVO_ON:

CHUTE\_SERVO\_ON: Parachute Servo ON PWM value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Parachute Servo PWM value when parachute is released


+-------------+-----------+-------+
| Range       | Increment | Units |
+=============+===========+=======+
| 1000 - 2000 | 1         | pwm   |
+-------------+-----------+-------+




.. _CHUTE_SERVO_OFF:

CHUTE\_SERVO\_OFF: Servo OFF PWM value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Parachute Servo PWM value when parachute is not released


+-------------+-----------+-------+
| Range       | Increment | Units |
+=============+===========+=======+
| 1000 - 2000 | 1         | pwm   |
+-------------+-----------+-------+




.. _CHUTE_ALT_MIN:

CHUTE\_ALT\_MIN: Parachute min altitude in meters above home
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Parachute min altitude above home\.  Parachute will not be released below this altitude\.  0 to disable alt check\.


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 0 - 32000 | 1         | Meters |
+-----------+-----------+--------+




.. _CHUTE_DELAY_MS:

CHUTE\_DELAY\_MS: Parachute release delay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Delay in millseconds between motor stop and chute release


+----------+-----------+--------------+
| Range    | Increment | Units        |
+==========+===========+==============+
| 0 - 5000 | 1         | Milliseconds |
+----------+-----------+--------------+





.. _parameters_RNGFND:

RNGFND Parameters
-----------------


.. _RNGFND_TYPE:

RNGFND\_TYPE: Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


What type of rangefinder device that is connected


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Analog              | |
| +-------+---------------------+ |
| | 2     | APM2-MaxbotixI2C    | |
| +-------+---------------------+ |
| | 3     | APM2-PulsedLightI2C | |
| +-------+---------------------+ |
| | 4     | PX4-I2C             | |
| +-------+---------------------+ |
| | 5     | PX4-PWM             | |
| +-------+---------------------+ |
| | 6     | BBB-PRU             | |
| +-------+---------------------+ |
| | 7     | LightWareI2C        | |
| +-------+---------------------+ |
| | 8     | LightWareSerial     | |
| +-------+---------------------+ |
| | 9     | Bebop               | |
| +-------+---------------------+ |
| | 10    | MAVLink             | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _RNGFND_PIN:

RNGFND\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog pin that rangefinder is connected to\. Set this to 0\.\.9 for the APM2 analog pins\. Set to 64 on an APM1 for the dedicated \'airspeed\' port on the end of the board\. Set to 11 on PX4 for the analog \'airspeed\' port\. Set to 15 on the Pixhawk for the analog \'airspeed\' port\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | Not Used              | |
| +-------+-----------------------+ |
| | 0     | APM2-A0               | |
| +-------+-----------------------+ |
| | 1     | APM2-A1               | |
| +-------+-----------------------+ |
| | 2     | APM2-A2               | |
| +-------+-----------------------+ |
| | 3     | APM2-A3               | |
| +-------+-----------------------+ |
| | 4     | APM2-A4               | |
| +-------+-----------------------+ |
| | 5     | APM2-A5               | |
| +-------+-----------------------+ |
| | 6     | APM2-A6               | |
| +-------+-----------------------+ |
| | 7     | APM2-A7               | |
| +-------+-----------------------+ |
| | 8     | APM2-A8               | |
| +-------+-----------------------+ |
| | 9     | APM2-A9               | |
| +-------+-----------------------+ |
| | 11    | PX4-airspeed port     | |
| +-------+-----------------------+ |
| | 15    | Pixhawk-airspeed port | |
| +-------+-----------------------+ |
| | 64    | APM1-airspeed port    | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _RNGFND_SCALING:

RNGFND\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\.


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 0.001     | meters/Volt |
+-----------+-------------+




.. _RNGFND_OFFSET:

RNGFND\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance for analog rangefinders\. Offset added to distance in centimeters for PWM and I2C Lidars


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | Volts |
+-----------+-------+




.. _RNGFND_FUNCTION:

RNGFND\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND_MIN_CM:

RNGFND\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND_MAX_CM:

RNGFND\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND_STOP_PIN:

RNGFND\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for an analog rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This can be used to ensure that multiple sonar rangefinders don\'t interfere with each other\.


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | -1    | Not Used        | |
| +-------+-----------------+ |
| | 50    | Pixhawk AUXOUT1 | |
| +-------+-----------------+ |
| | 51    | Pixhawk AUXOUT2 | |
| +-------+-----------------+ |
| | 52    | Pixhawk AUXOUT3 | |
| +-------+-----------------+ |
| | 53    | Pixhawk AUXOUT4 | |
| +-------+-----------------+ |
| | 54    | Pixhawk AUXOUT5 | |
| +-------+-----------------+ |
| | 55    | Pixhawk AUXOUT6 | |
| +-------+-----------------+ |
| | 111   | PX4 FMU Relay1  | |
| +-------+-----------------+ |
| | 112   | PX4 FMU Relay2  | |
| +-------+-----------------+ |
| | 113   | PX4IO Relay1    | |
| +-------+-----------------+ |
| | 114   | PX4IO Relay2    | |
| +-------+-----------------+ |
| | 115   | PX4IO ACC1      | |
| +-------+-----------------+ |
| | 116   | PX4IO ACC2      | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RNGFND_SETTLE:

RNGFND\_SETTLE: Rangefinder settle time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The time in milliseconds that the rangefinder reading takes to settle\. This is only used when a STOP\_PIN is specified\. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP\_PIN high\. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again\.


+-----------+--------------+
| Increment | Units        |
+===========+==============+
| 1         | milliseconds |
+-----------+--------------+




.. _RNGFND_RMETRIC:

RNGFND\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND_PWRRNG:

RNGFND\_PWRRNG: Powersave range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode \(if available\)\. A value of zero means power saving is not enabled


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 - 32767 | meters |
+-----------+--------+




.. _RNGFND_GNDCLEAR:

RNGFND\_GNDCLEAR: Distance \(in cm\) from the range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets the expected range measurement\(in cm\) that the range finder should return when the vehicle is on the ground\.


+---------+-----------+-------------+
| Range   | Increment | Units       |
+=========+===========+=============+
| 0 - 127 | 1         | centimeters |
+---------+-----------+-------------+




.. _RNGFND2_TYPE:

RNGFND2\_TYPE: Second Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

What type of rangefinder device that is connected


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Analog              | |
| +-------+---------------------+ |
| | 2     | APM2-MaxbotixI2C    | |
| +-------+---------------------+ |
| | 3     | APM2-PulsedLightI2C | |
| +-------+---------------------+ |
| | 4     | PX4-I2C             | |
| +-------+---------------------+ |
| | 5     | PX4-PWM             | |
| +-------+---------------------+ |
| | 6     | BBB-PRU             | |
| +-------+---------------------+ |
| | 7     | LightWareI2C        | |
| +-------+---------------------+ |
| | 8     | LightWareSerial     | |
| +-------+---------------------+ |
| | 9     | Bebop               | |
| +-------+---------------------+ |
| | 10    | MAVLink             | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _RNGFND2_PIN:

RNGFND2\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Analog pin that rangefinder is connected to\. Set this to 0\.\.9 for the APM2 analog pins\. Set to 64 on an APM1 for the dedicated \'airspeed\' port on the end of the board\. Set to 11 on PX4 for the analog \'airspeed\' port\. Set to 15 on the Pixhawk for the analog \'airspeed\' port\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | Not Used              | |
| +-------+-----------------------+ |
| | 0     | APM2-A0               | |
| +-------+-----------------------+ |
| | 1     | APM2-A1               | |
| +-------+-----------------------+ |
| | 2     | APM2-A2               | |
| +-------+-----------------------+ |
| | 3     | APM2-A3               | |
| +-------+-----------------------+ |
| | 4     | APM2-A4               | |
| +-------+-----------------------+ |
| | 5     | APM2-A5               | |
| +-------+-----------------------+ |
| | 6     | APM2-A6               | |
| +-------+-----------------------+ |
| | 7     | APM2-A7               | |
| +-------+-----------------------+ |
| | 8     | APM2-A8               | |
| +-------+-----------------------+ |
| | 9     | APM2-A9               | |
| +-------+-----------------------+ |
| | 11    | PX4-airspeed port     | |
| +-------+-----------------------+ |
| | 15    | Pixhawk-airspeed port | |
| +-------+-----------------------+ |
| | 64    | APM1-airspeed port    | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _RNGFND2_SCALING:

RNGFND2\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\.


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 0.001     | meters/Volt |
+-----------+-------------+




.. _RNGFND2_OFFSET:

RNGFND2\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Offset in volts for zero distance


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.001     | Volts |
+-----------+-------+




.. _RNGFND2_FUNCTION:

RNGFND2\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND2_MIN_CM:

RNGFND2\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND2_MAX_CM:

RNGFND2\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Maximum distance in centimeters that rangefinder can reliably read


+-----------+-------------+
| Increment | Units       |
+===========+=============+
| 1         | centimeters |
+-----------+-------------+




.. _RNGFND2_STOP_PIN:

RNGFND2\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Digital pin that enables\/disables rangefinder measurement for an analog rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This can be used to ensure that multiple sonar rangefinders don\'t interfere with each other\.


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | -1    | Not Used        | |
| +-------+-----------------+ |
| | 50    | Pixhawk AUXOUT1 | |
| +-------+-----------------+ |
| | 51    | Pixhawk AUXOUT2 | |
| +-------+-----------------+ |
| | 52    | Pixhawk AUXOUT3 | |
| +-------+-----------------+ |
| | 53    | Pixhawk AUXOUT4 | |
| +-------+-----------------+ |
| | 54    | Pixhawk AUXOUT5 | |
| +-------+-----------------+ |
| | 55    | Pixhawk AUXOUT6 | |
| +-------+-----------------+ |
| | 111   | PX4 FMU Relay1  | |
| +-------+-----------------+ |
| | 112   | PX4 FMU Relay2  | |
| +-------+-----------------+ |
| | 113   | PX4IO Relay1    | |
| +-------+-----------------+ |
| | 114   | PX4IO Relay2    | |
| +-------+-----------------+ |
| | 115   | PX4IO ACC1      | |
| +-------+-----------------+ |
| | 116   | PX4IO ACC2      | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RNGFND2_SETTLE:

RNGFND2\_SETTLE: Sonar settle time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The time in milliseconds that the rangefinder reading takes to settle\. This is only used when a STOP\_PIN is specified\. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP\_PIN high\. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again\.


+-----------+--------------+
| Increment | Units        |
+===========+==============+
| 1         | milliseconds |
+-----------+--------------+




.. _RNGFND2_RMETRIC:

RNGFND2\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND2_GNDCLEAR:

RNGFND2\_GNDCLEAR: Distance \(in cm\) from the second range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the expected range measurement\(in cm\) that the second range finder should return when the vehicle is on the ground\.


+---------+-----------+-------------+
| Range   | Increment | Units       |
+=========+===========+=============+
| 0 - 127 | 1         | centimeters |
+---------+-----------+-------------+




.. _RNGFND_ADDR:

RNGFND\_ADDR: Bus address of sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the bus address of the sensor\, where applicable\. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses\. A value of 0 disables the sensor\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 127 | 1         |
+---------+-----------+




.. _RNGFND2_ADDR:

RNGFND2\_ADDR: Bus address of 2nd rangefinder
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the bus address of the sensor\, where applicable\. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses\. A value of 0 disables the sensor\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 127 | 1         |
+---------+-----------+




.. _RNGFND3_TYPE:

RNGFND3\_TYPE: Second Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


What type of rangefinder device that is connected


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Analog              | |
| +-------+---------------------+ |
| | 2     | APM2-MaxbotixI2C    | |
| +-------+---------------------+ |
| | 3     | APM2-PulsedLightI2C | |
| +-------+---------------------+ |
| | 4     | PX4-I2C             | |
| +-------+---------------------+ |
| | 5     | PX4-PWM             | |
| +-------+---------------------+ |
| | 6     | BBB-PRU             | |
| +-------+---------------------+ |
| | 7     | LightWareI2C        | |
| +-------+---------------------+ |
| | 8     | LightWareSerial     | |
| +-------+---------------------+ |
| | 9     | Bebop               | |
| +-------+---------------------+ |
| | 10    | MAVLink             | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _RNGFND3_PIN:

RNGFND3\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog pin that rangefinder is connected to\. Set this to 0\.\.9 for the APM2 analog pins\. Set to 64 on an APM1 for the dedicated \'airspeed\' port on the end of the board\. Set to 11 on PX4 for the analog \'airspeed\' port\. Set to 15 on the Pixhawk for the analog \'airspeed\' port\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | Not Used              | |
| +-------+-----------------------+ |
| | 0     | APM2-A0               | |
| +-------+-----------------------+ |
| | 1     | APM2-A1               | |
| +-------+-----------------------+ |
| | 2     | APM2-A2               | |
| +-------+-----------------------+ |
| | 3     | APM2-A3               | |
| +-------+-----------------------+ |
| | 4     | APM2-A4               | |
| +-------+-----------------------+ |
| | 5     | APM2-A5               | |
| +-------+-----------------------+ |
| | 6     | APM2-A6               | |
| +-------+-----------------------+ |
| | 7     | APM2-A7               | |
| +-------+-----------------------+ |
| | 8     | APM2-A8               | |
| +-------+-----------------------+ |
| | 9     | APM2-A9               | |
| +-------+-----------------------+ |
| | 11    | PX4-airspeed port     | |
| +-------+-----------------------+ |
| | 15    | Pixhawk-airspeed port | |
| +-------+-----------------------+ |
| | 64    | APM1-airspeed port    | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _RNGFND3_SCALING:

RNGFND3\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\.


+-------------+-----------+
| Units       | Increment |
+=============+===========+
| meters/Volt | 0.001     |
+-------------+-----------+




.. _RNGFND3_OFFSET:

RNGFND3\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance


+-------+-----------+
| Units | Increment |
+=======+===========+
| Volts | 0.001     |
+-------+-----------+




.. _RNGFND3_FUNCTION:

RNGFND3\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND3_MIN_CM:

RNGFND3\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-------------+-----------+
| Units       | Increment |
+=============+===========+
| centimeters | 1         |
+-------------+-----------+




.. _RNGFND3_MAX_CM:

RNGFND3\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-------------+-----------+
| Units       | Increment |
+=============+===========+
| centimeters | 1         |
+-------------+-----------+




.. _RNGFND3_STOP_PIN:

RNGFND3\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for an analog rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This can be used to ensure that multiple sonar rangefinders don\'t interfere with each other\.


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | -1    | Not Used        | |
| +-------+-----------------+ |
| | 50    | Pixhawk AUXOUT1 | |
| +-------+-----------------+ |
| | 51    | Pixhawk AUXOUT2 | |
| +-------+-----------------+ |
| | 52    | Pixhawk AUXOUT3 | |
| +-------+-----------------+ |
| | 53    | Pixhawk AUXOUT4 | |
| +-------+-----------------+ |
| | 54    | Pixhawk AUXOUT5 | |
| +-------+-----------------+ |
| | 55    | Pixhawk AUXOUT6 | |
| +-------+-----------------+ |
| | 111   | PX4 FMU Relay1  | |
| +-------+-----------------+ |
| | 112   | PX4 FMU Relay2  | |
| +-------+-----------------+ |
| | 113   | PX4IO Relay1    | |
| +-------+-----------------+ |
| | 114   | PX4IO Relay2    | |
| +-------+-----------------+ |
| | 115   | PX4IO ACC1      | |
| +-------+-----------------+ |
| | 116   | PX4IO ACC2      | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RNGFND3_SETTLE:

RNGFND3\_SETTLE: Sonar settle time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The time in milliseconds that the rangefinder reading takes to settle\. This is only used when a STOP\_PIN is specified\. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP\_PIN high\. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again\.


+--------------+-----------+
| Units        | Increment |
+==============+===========+
| milliseconds | 1         |
+--------------+-----------+




.. _RNGFND3_RMETRIC:

RNGFND3\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND3_GNDCLEAR:

RNGFND3\_GNDCLEAR: Distance \(in cm\) from the second range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the expected range measurement\(in cm\) that the second range finder should return when the vehicle is on the ground\.


+---------+-----------+-------------+
| Range   | Increment | Units       |
+=========+===========+=============+
| 0 - 127 | 1         | centimeters |
+---------+-----------+-------------+




.. _RNGFND3_ADDR:

RNGFND3\_ADDR: Bus address of 2nd rangefinder
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the bus address of the sensor\, where applicable\. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses\. A value of 0 disables the sensor\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 127 | 1         |
+---------+-----------+




.. _RNGFND4_TYPE:

RNGFND4\_TYPE: Second Rangefinder type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


What type of rangefinder device that is connected


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Analog              | |
| +-------+---------------------+ |
| | 2     | APM2-MaxbotixI2C    | |
| +-------+---------------------+ |
| | 3     | APM2-PulsedLightI2C | |
| +-------+---------------------+ |
| | 4     | PX4-I2C             | |
| +-------+---------------------+ |
| | 5     | PX4-PWM             | |
| +-------+---------------------+ |
| | 6     | BBB-PRU             | |
| +-------+---------------------+ |
| | 7     | LightWareI2C        | |
| +-------+---------------------+ |
| | 8     | LightWareSerial     | |
| +-------+---------------------+ |
| | 9     | Bebop               | |
| +-------+---------------------+ |
| | 10    | MAVLink             | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _RNGFND4_PIN:

RNGFND4\_PIN: Rangefinder pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Analog pin that rangefinder is connected to\. Set this to 0\.\.9 for the APM2 analog pins\. Set to 64 on an APM1 for the dedicated \'airspeed\' port on the end of the board\. Set to 11 on PX4 for the analog \'airspeed\' port\. Set to 15 on the Pixhawk for the analog \'airspeed\' port\.


+-----------------------------------+
| Values                            |
+===================================+
| +-------+-----------------------+ |
| | Value | Meaning               | |
| +=======+=======================+ |
| | -1    | Not Used              | |
| +-------+-----------------------+ |
| | 0     | APM2-A0               | |
| +-------+-----------------------+ |
| | 1     | APM2-A1               | |
| +-------+-----------------------+ |
| | 2     | APM2-A2               | |
| +-------+-----------------------+ |
| | 3     | APM2-A3               | |
| +-------+-----------------------+ |
| | 4     | APM2-A4               | |
| +-------+-----------------------+ |
| | 5     | APM2-A5               | |
| +-------+-----------------------+ |
| | 6     | APM2-A6               | |
| +-------+-----------------------+ |
| | 7     | APM2-A7               | |
| +-------+-----------------------+ |
| | 8     | APM2-A8               | |
| +-------+-----------------------+ |
| | 9     | APM2-A9               | |
| +-------+-----------------------+ |
| | 11    | PX4-airspeed port     | |
| +-------+-----------------------+ |
| | 15    | Pixhawk-airspeed port | |
| +-------+-----------------------+ |
| | 64    | APM1-airspeed port    | |
| +-------+-----------------------+ |
|                                   |
+-----------------------------------+




.. _RNGFND4_SCALING:

RNGFND4\_SCALING: Rangefinder scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between rangefinder reading and distance\. For the linear and inverted functions this is in meters per volt\. For the hyperbolic function the units are meterVolts\.


+-------------+-----------+
| Units       | Increment |
+=============+===========+
| meters/Volt | 0.001     |
+-------------+-----------+




.. _RNGFND4_OFFSET:

RNGFND4\_OFFSET: rangefinder offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset in volts for zero distance


+-------+-----------+
| Units | Increment |
+=======+===========+
| Volts | 0.001     |
+-------+-----------+




.. _RNGFND4_FUNCTION:

RNGFND4\_FUNCTION: Rangefinder function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control over what function is used to calculate distance\. For a linear function\, the distance is \(voltage\-offset\)\*scaling\. For a inverted function the distance is \(offset\-voltage\)\*scaling\. For a hyperbolic function the distance is scaling\/\(voltage\-offset\)\. The functions return the distance in meters\.


+------------------------+
| Values                 |
+========================+
| +-------+------------+ |
| | Value | Meaning    | |
| +=======+============+ |
| | 0     | Linear     | |
| +-------+------------+ |
| | 1     | Inverted   | |
| +-------+------------+ |
| | 2     | Hyperbolic | |
| +-------+------------+ |
|                        |
+------------------------+




.. _RNGFND4_MIN_CM:

RNGFND4\_MIN\_CM: Rangefinder minimum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum distance in centimeters that rangefinder can reliably read


+-------------+-----------+
| Units       | Increment |
+=============+===========+
| centimeters | 1         |
+-------------+-----------+




.. _RNGFND4_MAX_CM:

RNGFND4\_MAX\_CM: Rangefinder maximum distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum distance in centimeters that rangefinder can reliably read


+-------------+-----------+
| Units       | Increment |
+=============+===========+
| centimeters | 1         |
+-------------+-----------+




.. _RNGFND4_STOP_PIN:

RNGFND4\_STOP\_PIN: Rangefinder stop pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Digital pin that enables\/disables rangefinder measurement for an analog rangefinder\. A value of \-1 means no pin\. If this is set\, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it\. This can be used to ensure that multiple sonar rangefinders don\'t interfere with each other\.


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | -1    | Not Used        | |
| +-------+-----------------+ |
| | 50    | Pixhawk AUXOUT1 | |
| +-------+-----------------+ |
| | 51    | Pixhawk AUXOUT2 | |
| +-------+-----------------+ |
| | 52    | Pixhawk AUXOUT3 | |
| +-------+-----------------+ |
| | 53    | Pixhawk AUXOUT4 | |
| +-------+-----------------+ |
| | 54    | Pixhawk AUXOUT5 | |
| +-------+-----------------+ |
| | 55    | Pixhawk AUXOUT6 | |
| +-------+-----------------+ |
| | 111   | PX4 FMU Relay1  | |
| +-------+-----------------+ |
| | 112   | PX4 FMU Relay2  | |
| +-------+-----------------+ |
| | 113   | PX4IO Relay1    | |
| +-------+-----------------+ |
| | 114   | PX4IO Relay2    | |
| +-------+-----------------+ |
| | 115   | PX4IO ACC1      | |
| +-------+-----------------+ |
| | 116   | PX4IO ACC2      | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RNGFND4_SETTLE:

RNGFND4\_SETTLE: Sonar settle time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The time in milliseconds that the rangefinder reading takes to settle\. This is only used when a STOP\_PIN is specified\. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP\_PIN high\. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again\.


+--------------+-----------+
| Units        | Increment |
+==============+===========+
| milliseconds | 1         |
+--------------+-----------+




.. _RNGFND4_RMETRIC:

RNGFND4\_RMETRIC: Ratiometric
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter sets whether an analog rangefinder is ratiometric\. Most analog rangefinders are ratiometric\, meaning that their output voltage is influenced by the supply voltage\. Some analog rangefinders \(such as the SF\/02\) have their own internal voltage regulators so they are not ratiometric\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | No      | |
| +-------+---------+ |
| | 1     | Yes     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RNGFND4_GNDCLEAR:

RNGFND4\_GNDCLEAR: Distance \(in cm\) from the second range finder to the ground
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the expected range measurement\(in cm\) that the second range finder should return when the vehicle is on the ground\.


+---------+-----------+-------------+
| Range   | Increment | Units       |
+=========+===========+=============+
| 0 - 127 | 1         | centimeters |
+---------+-----------+-------------+




.. _RNGFND4_ADDR:

RNGFND4\_ADDR: Bus address of 2nd rangefinder
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the bus address of the sensor\, where applicable\. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses\. A value of 0 disables the sensor\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 127 | 1         |
+---------+-----------+





.. _parameters_TERRAIN_:

TERRAIN\_ Parameters
--------------------


.. _TERRAIN_ENABLE:

TERRAIN\_ENABLE: Terrain data enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable terrain data\. This enables the vehicle storing a database of terrain data on the SD card\. The terrain data is requested from the ground station as needed\, and stored for later use on the SD card\. To be useful the ground station must support TERRAIN\_REQUEST messages and have access to a terrain database\, such as the SRTM database\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _TERRAIN_SPACING:

TERRAIN\_SPACING: Terrain grid spacing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Distance between terrain grid points in meters\. This controls the horizontal resolution of the terrain data that is stored on te SD card and requested from the ground station\. If your GCS is using the worldwide SRTM database then a resolution of 100 meters is appropriate\. Some parts of the world may have higher resolution data available\, such as 30 meter data available in the SRTM database in the USA\. The grid spacing also controls how much data is kept in memory during flight\. A larger grid spacing will allow for a larger amount of data in memory\. A grid spacing of 100 meters results in the vehicle keeping 12 grid squares in memory with each grid square having a size of 2\.7 kilometers by 3\.2 kilometers\. Any additional grid squares are stored on the SD once they are fetched from the GCS and will be demand loaded as needed\.


+--------+-----------+
| Units  | Increment |
+========+===========+
| meters | 1         |
+--------+-----------+





.. _parameters_ADSB_:

ADSB\_ Parameters
-----------------


.. _ADSB_ENABLE:

ADSB\_ENABLE: Enable ADSB
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable ADS\-B


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _ADSB_BEHAVIOR:

ADSB\_BEHAVIOR: ADSB based Collision Avoidance Behavior
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ADSB based Collision Avoidance Behavior selector


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | None             | |
| +-------+------------------+ |
| | 1     | Loiter           | |
| +-------+------------------+ |
| | 2     | LoiterAndDescend | |
| +-------+------------------+ |
|                              |
+------------------------------+





.. _parameters_Q_:

Q\_ Parameters
--------------


.. _Q_ENABLE:

Q\_ENABLE: Enable QuadPlane
~~~~~~~~~~~~~~~~~~~~~~~~~~~


This enables QuadPlane functionality\, assuming multicopter motors start on output 5\. If this is set to 2 then when starting AUTO mode it will initially be in VTOL AUTO mode\.


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | Disable          | |
| +-------+------------------+ |
| | 1     | Enable           | |
| +-------+------------------+ |
| | 2     | Enable VTOL AUTO | |
| +-------+------------------+ |
|                              |
+------------------------------+




.. _Q_ANGLE_MAX:

Q\_ANGLE\_MAX: Angle Max
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Maximum lean angle in all VTOL flight modes


+-------------+---------------+
| Range       | Units         |
+=============+===============+
| 1000 - 8000 | Centi-degrees |
+-------------+---------------+




.. _Q_TRANSITION_MS:

Q\_TRANSITION\_MS: Transition time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Transition time in milliseconds after minimum airspeed is reached


+-----------+---------------+
| Range     | Units         |
+===========+===============+
| 0 - 30000 | milli-seconds |
+-----------+---------------+




.. _Q_PZ_P:

Q\_PZ\_P: Position \(vertical\) controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position \(vertical\) controller P gain\.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller


+---------------+
| Range         |
+===============+
| 1.000 - 3.000 |
+---------------+




.. _Q_PXY_P:

Q\_PXY\_P: Position \(horizonal\) controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Loiter position controller P gain\.  Converts the distance \(in the latitude direction\) to the target location into a desired speed which is then passed to the loiter latitude rate controller


+---------------+
| Range         |
+===============+
| 0.500 - 2.000 |
+---------------+




.. _Q_VXY_P:

Q\_VXY\_P: Velocity \(horizontal\) P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Velocity \(horizontal\) P gain\.  Converts the difference between desired velocity to a target acceleration


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 6.0 | 0.1       |
+-----------+-----------+




.. _Q_VXY_I:

Q\_VXY\_I: Velocity \(horizontal\) I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Velocity \(horizontal\) I gain\.  Corrects long\-term difference in desired velocity to a target acceleration


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.02 - 1.00 | 0.01      |
+-------------+-----------+




.. _Q_VXY_IMAX:

Q\_VXY\_IMAX: Velocity \(horizontal\) integrator maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Velocity \(horizontal\) integrator maximum\.  Constrains the target acceleration that the I gain will output


+----------+-----------+--------+
| Range    | Increment | Units  |
+==========+===========+========+
| 0 - 4500 | 10        | cm/s/s |
+----------+-----------+--------+




.. _Q_VZ_P:

Q\_VZ\_P: Velocity \(vertical\) P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Velocity \(vertical\) P gain\.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller


+---------------+
| Range         |
+===============+
| 1.000 - 8.000 |
+---------------+




.. _Q_AZ_P:

Q\_AZ\_P: Throttle acceleration controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Throttle acceleration controller P gain\.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output


+---------------+
| Range         |
+===============+
| 0.500 - 1.500 |
+---------------+




.. _Q_AZ_I:

Q\_AZ\_I: Throttle acceleration controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Throttle acceleration controller I gain\.  Corrects long\-term difference in desired vertical acceleration and actual acceleration


+---------------+
| Range         |
+===============+
| 0.000 - 3.000 |
+---------------+




.. _Q_AZ_IMAX:

Q\_AZ\_IMAX: Throttle acceleration controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Throttle acceleration controller I gain maximum\.  Constrains the maximum pwm that the I term will generate


+----------+------------+
| Range    | Units      |
+==========+============+
| 0 - 1000 | Percent*10 |
+----------+------------+




.. _Q_AZ_D:

Q\_AZ\_D: Throttle acceleration controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Throttle acceleration controller D gain\.  Compensates for short\-term change in desired vertical acceleration vs actual acceleration


+---------------+
| Range         |
+===============+
| 0.000 - 0.400 |
+---------------+




.. _Q_AZ_FILT_HZ:

Q\_AZ\_FILT\_HZ: Throttle acceleration filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Filter applied to acceleration to reduce noise\.  Lower values reduce noise but add delay\.


+-----------------+-------+
| Range           | Units |
+=================+=======+
| 1.000 - 100.000 | Hz    |
+-----------------+-------+




.. _Q_VELZ_MAX:

Q\_VELZ\_MAX: Pilot maximum vertical speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The maximum vertical velocity the pilot may request in cm\/s


+----------+-----------+--------------------+
| Range    | Increment | Units              |
+==========+===========+====================+
| 50 - 500 | 10        | Centimeters/Second |
+----------+-----------+--------------------+




.. _Q_ACCEL_Z:

Q\_ACCEL\_Z: Pilot vertical acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The vertical acceleration used when pilot is controlling the altitude


+----------+-----------+--------+
| Range    | Increment | Units  |
+==========+===========+========+
| 50 - 500 | 10        | cm/s/s |
+----------+-----------+--------+




.. _Q_RC_SPEED:

Q\_RC\_SPEED: RC output speed in Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the PWM refresh rate in Hz for QuadPlane quad motors


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 50 - 500 | 10        | Hz    |
+----------+-----------+-------+




.. _Q_THR_MIN_PWM:

Q\_THR\_MIN\_PWM: Minimum PWM output
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the minimum PWM output for the quad motors


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | Hz    |
+------------+-----------+-------+




.. _Q_THR_MAX_PWM:

Q\_THR\_MAX\_PWM: Maximum PWM output
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the maximum PWM output for the quad motors


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | Hz    |
+------------+-----------+-------+




.. _Q_ASSIST_SPEED:

Q\_ASSIST\_SPEED: Quadplane assistance speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the speed below which the quad motors will provide stability and lift assistance in fixed wing modes\. Zero means no assistance except during transition


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 100 | 0.1       | m/s   |
+---------+-----------+-------+




.. _Q_YAW_RATE_MAX:

Q\_YAW\_RATE\_MAX: Maximum yaw rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the maximum yaw rate in degrees\/second


+----------+-----------+----------------+
| Range    | Increment | Units          |
+==========+===========+================+
| 50 - 500 | 1         | degrees/second |
+----------+-----------+----------------+




.. _Q_LAND_SPEED:

Q\_LAND\_SPEED: Land speed
~~~~~~~~~~~~~~~~~~~~~~~~~~


The descent speed for the final stage of landing in cm\/s


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 30 - 200 | 10        | cm/s  |
+----------+-----------+-------+




.. _Q_LAND_FINAL_ALT:

Q\_LAND\_FINAL\_ALT: Land final altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The altitude at which we should switch to Q\_LAND\_SPEED descent rate


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 0.5 - 50 | 0.1       | m     |
+----------+-----------+-------+




.. _Q_THR_MID:

Q\_THR\_MID: Throttle Mid Position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The throttle output \(0 \~ 1000\) when throttle stick is in mid position\.  Used to scale the manual throttle so that the mid throttle stick position is close to the throttle required to hover


+-----------+-----------+------------+
| Range     | Increment | Units      |
+===========+===========+============+
| 300 - 700 | 10        | Percent*10 |
+-----------+-----------+------------+




.. _Q_TRAN_PIT_MAX:

Q\_TRAN\_PIT\_MAX: Transition max pitch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum pitch during transition to auto fixed wing flight


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 30 | 1         | Degrees |
+--------+-----------+---------+




.. _Q_FRAME_CLASS:

Q\_FRAME\_CLASS: Frame Class
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls major frame class for multicopter component


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Quad     | |
| +-------+----------+ |
| | 1     | Hexa     | |
| +-------+----------+ |
| | 2     | Octa     | |
| +-------+----------+ |
| | 3     | OctaQuad | |
| +-------+----------+ |
| | 4     | Y6       | |
| +-------+----------+ |
|                      |
+----------------------+




.. _Q_FRAME_TYPE:

Q\_FRAME\_TYPE: Frame Type \(\+\, X or V\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls motor mixing for multicopter component


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Plus    | |
| +-------+---------+ |
| | 1     | X       | |
| +-------+---------+ |
| | 2     | V       | |
| +-------+---------+ |
| | 3     | H       | |
| +-------+---------+ |
| | 4     | V-Tail  | |
| +-------+---------+ |
| | 5     | A-Tail  | |
| +-------+---------+ |
| | 10    | Y6B     | |
| +-------+---------+ |
|                     |
+---------------------+




.. _Q_VFWD_GAIN:

Q\_VFWD\_GAIN: Forward velocity hold gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls use of forward motor in vtol modes\. If this is zero then the forward motor will not be used for position control in VTOL modes\. A value of 0\.1 is a good place to start if you want to use the forward motor for position control\. No forward motor will be used in QSTABILIZE or QHOVER modes\. Use QLOITER for position hold with the forward motor\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 0.5 | 0.01      |
+---------+-----------+




.. _Q_WVANE_GAIN:

Q\_WVANE\_GAIN: Weathervaning gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls the tendency to yaw to face into the wind\. A value of 0\.4 is good for reasonably quick wind direction correction\. The weathervaning works by turning into the direction of roll\.


+-------+-----------+
| Range | Increment |
+=======+===========+
| 0 - 1 | 0.01      |
+-------+-----------+




.. _Q_WVANE_MINROLL:

Q\_WVANE\_MINROLL: Weathervaning min roll
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This set the minimum roll in degrees before active weathervaning will start\. This may need to be larger if your aircraft has bad roll trim\.


+--------+-----------+
| Range  | Increment |
+========+===========+
| 0 - 10 | 0.1       |
+--------+-----------+




.. _Q_RTL_ALT:

Q\_RTL\_ALT: QRTL return altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The altitude which QRTL mode heads to initially


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 1 - 200 | 1         | m     |
+---------+-----------+-------+




.. _Q_RTL_MODE:

Q\_RTL\_MODE: VTOL RTL mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~


If this is set to 1 then an RTL will change to QRTL when the loiter target is reached


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _Q_TILT_MASK:

Q\_TILT\_MASK: Tiltrotor mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is a bitmask of motors that are tiltable in a tiltrotor \(or tiltwing\)\. The mask is in terms of the standard motor order for the frame type\.


.. _Q_TILT_RATE:

Q\_TILT\_RATE: Tiltrotor tilt rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the maximum speed at which the motor angle will change for a tiltrotor


+----------+-----------+----------------+
| Range    | Increment | Units          |
+==========+===========+================+
| 10 - 300 | 1         | degrees/second |
+----------+-----------+----------------+




.. _Q_TILT_MAX:

Q\_TILT\_MAX: Tiltrotor maximum VTOL angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the maximum angle of the tiltable motors at which multicopter control will be enabled\. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT\_RATE


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 20 - 80 | 1         | degrees |
+---------+-----------+---------+




.. _Q_GUIDED_MODE:

Q\_GUIDED\_MODE: Enable VTOL in GUIDED mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This enables use of VTOL in guided mode\. When enabled the aircraft will switch to VTOL flight when the guided destination is reached and hover at the destination\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _Q_THR_MIN:

Q\_THR\_MIN: Throttle Minimum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The minimum throttle that will be sent to the motors to keep them spinning


+---------+-----------+------------+
| Range   | Increment | Units      |
+=========+===========+============+
| 0 - 300 | 1         | Percent*10 |
+---------+-----------+------------+





.. _parameters_TUNE_:

TUNE\_ Parameters
-----------------


.. _TUNE_PARAM:

TUNE\_PARAM: Transmitter tuning parameter or set of parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets which parameter or set of parameters will be tuned\. Values greater than 100 indicate a set of parameters rather than a single parameter\. Parameters less than 50 are for QuadPlane vertical lift motors only\.


+--------------------------------+
| Values                         |
+================================+
| +-------+--------------------+ |
| | Value | Meaning            | |
| +=======+====================+ |
| | 0     | None               | |
| +-------+--------------------+ |
| | 1     | RateRollPI         | |
| +-------+--------------------+ |
| | 2     | RateRollP          | |
| +-------+--------------------+ |
| | 3     | RateRollI          | |
| +-------+--------------------+ |
| | 4     | RateRollD          | |
| +-------+--------------------+ |
| | 5     | RatePitchPI        | |
| +-------+--------------------+ |
| | 6     | RatePitchP         | |
| +-------+--------------------+ |
| | 7     | RatePitchI         | |
| +-------+--------------------+ |
| | 8     | RatePitchD         | |
| +-------+--------------------+ |
| | 9     | RateYawPI          | |
| +-------+--------------------+ |
| | 10    | RateYawP           | |
| +-------+--------------------+ |
| | 11    | RateYawI           | |
| +-------+--------------------+ |
| | 12    | RateYawD           | |
| +-------+--------------------+ |
| | 13    | AngleRollP         | |
| +-------+--------------------+ |
| | 14    | AnglePitchP        | |
| +-------+--------------------+ |
| | 15    | AngleYawP          | |
| +-------+--------------------+ |
| | 16    | PosXYP             | |
| +-------+--------------------+ |
| | 17    | PosZP              | |
| +-------+--------------------+ |
| | 18    | VelXYP             | |
| +-------+--------------------+ |
| | 19    | VelXYI             | |
| +-------+--------------------+ |
| | 20    | VelZP              | |
| +-------+--------------------+ |
| | 21    | AccelZP            | |
| +-------+--------------------+ |
| | 22    | AccelZI            | |
| +-------+--------------------+ |
| | 23    | AccelZD            | |
| +-------+--------------------+ |
| | 50    | FixedWingRollP     | |
| +-------+--------------------+ |
| | 51    | FixedWingRollI     | |
| +-------+--------------------+ |
| | 52    | FixedWingRollD     | |
| +-------+--------------------+ |
| | 53    | FixedWingRollFF    | |
| +-------+--------------------+ |
| | 54    | FixedWingPitchP    | |
| +-------+--------------------+ |
| | 55    | FixedWingPitchI    | |
| +-------+--------------------+ |
| | 56    | FixedWingPitchD    | |
| +-------+--------------------+ |
| | 57    | FixedWingPitchFF   | |
| +-------+--------------------+ |
| | 101   | Set_RateRollPitch  | |
| +-------+--------------------+ |
| | 102   | Set_RateRoll       | |
| +-------+--------------------+ |
| | 103   | Set_RatePitch      | |
| +-------+--------------------+ |
| | 104   | Set_RateYaw        | |
| +-------+--------------------+ |
| | 105   | Set_AngleRollPitch | |
| +-------+--------------------+ |
| | 106   | Set_VelXY          | |
| +-------+--------------------+ |
| | 107   | Set_AccelZ         | |
| +-------+--------------------+ |
|                                |
+--------------------------------+




.. _TUNE_CHAN:

TUNE\_CHAN: Transmitter tuning channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the channel for transmitter tuning\. This should be connected to a knob or slider on your transmitter\. It needs to be setup to use the PWM range given by TUNE\_CHAN\_MIN to TUNE\_CHAN\_MAX


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 5     | Chan5   | |
| +-------+---------+ |
| | 6     | Chan6   | |
| +-------+---------+ |
| | 7     | Chan7   | |
| +-------+---------+ |
| | 8     | Chan8   | |
| +-------+---------+ |
| | 9     | Chan9   | |
| +-------+---------+ |
| | 10    | Chan10  | |
| +-------+---------+ |
| | 11    | Chan11  | |
| +-------+---------+ |
| | 12    | Chan12  | |
| +-------+---------+ |
| | 13    | Chan13  | |
| +-------+---------+ |
| | 14    | Chan14  | |
| +-------+---------+ |
| | 15    | Chan15  | |
| +-------+---------+ |
| | 16    | Chan16  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _TUNE_CHAN_MIN:

TUNE\_CHAN\_MIN: Transmitter tuning channel minimum pwm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the PWM lower limit for the tuning channel


+------------+
| Range      |
+============+
| 900 - 2100 |
+------------+




.. _TUNE_CHAN_MAX:

TUNE\_CHAN\_MAX: Transmitter tuning channel maximum pwm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the PWM upper limit for the tuning channel


+------------+
| Range      |
+============+
| 900 - 2100 |
+------------+




.. _TUNE_SELECTOR:

TUNE\_SELECTOR: Transmitter tuning selector channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the channel for the transmitter tuning selector switch\. This should be a 2 position switch\, preferably spring loaded\. A PWM above 1700 means high\, below 1300 means low\. If no selector is set then you won\'t be able to switch between parameters during flight or re\-center the tuning knob


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Chan1   | |
| +-------+---------+ |
| | 2     | Chan3   | |
| +-------+---------+ |
| | 3     | Chan3   | |
| +-------+---------+ |
| | 4     | Chan4   | |
| +-------+---------+ |
| | 5     | Chan5   | |
| +-------+---------+ |
| | 6     | Chan6   | |
| +-------+---------+ |
| | 7     | Chan7   | |
| +-------+---------+ |
| | 8     | Chan8   | |
| +-------+---------+ |
| | 9     | Chan9   | |
| +-------+---------+ |
| | 10    | Chan10  | |
| +-------+---------+ |
| | 11    | Chan11  | |
| +-------+---------+ |
| | 12    | Chan12  | |
| +-------+---------+ |
| | 13    | Chan13  | |
| +-------+---------+ |
| | 14    | Chan14  | |
| +-------+---------+ |
| | 15    | Chan15  | |
| +-------+---------+ |
| | 16    | Chan16  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _TUNE_RANGE:

TUNE\_RANGE: Transmitter tuning range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the range over which tuning will change a parameter\. A value of 2 means the tuning parameter will go from 0\.5 times the start value to 2x the start value over the range of the tuning channel



.. _parameters_Q_A_:

Q\_A\_ Parameters
-----------------


.. _Q_A_RAT_RLL_P:

Q\_A\_RAT\_RLL\_P: Roll axis rate controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate controller P gain\.  Converts the difference between desired roll rate and actual roll rate into a motor speed output


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.08 - 0.30 | 0.005     |
+-------------+-----------+




.. _Q_A_RAT_RLL_I:

Q\_A\_RAT\_RLL\_I: Roll axis rate controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate controller I gain\.  Corrects long\-term difference in desired roll rate vs actual roll rate


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.01 - 0.5 | 0.01      |
+------------+-----------+




.. _Q_A_RAT_RLL_IMAX:

Q\_A\_RAT\_RLL\_IMAX: Roll axis rate controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate controller I gain maximum\.  Constrains the maximum motor output that the I gain will output


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 1 | 0.01      | Percent |
+-------+-----------+---------+




.. _Q_A_RAT_RLL_D:

Q\_A\_RAT\_RLL\_D: Roll axis rate controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate controller D gain\.  Compensates for short\-term change in desired roll rate vs actual roll rate


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.0 - 0.02 | 0.001     |
+------------+-----------+




.. _Q_A_RAT_RLL_FILT:

Q\_A\_RAT\_RLL\_FILT: Roll axis rate conroller input frequency in Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate conroller input frequency in Hz


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 1 - 100 | 1         | Hz    |
+---------+-----------+-------+




.. _Q_A_RAT_PIT_P:

Q\_A\_RAT\_PIT\_P: Pitch axis rate controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate controller P gain\.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.08 - 0.30 | 0.005     |
+-------------+-----------+




.. _Q_A_RAT_PIT_I:

Q\_A\_RAT\_PIT\_I: Pitch axis rate controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate controller I gain\.  Corrects long\-term difference in desired pitch rate vs actual pitch rate


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.01 - 0.5 | 0.01      |
+------------+-----------+




.. _Q_A_RAT_PIT_IMAX:

Q\_A\_RAT\_PIT\_IMAX: Pitch axis rate controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate controller I gain maximum\.  Constrains the maximum motor output that the I gain will output


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 1 | 0.01      | Percent |
+-------+-----------+---------+




.. _Q_A_RAT_PIT_D:

Q\_A\_RAT\_PIT\_D: Pitch axis rate controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate controller D gain\.  Compensates for short\-term change in desired pitch rate vs actual pitch rate


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.0 - 0.02 | 0.001     |
+------------+-----------+




.. _Q_A_RAT_PIT_FILT:

Q\_A\_RAT\_PIT\_FILT: Pitch axis rate conroller input frequency in Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate conroller input frequency in Hz


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 1 - 100 | 1         | Hz    |
+---------+-----------+-------+




.. _Q_A_RAT_YAW_P:

Q\_A\_RAT\_YAW\_P: Yaw axis rate controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate controller P gain\.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.10 - 0.50 | 0.005     |
+-------------+-----------+




.. _Q_A_RAT_YAW_I:

Q\_A\_RAT\_YAW\_I: Yaw axis rate controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate controller I gain\.  Corrects long\-term difference in desired yaw rate vs actual yaw rate


+--------------+-----------+
| Range        | Increment |
+==============+===========+
| 0.010 - 0.05 | 0.01      |
+--------------+-----------+




.. _Q_A_RAT_YAW_IMAX:

Q\_A\_RAT\_YAW\_IMAX: Yaw axis rate controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate controller I gain maximum\.  Constrains the maximum motor output that the I gain will output


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 1 | 0.01      | Percent |
+-------+-----------+---------+




.. _Q_A_RAT_YAW_D:

Q\_A\_RAT\_YAW\_D: Yaw axis rate controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate controller D gain\.  Compensates for short\-term change in desired yaw rate vs actual yaw rate


+--------------+-----------+
| Range        | Increment |
+==============+===========+
| 0.000 - 0.02 | 0.001     |
+--------------+-----------+




.. _Q_A_RAT_YAW_FILT:

Q\_A\_RAT\_YAW\_FILT: Yaw axis rate conroller input frequency in Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate conroller input frequency in Hz


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 1 - 100 | 1         | Hz    |
+---------+-----------+-------+





.. _parameters_RC1_:

RC1\_ Parameters
----------------


.. _RC1_MIN:

RC1\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC1_TRIM:

RC1\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC1_MAX:

RC1\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC1_REV:

RC1\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC1_DZ:

RC1\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+





.. _parameters_RC2_:

RC2\_ Parameters
----------------


.. _RC2_MIN:

RC2\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC2_TRIM:

RC2\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC2_MAX:

RC2\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC2_REV:

RC2\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC2_DZ:

RC2\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+





.. _parameters_RC3_:

RC3\_ Parameters
----------------


.. _RC3_MIN:

RC3\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC3_TRIM:

RC3\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC3_MAX:

RC3\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC3_REV:

RC3\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC3_DZ:

RC3\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+





.. _parameters_RC4_:

RC4\_ Parameters
----------------


.. _RC4_MIN:

RC4\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC4_TRIM:

RC4\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC4_MAX:

RC4\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC4_REV:

RC4\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC4_DZ:

RC4\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+





.. _parameters_RC5_:

RC5\_ Parameters
----------------


.. _RC5_MIN:

RC5\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC5_TRIM:

RC5\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC5_MAX:

RC5\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC5_REV:

RC5\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC5_DZ:

RC5\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC5_FUNCTION:

RC5\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC6_:

RC6\_ Parameters
----------------


.. _RC6_MIN:

RC6\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC6_TRIM:

RC6\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC6_MAX:

RC6\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC6_REV:

RC6\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC6_DZ:

RC6\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC6_FUNCTION:

RC6\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC7_:

RC7\_ Parameters
----------------


.. _RC7_MIN:

RC7\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC7_TRIM:

RC7\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC7_MAX:

RC7\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC7_REV:

RC7\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC7_DZ:

RC7\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC7_FUNCTION:

RC7\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC8_:

RC8\_ Parameters
----------------


.. _RC8_MIN:

RC8\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC8_TRIM:

RC8\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC8_MAX:

RC8\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC8_REV:

RC8\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC8_DZ:

RC8\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC8_FUNCTION:

RC8\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC9_:

RC9\_ Parameters
----------------


.. _RC9_MIN:

RC9\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC9_TRIM:

RC9\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC9_MAX:

RC9\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC9_REV:

RC9\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC9_DZ:

RC9\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC9_FUNCTION:

RC9\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC10_:

RC10\_ Parameters
-----------------


.. _RC10_MIN:

RC10\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC10_TRIM:

RC10\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC10_MAX:

RC10\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC10_REV:

RC10\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC10_DZ:

RC10\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC10_FUNCTION:

RC10\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC11_:

RC11\_ Parameters
-----------------


.. _RC11_MIN:

RC11\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC11_TRIM:

RC11\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC11_MAX:

RC11\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC11_REV:

RC11\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC11_DZ:

RC11\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC11_FUNCTION:

RC11\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC12_:

RC12\_ Parameters
-----------------


.. _RC12_MIN:

RC12\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC12_TRIM:

RC12\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC12_MAX:

RC12\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC12_REV:

RC12\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC12_DZ:

RC12\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC12_FUNCTION:

RC12\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC13_:

RC13\_ Parameters
-----------------


.. _RC13_MIN:

RC13\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC13_TRIM:

RC13\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC13_MAX:

RC13\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC13_REV:

RC13\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC13_DZ:

RC13\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC13_FUNCTION:

RC13\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC14_:

RC14\_ Parameters
-----------------


.. _RC14_MIN:

RC14\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC14_TRIM:

RC14\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC14_MAX:

RC14\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC14_REV:

RC14\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC14_DZ:

RC14\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC14_FUNCTION:

RC14\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC15_:

RC15\_ Parameters
-----------------


.. _RC15_MIN:

RC15\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC15_TRIM:

RC15\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC15_MAX:

RC15\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC15_REV:

RC15\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC15_DZ:

RC15\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC15_FUNCTION:

RC15\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RC16_:

RC16\_ Parameters
-----------------


.. _RC16_MIN:

RC16\_MIN: RC min PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC minimum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC16_TRIM:

RC16\_TRIM: RC trim PWM
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC trim \(neutral\) PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC16_MAX:

RC16\_MAX: RC max PWM
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC maximum PWM pulse width\. Typically 1000 is lower limit\, 1500 is neutral and 2000 is upper limit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 800 - 2200 | 1         | pwm   |
+------------+-----------+-------+




.. _RC16_REV:

RC16\_REV: RC reverse
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Reverse servo operation\. Set to 1 for normal \(forward\) operation\. Set to \-1 to reverse this channel\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Reversed | |
| +-------+----------+ |
| | 1     | Normal   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _RC16_DZ:

RC16\_DZ: RC dead\-zone
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

dead zone around trim or bottom


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 200 | pwm   |
+---------+-------+




.. _RC16_FUNCTION:

RC16\_FUNCTION: Servo out function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Disabled\(0\) will setup this output for control by auto missions or MAVLink servo set commands\. any other value will enable the corresponding function


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | RCPassThru           | |
| +-------+----------------------+ |
| | 2     | Flap                 | |
| +-------+----------------------+ |
| | 3     | Flap_auto            | |
| +-------+----------------------+ |
| | 4     | Aileron              | |
| +-------+----------------------+ |
| | 6     | mount_pan            | |
| +-------+----------------------+ |
| | 7     | mount_tilt           | |
| +-------+----------------------+ |
| | 8     | mount_roll           | |
| +-------+----------------------+ |
| | 9     | mount_open           | |
| +-------+----------------------+ |
| | 10    | camera_trigger       | |
| +-------+----------------------+ |
| | 11    | release              | |
| +-------+----------------------+ |
| | 12    | mount2_pan           | |
| +-------+----------------------+ |
| | 13    | mount2_tilt          | |
| +-------+----------------------+ |
| | 14    | mount2_roll          | |
| +-------+----------------------+ |
| | 15    | mount2_open          | |
| +-------+----------------------+ |
| | 16    | DifferentialSpoiler1 | |
| +-------+----------------------+ |
| | 17    | DifferentialSpoiler2 | |
| +-------+----------------------+ |
| | 18    | AileronWithInput     | |
| +-------+----------------------+ |
| | 19    | Elevator             | |
| +-------+----------------------+ |
| | 20    | ElevatorWithInput    | |
| +-------+----------------------+ |
| | 21    | Rudder               | |
| +-------+----------------------+ |
| | 24    | Flaperon1            | |
| +-------+----------------------+ |
| | 25    | Flaperon2            | |
| +-------+----------------------+ |
| | 26    | GroundSteering       | |
| +-------+----------------------+ |
| | 27    | Parachute            | |
| +-------+----------------------+ |
| | 28    | EPM                  | |
| +-------+----------------------+ |
| | 29    | LandingGear          | |
| +-------+----------------------+ |
| | 30    | EngineRunEnable      | |
| +-------+----------------------+ |
| | 31    | HeliRSC              | |
| +-------+----------------------+ |
| | 32    | HeliTailRSC          | |
| +-------+----------------------+ |
| | 33    | Motor1               | |
| +-------+----------------------+ |
| | 34    | Motor2               | |
| +-------+----------------------+ |
| | 35    | Motor3               | |
| +-------+----------------------+ |
| | 36    | Motor4               | |
| +-------+----------------------+ |
| | 37    | Motor5               | |
| +-------+----------------------+ |
| | 38    | Motor6               | |
| +-------+----------------------+ |
| | 39    | Motor7               | |
| +-------+----------------------+ |
| | 40    | Motor8               | |
| +-------+----------------------+ |
| | 51    | RCIN1                | |
| +-------+----------------------+ |
| | 52    | RCIN2                | |
| +-------+----------------------+ |
| | 53    | RCIN3                | |
| +-------+----------------------+ |
| | 54    | RCIN4                | |
| +-------+----------------------+ |
| | 55    | RCIN5                | |
| +-------+----------------------+ |
| | 56    | RCIN6                | |
| +-------+----------------------+ |
| | 57    | RCIN7                | |
| +-------+----------------------+ |
| | 58    | RCIN8                | |
| +-------+----------------------+ |
| | 59    | RCIN9                | |
| +-------+----------------------+ |
| | 60    | RCIN10               | |
| +-------+----------------------+ |
| | 61    | RCIN11               | |
| +-------+----------------------+ |
| | 62    | RCIN12               | |
| +-------+----------------------+ |
| | 63    | RCIN13               | |
| +-------+----------------------+ |
| | 64    | RCIN14               | |
| +-------+----------------------+ |
| | 65    | RCIN15               | |
| +-------+----------------------+ |
| | 66    | RCIN16               | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+





.. _parameters_RLL2SRV_:

RLL2SRV\_ Parameters
--------------------


.. _RLL2SRV_TCONST:

RLL2SRV\_TCONST: Roll Time Constant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the time constant in seconds from demanded to achieved bank angle\. A value of 0\.5 is a good default and will work with nearly all models\. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve\.


+-----------+-----------+---------+
| Range     | Increment | Units   |
+===========+===========+=========+
| 0.4 - 1.0 | 0.1       | seconds |
+-----------+-----------+---------+




.. _RLL2SRV_P:

RLL2SRV\_P: Proportional Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from bank angle error to aileron\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 4.0 | 0.1       |
+-----------+-----------+




.. _RLL2SRV_D:

RLL2SRV\_D: Damping Gain
~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from roll rate to aileron\. This adjusts the damping of the roll control loop\. It has the same effect as RLL2SRV\_D in the old PID controller but without the spikes in servo demands\. This gain helps to reduce rolling in turbulence\. It should be increased in 0\.01 increments as too high a value can lead to a high frequency roll oscillation that could overstress the airframe\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 0.1 | 0.01      |
+---------+-----------+




.. _RLL2SRV_I:

RLL2SRV\_I: Integrator Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from the integral of bank angle to aileron\. It has the same effect as RLL2SRV\_I in the old PID controller\. Increasing this gain causes the controller to trim out steady offsets due to an out of trim aircraft\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 1.0 | 0.05      |
+---------+-----------+




.. _RLL2SRV_RMAX:

RLL2SRV\_RMAX: Maximum Roll Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the maximum roll rate that the controller will demand \(degrees\/sec\)\. Setting it to zero disables the limit\. If this value is set too low\, then the roll can\'t keep up with the navigation demands and the plane will start weaving\. If it is set too high \(or disabled by setting to zero\) then ailerons will get large inputs at the start of turns\. A limit of 60 degrees\/sec is a good default\.


+---------+-----------+----------------+
| Range   | Increment | Units          |
+=========+===========+================+
| 0 - 180 | 1         | degrees/second |
+---------+-----------+----------------+




.. _RLL2SRV_IMAX:

RLL2SRV\_IMAX: Integrator limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This limits the number of degrees of aileron in centi\-degrees over which the integrator will operate\. At the default setting of 3000 centi\-degrees\, the integrator will be limited to \+\- 30 degrees of servo travel\. The maximum servo deflection is \+\- 45 centi\-degrees\, so the default value represents a 2\/3rd of the total control throw which is adequate unless the aircraft is severely out of trim\.


+----------+-----------+
| Range    | Increment |
+==========+===========+
| 0 - 4500 | 1         |
+----------+-----------+




.. _RLL2SRV_FF:

RLL2SRV\_FF: Feed forward Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from demanded rate to aileron output\. 


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 4.0 | 0.1       |
+-----------+-----------+





.. _parameters_PTCH2SRV_:

PTCH2SRV\_ Parameters
---------------------


.. _PTCH2SRV_TCONST:

PTCH2SRV\_TCONST: Pitch Time Constant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the time constant in seconds from demanded to achieved pitch angle\. A value of 0\.5 is a good default and will work with nearly all models\. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve\.


+-----------+-----------+---------+
| Range     | Increment | Units   |
+===========+===========+=========+
| 0.4 - 1.0 | 0.1       | seconds |
+-----------+-----------+---------+




.. _PTCH2SRV_P:

PTCH2SRV\_P: Proportional Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from pitch angle to elevator\. This gain works the same way as PTCH2SRV\_P in the old PID controller and can be set to the same value\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 3.0 | 0.1       |
+-----------+-----------+




.. _PTCH2SRV_D:

PTCH2SRV\_D: Damping Gain
~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from pitch rate to elevator\. This adjusts the damping of the pitch control loop\. It has the same effect as PTCH2SRV\_D in the old PID controller and can be set to the same value\, but without the spikes in servo demands\. This gain helps to reduce pitching in turbulence\. Some airframes such as flying wings that have poor pitch damping can benefit from increasing this gain term\. This should be increased in 0\.01 increments as too high a value can lead to a high frequency pitch oscillation that could overstress the airframe\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 0.1 | 0.01      |
+---------+-----------+




.. _PTCH2SRV_I:

PTCH2SRV\_I: Integrator Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain applied to the integral of pitch angle\. It has the same effect as PTCH2SRV\_I in the old PID controller and can be set to the same value\. Increasing this gain causes the controller to trim out constant offsets between demanded and measured pitch angle\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 0.5 | 0.05      |
+---------+-----------+




.. _PTCH2SRV_RMAX_UP:

PTCH2SRV\_RMAX\_UP: Pitch up max rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the maximum nose up pitch rate that the controller will demand \(degrees\/sec\)\. Setting it to zero disables the limit\.


+---------+-----------+----------------+
| Range   | Increment | Units          |
+=========+===========+================+
| 0 - 100 | 1         | degrees/second |
+---------+-----------+----------------+




.. _PTCH2SRV_RMAX_DN:

PTCH2SRV\_RMAX\_DN: Pitch down max rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the maximum nose down pitch rate that the controller will demand \(degrees\/sec\)\. Setting it to zero disables the limit\.


+---------+-----------+----------------+
| Range   | Increment | Units          |
+=========+===========+================+
| 0 - 100 | 1         | degrees/second |
+---------+-----------+----------------+




.. _PTCH2SRV_RLL:

PTCH2SRV\_RLL: Roll compensation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain term that is applied to the pitch rate offset calculated as required to keep the nose level during turns\. The default value is 1 which will work for all models\. Advanced users can use it to correct for height variation in turns\. If height is lost initially in turns this can be increased in small increments of 0\.05 to compensate\. If height is gained initially in turns then it can be decreased\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.7 - 1.5 | 0.05      |
+-----------+-----------+




.. _PTCH2SRV_IMAX:

PTCH2SRV\_IMAX: Integrator limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This limits the number of centi\-degrees of elevator over which the integrator will operate\. At the default setting of 3000 centi\-degrees\, the integrator will be limited to \+\- 30 degrees of servo travel\. The maximum servo deflection is \+\- 45 degrees\, so the default value represents a 2\/3rd of the total control throw which is adequate for most aircraft unless they are severely out of trim or have very limited elevator control effectiveness\.


+----------+-----------+
| Range    | Increment |
+==========+===========+
| 0 - 4500 | 1         |
+----------+-----------+




.. _PTCH2SRV_FF:

PTCH2SRV\_FF: Feed forward Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from demanded rate to elevator output\. 


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 4.0 | 0.1       |
+-----------+-----------+





.. _parameters_YAW2SRV_:

YAW2SRV\_ Parameters
--------------------


.. _YAW2SRV_SLIP:

YAW2SRV\_SLIP: Sideslip control gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from measured lateral acceleration to demanded yaw rate\. It should be set to zero unless active control of sideslip is desired\. This will only work effectively if there is enough fuselage side area to generate a measureable lateral acceleration when the model sideslips\. Flying wings and most gliders cannot use this term\. This term should only be adjusted after the basic yaw damper gain YAW2SRV\_DAMP is tuned and the YAW2SRV\_INT integrator gain has been set\. Set this gain to zero if only yaw damping is required\.


+-------+-----------+
| Range | Increment |
+=======+===========+
| 0 - 4 | 0.25      |
+-------+-----------+




.. _YAW2SRV_INT:

YAW2SRV\_INT: Sideslip control integrator
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the integral gain from lateral acceleration error\. This gain should only be non\-zero if active control over sideslip is desired\. If active control over sideslip is required then this can be set to 1\.0 as a first try\.


+-------+-----------+
| Range | Increment |
+=======+===========+
| 0 - 2 | 0.25      |
+-------+-----------+




.. _YAW2SRV_DAMP:

YAW2SRV\_DAMP: Yaw damping
~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from yaw rate to rudder\. It acts as a damper on yaw motion\. If a basic yaw damper is required\, this gain term can be incremented\, whilst leaving the YAW2SRV\_SLIP and YAW2SRV\_INT gains at zero\. Note that unlike with a standard PID controller\, if this damping term is zero then the integrator will also be disabled\.


+-------+-----------+
| Range | Increment |
+=======+===========+
| 0 - 2 | 0.25      |
+-------+-----------+




.. _YAW2SRV_RLL:

YAW2SRV\_RLL: Yaw coordination gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain term that is applied to the yaw rate offset calculated as required to keep the yaw rate consistent with the turn rate for a coordinated turn\. The default value is 1 which will work for all models\. Advanced users can use it to correct for any tendency to yaw away from or into the turn once the turn is established\. Increase to make the model yaw more initially and decrease to make the model yaw less initially\. If values greater than 1\.1 or less than 0\.9 are required then it normally indicates a problem with the airspeed calibration\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.8 - 1.2 | 0.05      |
+-----------+-----------+




.. _YAW2SRV_IMAX:

YAW2SRV\_IMAX: Integrator limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This limits the number of centi\-degrees of rudder over which the integrator will operate\. At the default setting of 1500 centi\-degrees\, the integrator will be limited to \+\- 15 degrees of servo travel\. The maximum servo deflection is \+\- 45 degrees\, so the default value represents a 1\/3rd of the total control throw which is adequate for most aircraft unless they are severely out of trim or have very limited rudder control effectiveness\.


+----------+-----------+
| Range    | Increment |
+==========+===========+
| 0 - 4500 | 1         |
+----------+-----------+





.. _parameters_STEER2SRV_:

STEER2SRV\_ Parameters
----------------------


.. _STEER2SRV_TCONST:

STEER2SRV\_TCONST: Steering Time Constant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the time constant in seconds from demanded to achieved steering angle\. A value of 0\.75 is a good default and will work with nearly all rovers\. Ground steering in aircraft needs a bit smaller time constant\, and a value of 0\.5 is recommended for best ground handling in fixed wing aircraft\. A value of 0\.75 means that the controller will try to correct any deviation between the desired and actual steering angle in 0\.75 seconds\. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the vehicle can achieve\.


+-----------+-----------+---------+
| Range     | Increment | Units   |
+===========+===========+=========+
| 0.4 - 1.0 | 0.1       | seconds |
+-----------+-----------+---------+




.. _STEER2SRV_P:

STEER2SRV\_P: Steering turning gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The proportional gain for steering\. This should be approximately equal to the diameter of the turning circle of the vehicle at low speed and maximum steering angle


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.1 - 10.0 | 0.1       |
+------------+-----------+




.. _STEER2SRV_I:

STEER2SRV\_I: Integrator Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the gain from the integral of steering angle\. Increasing this gain causes the controller to trim out steady offsets due to an out of trim vehicle\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 1.0 | 0.05      |
+---------+-----------+




.. _STEER2SRV_D:

STEER2SRV\_D: Damping Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~


This adjusts the damping of the steering control loop\. This gain helps to reduce steering jitter with vibration\. It should be increased in 0\.01 increments as too high a value can lead to a high frequency steering oscillation that could overstress the vehicle\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 0.1 | 0.01      |
+---------+-----------+




.. _STEER2SRV_IMAX:

STEER2SRV\_IMAX: Integrator limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This limits the number of degrees of steering in centi\-degrees over which the integrator will operate\. At the default setting of 1500 centi\-degrees\, the integrator will be limited to \+\- 15 degrees of servo travel\. The maximum servo deflection is \+\- 45 centi\-degrees\, so the default value represents a 1\/3rd of the total control throw which is adequate unless the vehicle is severely out of trim\.


+----------+-----------+
| Range    | Increment |
+==========+===========+
| 0 - 4500 | 1         |
+----------+-----------+




.. _STEER2SRV_MINSPD:

STEER2SRV\_MINSPD: Minimum speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the minimum assumed ground speed in meters\/second for steering\. Having a minimum speed prevents oscillations when the vehicle first starts moving\. The vehicle can still drive slower than this limit\, but the steering calculations will be done based on this minimum speed\.


+-------+-----------+-------+
| Range | Increment | Units |
+=======+===========+=======+
| 0 - 5 | 0.1       | m/s   |
+-------+-----------+-------+




.. _STEER2SRV_FF:

STEER2SRV\_FF: Steering feed forward
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The feed forward gain for steering this is the ratio of the achieved turn rate to applied steering\. A value of 1 means that the vehicle would yaw at a rate of 45 degrees per second with full steering deflection at 1m\/s ground speed\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.0 - 10.0 | 0.1       |
+------------+-----------+





.. _parameters_COMPASS_:

COMPASS\_ Parameters
--------------------


.. _COMPASS_OFS_X:

COMPASS\_OFS\_X: Compass offsets in milligauss on the X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to the compass x\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_OFS_Y:

COMPASS\_OFS\_Y: Compass offsets in milligauss on the Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to the compass y\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_OFS_Z:

COMPASS\_OFS\_Z: Compass offsets in milligauss on the Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to the compass z\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_DEC:

COMPASS\_DEC: Compass declination
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


An angle to compensate between the true north and magnetic north


+----------------+-----------+---------+
| Range          | Increment | Units   |
+================+===========+=========+
| -3.142 - 3.142 | 0.01      | Radians |
+----------------+-----------+---------+




.. _COMPASS_LEARN:

COMPASS\_LEARN: Learn compass offsets automatically
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the automatic learning of compass offsets\. You can enable learning either using a compass\-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator\. If this option is enabled then the learnt offsets are saved when you disarm the vehicle\.


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | Disabled          | |
| +-------+-------------------+ |
| | 1     | Internal-Learning | |
| +-------+-------------------+ |
| | 2     | EKF-Learning      | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _COMPASS_USE:

COMPASS\_USE: Use compass for yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the use of the compass \(instead of the GPS\) for determining heading


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_AUTODEC:

COMPASS\_AUTODEC: Auto Declination
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the automatic calculation of the declination based on gps location


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_MOTCT:

COMPASS\_MOTCT: Motor interference compensation type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Set motor interference compensation type to disabled\, throttle or current\.  Do not change manually\.


+--------------------------+
| Values                   |
+==========================+
| +-------+--------------+ |
| | Value | Meaning      | |
| +=======+==============+ |
| | 0     | Disabled     | |
| +-------+--------------+ |
| | 1     | Use Throttle | |
| +-------+--------------+ |
| | 2     | Use Current  | |
| +-------+--------------+ |
|                          |
+--------------------------+




.. _COMPASS_MOT_X:

COMPASS\_MOT\_X: Motor interference compensation for body frame X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to the compass\'s x\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_MOT_Y:

COMPASS\_MOT\_Y: Motor interference compensation for body frame Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to the compass\'s y\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_MOT_Z:

COMPASS\_MOT\_Z: Motor interference compensation for body frame Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to the compass\'s z\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_ORIENT:

COMPASS\_ORIENT: Compass orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The orientation of the compass relative to the autopilot board\. This will default to the right value for each board type\, but can be changed if you have an external compass\. See the documentation for your external compass for the right value\. The correct orientation should give the X axis forward\, the Y axis to the right and the Z axis down\. So if your aircraft is pointing west it should show a positive value for the Y axis\, and a value close to zero for the X axis\. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented\. NOTE\: This orientation is combined with any AHRS\_ORIENTATION setting\.


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Yaw45               | |
| +-------+---------------------+ |
| | 2     | Yaw90               | |
| +-------+---------------------+ |
| | 3     | Yaw135              | |
| +-------+---------------------+ |
| | 4     | Yaw180              | |
| +-------+---------------------+ |
| | 5     | Yaw225              | |
| +-------+---------------------+ |
| | 6     | Yaw270              | |
| +-------+---------------------+ |
| | 7     | Yaw315              | |
| +-------+---------------------+ |
| | 8     | Roll180             | |
| +-------+---------------------+ |
| | 9     | Roll180Yaw45        | |
| +-------+---------------------+ |
| | 10    | Roll180Yaw90        | |
| +-------+---------------------+ |
| | 11    | Roll180Yaw135       | |
| +-------+---------------------+ |
| | 12    | Pitch180            | |
| +-------+---------------------+ |
| | 13    | Roll180Yaw225       | |
| +-------+---------------------+ |
| | 14    | Roll180Yaw270       | |
| +-------+---------------------+ |
| | 15    | Roll180Yaw315       | |
| +-------+---------------------+ |
| | 16    | Roll90              | |
| +-------+---------------------+ |
| | 17    | Roll90Yaw45         | |
| +-------+---------------------+ |
| | 18    | Roll90Yaw90         | |
| +-------+---------------------+ |
| | 19    | Roll90Yaw135        | |
| +-------+---------------------+ |
| | 20    | Roll270             | |
| +-------+---------------------+ |
| | 21    | Roll270Yaw45        | |
| +-------+---------------------+ |
| | 22    | Roll270Yaw90        | |
| +-------+---------------------+ |
| | 23    | Roll270Yaw136       | |
| +-------+---------------------+ |
| | 24    | Pitch90             | |
| +-------+---------------------+ |
| | 25    | Pitch270            | |
| +-------+---------------------+ |
| | 26    | Pitch180Yaw90       | |
| +-------+---------------------+ |
| | 27    | Pitch180Yaw270      | |
| +-------+---------------------+ |
| | 28    | Roll90Pitch90       | |
| +-------+---------------------+ |
| | 29    | Roll180Pitch90      | |
| +-------+---------------------+ |
| | 30    | Roll270Pitch90      | |
| +-------+---------------------+ |
| | 31    | Roll90Pitch180      | |
| +-------+---------------------+ |
| | 32    | Roll270Pitch180     | |
| +-------+---------------------+ |
| | 33    | Roll90Pitch270      | |
| +-------+---------------------+ |
| | 34    | Roll180Pitch270     | |
| +-------+---------------------+ |
| | 35    | Roll270Pitch270     | |
| +-------+---------------------+ |
| | 36    | Roll90Pitch180Yaw90 | |
| +-------+---------------------+ |
| | 37    | Roll90Yaw270        | |
| +-------+---------------------+ |
| | 38    | Yaw293Pitch68Roll90 | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _COMPASS_EXTERNAL:

COMPASS\_EXTERNAL: Compass is attached via an external cable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Configure compass so it is attached externally\. This is auto\-detected on PX4 and Pixhawk\. Set to 1 if the compass is externally connected\. When externally connected the COMPASS\_ORIENT option operates independently of the AHRS\_ORIENTATION board orientation option\. If set to 0 or 1 then auto\-detection by bus connection can override the value\. If set to 2 then auto\-detection will be disabled\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | 0     | Internal       | |
| +-------+----------------+ |
| | 1     | External       | |
| +-------+----------------+ |
| | 2     | ForcedExternal | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _COMPASS_OFS2_X:

COMPASS\_OFS2\_X: Compass2 offsets in milligauss on the X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to compass2\'s x\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_OFS2_Y:

COMPASS\_OFS2\_Y: Compass2 offsets in milligauss on the Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to compass2\'s y\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_OFS2_Z:

COMPASS\_OFS2\_Z: Compass2 offsets in milligauss on the Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to compass2\'s z\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_MOT2_X:

COMPASS\_MOT2\_X: Motor interference compensation to compass2 for body frame X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to compass2\'s x\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_MOT2_Y:

COMPASS\_MOT2\_Y: Motor interference compensation to compass2 for body frame Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to compass2\'s y\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_MOT2_Z:

COMPASS\_MOT2\_Z: Motor interference compensation to compass2 for body frame Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to compass2\'s z\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_PRIMARY:

COMPASS\_PRIMARY: Choose primary compass
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If more than one compass is available this selects which compass is the primary\. Normally 0\=External\, 1\=Internal\. If no External compass is attached this parameter is ignored


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | FirstCompass  | |
| +-------+---------------+ |
| | 1     | SecondCompass | |
| +-------+---------------+ |
| | 2     | ThirdCompass  | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _COMPASS_OFS3_X:

COMPASS\_OFS3\_X: Compass3 offsets in milligauss on the X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to compass3\'s x\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_OFS3_Y:

COMPASS\_OFS3\_Y: Compass3 offsets in milligauss on the Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to compass3\'s y\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_OFS3_Z:

COMPASS\_OFS3\_Z: Compass3 offsets in milligauss on the Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to be added to compass3\'s z\-axis values to compensate for metal in the frame


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| -400 - 400 | 1         | milligauss |
+------------+-----------+------------+




.. _COMPASS_MOT3_X:

COMPASS\_MOT3\_X: Motor interference compensation to compass3 for body frame X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to compass3\'s x\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_MOT3_Y:

COMPASS\_MOT3\_Y: Motor interference compensation to compass3 for body frame Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to compass3\'s y\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_MOT3_Z:

COMPASS\_MOT3\_Z: Motor interference compensation to compass3 for body frame Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Multiplied by the current throttle and added to compass3\'s z\-axis values to compensate for motor interference


+--------------+-----------+------------------------------------+
| Range        | Increment | Units                              |
+==============+===========+====================================+
| -1000 - 1000 | 1         | Offset per Amp or at Full Throttle |
+--------------+-----------+------------------------------------+




.. _COMPASS_DEV_ID:

COMPASS\_DEV\_ID: Compass device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Compass device id\.  Automatically detected\, do not set manually


.. _COMPASS_DEV_ID2:

COMPASS\_DEV\_ID2: Compass2 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Second compass\'s device id\.  Automatically detected\, do not set manually


.. _COMPASS_DEV_ID3:

COMPASS\_DEV\_ID3: Compass3 device id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Third compass\'s device id\.  Automatically detected\, do not set manually


.. _COMPASS_USE2:

COMPASS\_USE2: Compass2 used for yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the second compass for determining heading\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_ORIENT2:

COMPASS\_ORIENT2: Compass2 orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The orientation of the second compass relative to the frame \(if external\) or autopilot board \(if internal\)\.


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Yaw45               | |
| +-------+---------------------+ |
| | 2     | Yaw90               | |
| +-------+---------------------+ |
| | 3     | Yaw135              | |
| +-------+---------------------+ |
| | 4     | Yaw180              | |
| +-------+---------------------+ |
| | 5     | Yaw225              | |
| +-------+---------------------+ |
| | 6     | Yaw270              | |
| +-------+---------------------+ |
| | 7     | Yaw315              | |
| +-------+---------------------+ |
| | 8     | Roll180             | |
| +-------+---------------------+ |
| | 9     | Roll180Yaw45        | |
| +-------+---------------------+ |
| | 10    | Roll180Yaw90        | |
| +-------+---------------------+ |
| | 11    | Roll180Yaw135       | |
| +-------+---------------------+ |
| | 12    | Pitch180            | |
| +-------+---------------------+ |
| | 13    | Roll180Yaw225       | |
| +-------+---------------------+ |
| | 14    | Roll180Yaw270       | |
| +-------+---------------------+ |
| | 15    | Roll180Yaw315       | |
| +-------+---------------------+ |
| | 16    | Roll90              | |
| +-------+---------------------+ |
| | 17    | Roll90Yaw45         | |
| +-------+---------------------+ |
| | 18    | Roll90Yaw90         | |
| +-------+---------------------+ |
| | 19    | Roll90Yaw135        | |
| +-------+---------------------+ |
| | 20    | Roll270             | |
| +-------+---------------------+ |
| | 21    | Roll270Yaw45        | |
| +-------+---------------------+ |
| | 22    | Roll270Yaw90        | |
| +-------+---------------------+ |
| | 23    | Roll270Yaw136       | |
| +-------+---------------------+ |
| | 24    | Pitch90             | |
| +-------+---------------------+ |
| | 25    | Pitch270            | |
| +-------+---------------------+ |
| | 26    | Pitch180Yaw90       | |
| +-------+---------------------+ |
| | 27    | Pitch180Yaw270      | |
| +-------+---------------------+ |
| | 28    | Roll90Pitch90       | |
| +-------+---------------------+ |
| | 29    | Roll180Pitch90      | |
| +-------+---------------------+ |
| | 30    | Roll270Pitch90      | |
| +-------+---------------------+ |
| | 31    | Roll90Pitch180      | |
| +-------+---------------------+ |
| | 32    | Roll270Pitch180     | |
| +-------+---------------------+ |
| | 33    | Roll90Pitch270      | |
| +-------+---------------------+ |
| | 34    | Roll180Pitch270     | |
| +-------+---------------------+ |
| | 35    | Roll270Pitch270     | |
| +-------+---------------------+ |
| | 36    | Roll90Pitch180Yaw90 | |
| +-------+---------------------+ |
| | 37    | Roll90Yaw270        | |
| +-------+---------------------+ |
| | 38    | Yaw293Pitch68Roll90 | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _COMPASS_EXTERN2:

COMPASS\_EXTERN2: Compass2 is attached via an external cable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Configure second compass so it is attached externally\. This is auto\-detected on PX4 and Pixhawk\. If set to 0 or 1 then auto\-detection by bus connection can override the value\. If set to 2 then auto\-detection will be disabled\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | 0     | Internal       | |
| +-------+----------------+ |
| | 1     | External       | |
| +-------+----------------+ |
| | 2     | ForcedExternal | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _COMPASS_USE3:

COMPASS\_USE3: Compass3 used for yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the third compass for determining heading\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _COMPASS_ORIENT3:

COMPASS\_ORIENT3: Compass3 orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The orientation of the third compass relative to the frame \(if external\) or autopilot board \(if internal\)\.


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Yaw45               | |
| +-------+---------------------+ |
| | 2     | Yaw90               | |
| +-------+---------------------+ |
| | 3     | Yaw135              | |
| +-------+---------------------+ |
| | 4     | Yaw180              | |
| +-------+---------------------+ |
| | 5     | Yaw225              | |
| +-------+---------------------+ |
| | 6     | Yaw270              | |
| +-------+---------------------+ |
| | 7     | Yaw315              | |
| +-------+---------------------+ |
| | 8     | Roll180             | |
| +-------+---------------------+ |
| | 9     | Roll180Yaw45        | |
| +-------+---------------------+ |
| | 10    | Roll180Yaw90        | |
| +-------+---------------------+ |
| | 11    | Roll180Yaw135       | |
| +-------+---------------------+ |
| | 12    | Pitch180            | |
| +-------+---------------------+ |
| | 13    | Roll180Yaw225       | |
| +-------+---------------------+ |
| | 14    | Roll180Yaw270       | |
| +-------+---------------------+ |
| | 15    | Roll180Yaw315       | |
| +-------+---------------------+ |
| | 16    | Roll90              | |
| +-------+---------------------+ |
| | 17    | Roll90Yaw45         | |
| +-------+---------------------+ |
| | 18    | Roll90Yaw90         | |
| +-------+---------------------+ |
| | 19    | Roll90Yaw135        | |
| +-------+---------------------+ |
| | 20    | Roll270             | |
| +-------+---------------------+ |
| | 21    | Roll270Yaw45        | |
| +-------+---------------------+ |
| | 22    | Roll270Yaw90        | |
| +-------+---------------------+ |
| | 23    | Roll270Yaw136       | |
| +-------+---------------------+ |
| | 24    | Pitch90             | |
| +-------+---------------------+ |
| | 25    | Pitch270            | |
| +-------+---------------------+ |
| | 26    | Pitch180Yaw90       | |
| +-------+---------------------+ |
| | 27    | Pitch180Yaw270      | |
| +-------+---------------------+ |
| | 28    | Roll90Pitch90       | |
| +-------+---------------------+ |
| | 29    | Roll180Pitch90      | |
| +-------+---------------------+ |
| | 30    | Roll270Pitch90      | |
| +-------+---------------------+ |
| | 31    | Roll90Pitch180      | |
| +-------+---------------------+ |
| | 32    | Roll270Pitch180     | |
| +-------+---------------------+ |
| | 33    | Roll90Pitch270      | |
| +-------+---------------------+ |
| | 34    | Roll180Pitch270     | |
| +-------+---------------------+ |
| | 35    | Roll270Pitch270     | |
| +-------+---------------------+ |
| | 36    | Roll90Pitch180Yaw90 | |
| +-------+---------------------+ |
| | 37    | Roll90Yaw270        | |
| +-------+---------------------+ |
| | 38    | Yaw293Pitch68Roll90 | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _COMPASS_EXTERN3:

COMPASS\_EXTERN3: Compass3 is attached via an external cable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Configure third compass so it is attached externally\. This is auto\-detected on PX4 and Pixhawk\. If set to 0 or 1 then auto\-detection by bus connection can override the value\. If set to 2 then auto\-detection will be disabled\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | 0     | Internal       | |
| +-------+----------------+ |
| | 1     | External       | |
| +-------+----------------+ |
| | 2     | ForcedExternal | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _COMPASS_DIA_X:

COMPASS\_DIA\_X: Compass soft\-iron diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_X in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA_Y:

COMPASS\_DIA\_Y: Compass soft\-iron diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_Y in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA_Z:

COMPASS\_DIA\_Z: Compass soft\-iron diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_Z in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI_X:

COMPASS\_ODI\_X: Compass soft\-iron off\-diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_X in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI_Y:

COMPASS\_ODI\_Y: Compass soft\-iron off\-diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_Y in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI_Z:

COMPASS\_ODI\_Z: Compass soft\-iron off\-diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_Z in the compass soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA2_X:

COMPASS\_DIA2\_X: Compass2 soft\-iron diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_X in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA2_Y:

COMPASS\_DIA2\_Y: Compass2 soft\-iron diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_Y in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA2_Z:

COMPASS\_DIA2\_Z: Compass2 soft\-iron diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_Z in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI2_X:

COMPASS\_ODI2\_X: Compass2 soft\-iron off\-diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_X in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI2_Y:

COMPASS\_ODI2\_Y: Compass2 soft\-iron off\-diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_Y in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI2_Z:

COMPASS\_ODI2\_Z: Compass2 soft\-iron off\-diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_Z in the compass2 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA3_X:

COMPASS\_DIA3\_X: Compass3 soft\-iron diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_X in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA3_Y:

COMPASS\_DIA3\_Y: Compass3 soft\-iron diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_Y in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_DIA3_Z:

COMPASS\_DIA3\_Z: Compass3 soft\-iron diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DIA\_Z in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI3_X:

COMPASS\_ODI3\_X: Compass3 soft\-iron off\-diagonal X component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_X in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI3_Y:

COMPASS\_ODI3\_Y: Compass3 soft\-iron off\-diagonal Y component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_Y in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_ODI3_Z:

COMPASS\_ODI3\_Z: Compass3 soft\-iron off\-diagonal Z component
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ODI\_Z in the compass3 soft\-iron calibration matrix\: \[\[DIA\_X\, ODI\_X\, ODI\_Y\]\, \[ODI\_X\, DIA\_Y\, ODI\_Z\]\, \[ODI\_Y\, ODI\_Z\, DIA\_Z\]\]


.. _COMPASS_CAL_FIT:

COMPASS\_CAL\_FIT: Compass calibration fitness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the fitness level required for a successful compass calibration\. A lower value makes for a stricter fit \(less likely to pass\)\. This is the value used for the primary magnetometer\. Other magnetometers get double the value\.


+--------+-----------+
| Range  | Increment |
+========+===========+
| 4 - 20 | 0.1       |
+--------+-----------+





.. _parameters_SCHED_:

SCHED\_ Parameters
------------------


.. _SCHED_DEBUG:

SCHED\_DEBUG: Scheduler debug level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Set to non\-zero to enable scheduler debug messages\. When set to show \"Slips\" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load\. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table\.


+--------------------------+
| Values                   |
+==========================+
| +-------+--------------+ |
| | Value | Meaning      | |
| +=======+==============+ |
| | 0     | Disabled     | |
| +-------+--------------+ |
| | 2     | ShowSlips    | |
| +-------+--------------+ |
| | 3     | ShowOverruns | |
| +-------+--------------+ |
|                          |
+--------------------------+




.. _SCHED_LOOP_RATE:

SCHED\_LOOP\_RATE: Scheduling main loop rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the rate of the main control loop in Hz\. This should only be changed by developers\. This only takes effect on restart


+---------------------+----------------+
| Values              | RebootRequired |
+=====================+================+
| +-------+---------+ | True           |
| | Value | Meaning | |                |
| +=======+=========+ |                |
| | 50    | 50Hz    | |                |
| +-------+---------+ |                |
| | 100   | 100Hz   | |                |
| +-------+---------+ |                |
| | 200   | 200Hz   | |                |
| +-------+---------+ |                |
| | 250   | 250Hz   | |                |
| +-------+---------+ |                |
| | 300   | 300Hz   | |                |
| +-------+---------+ |                |
| | 400   | 400Hz   | |                |
| +-------+---------+ |                |
|                     |                |
+---------------------+----------------+





.. _parameters_RCMAP_:

RCMAP\_ Parameters
------------------


.. _RCMAP_ROLL:

RCMAP\_ROLL: Roll channel
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Roll channel number\. This is useful when you have a RC transmitter that can\'t change the channel order easily\. Roll is normally on channel 1\, but you can move it to any channel with this parameter\.  Reboot is required for changes to take effect\.


+-------+-----------+----------------+
| Range | Increment | RebootRequired |
+=======+===========+================+
| 1 - 8 | 1         | True           |
+-------+-----------+----------------+




.. _RCMAP_PITCH:

RCMAP\_PITCH: Pitch channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Pitch channel number\. This is useful when you have a RC transmitter that can\'t change the channel order easily\. Pitch is normally on channel 2\, but you can move it to any channel with this parameter\.  Reboot is required for changes to take effect\.


+-------+-----------+----------------+
| Range | Increment | RebootRequired |
+=======+===========+================+
| 1 - 8 | 1         | True           |
+-------+-----------+----------------+




.. _RCMAP_THROTTLE:

RCMAP\_THROTTLE: Throttle channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Throttle channel number\. This is useful when you have a RC transmitter that can\'t change the channel order easily\. Throttle is normally on channel 3\, but you can move it to any channel with this parameter\. Warning APM 2\.X\: Changing the throttle channel could produce unexpected fail\-safe results if connection between receiver and on\-board PPM Encoder is lost\. Disabling on\-board PPM Encoder is recommended\.  Reboot is required for changes to take effect\.


+-------+-----------+----------------+
| Range | Increment | RebootRequired |
+=======+===========+================+
| 1 - 8 | 1         | True           |
+-------+-----------+----------------+




.. _RCMAP_YAW:

RCMAP\_YAW: Yaw channel
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Yaw channel number\. This is useful when you have a RC transmitter that can\'t change the channel order easily\. Yaw \(also known as rudder\) is normally on channel 4\, but you can move it to any channel with this parameter\.  Reboot is required for changes to take effect\.


+-------+-----------+----------------+
| Range | Increment | RebootRequired |
+=======+===========+================+
| 1 - 8 | 1         | True           |
+-------+-----------+----------------+





.. _parameters_SR0_:

SR0\_ Parameters
----------------


.. _SR0_RAW_SENS:

SR0\_RAW\_SENS: Raw sensor stream rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Raw sensor stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_EXT_STAT:

SR0\_EXT\_STAT: Extended status stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extended status stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_RC_CHAN:

SR0\_RC\_CHAN: RC Channel stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC Channel stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_RAW_CTRL:

SR0\_RAW\_CTRL: Raw Control stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Raw Control stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_POSITION:

SR0\_POSITION: Position stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Position stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_EXTRA1:

SR0\_EXTRA1: Extra data type 1 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 1 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_EXTRA2:

SR0\_EXTRA2: Extra data type 2 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 2 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_EXTRA3:

SR0\_EXTRA3: Extra data type 3 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 3 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_PARAMS:

SR0\_PARAMS: Parameter stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Parameter stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+





.. _parameters_SR1_:

SR1\_ Parameters
----------------


.. _SR1_RAW_SENS:

SR1\_RAW\_SENS: Raw sensor stream rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Raw sensor stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_EXT_STAT:

SR1\_EXT\_STAT: Extended status stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extended status stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_RC_CHAN:

SR1\_RC\_CHAN: RC Channel stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC Channel stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_RAW_CTRL:

SR1\_RAW\_CTRL: Raw Control stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Raw Control stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_POSITION:

SR1\_POSITION: Position stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Position stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_EXTRA1:

SR1\_EXTRA1: Extra data type 1 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 1 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_EXTRA2:

SR1\_EXTRA2: Extra data type 2 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 2 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_EXTRA3:

SR1\_EXTRA3: Extra data type 3 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 3 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_PARAMS:

SR1\_PARAMS: Parameter stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Parameter stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+





.. _parameters_SR2_:

SR2\_ Parameters
----------------


.. _SR2_RAW_SENS:

SR2\_RAW\_SENS: Raw sensor stream rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Raw sensor stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_EXT_STAT:

SR2\_EXT\_STAT: Extended status stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extended status stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_RC_CHAN:

SR2\_RC\_CHAN: RC Channel stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC Channel stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_RAW_CTRL:

SR2\_RAW\_CTRL: Raw Control stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Raw Control stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_POSITION:

SR2\_POSITION: Position stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Position stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_EXTRA1:

SR2\_EXTRA1: Extra data type 1 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 1 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_EXTRA2:

SR2\_EXTRA2: Extra data type 2 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 2 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_EXTRA3:

SR2\_EXTRA3: Extra data type 3 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 3 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_PARAMS:

SR2\_PARAMS: Parameter stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Parameter stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+





.. _parameters_SR3_:

SR3\_ Parameters
----------------


.. _SR3_RAW_SENS:

SR3\_RAW\_SENS: Raw sensor stream rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Raw sensor stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_EXT_STAT:

SR3\_EXT\_STAT: Extended status stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extended status stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_RC_CHAN:

SR3\_RC\_CHAN: RC Channel stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

RC Channel stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_RAW_CTRL:

SR3\_RAW\_CTRL: Raw Control stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Raw Control stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_POSITION:

SR3\_POSITION: Position stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Position stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_EXTRA1:

SR3\_EXTRA1: Extra data type 1 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 1 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_EXTRA2:

SR3\_EXTRA2: Extra data type 2 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 2 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_EXTRA3:

SR3\_EXTRA3: Extra data type 3 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Extra data type 3 stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_PARAMS:

SR3\_PARAMS: Parameter stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Parameter stream rate to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+





.. _parameters_INS_:

INS\_ Parameters
----------------


.. _INS_PRODUCT_ID:

INS\_PRODUCT\_ID: IMU Product ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Which type of IMU is installed \(read\-only\)\.


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Unknown   | |
| +-------+-----------+ |
| | 1     | APM1-1280 | |
| +-------+-----------+ |
| | 2     | APM1-2560 | |
| +-------+-----------+ |
| | 88    | APM2      | |
| +-------+-----------+ |
| | 3     | SITL      | |
| +-------+-----------+ |
| | 4     | PX4v1     | |
| +-------+-----------+ |
| | 5     | PX4v2     | |
| +-------+-----------+ |
| | 256   | Flymaple  | |
| +-------+-----------+ |
| | 257   | Linux     | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _INS_GYROFFS_X:

INS\_GYROFFS\_X: Gyro offsets of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro sensor offsets of X axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_GYROFFS_Y:

INS\_GYROFFS\_Y: Gyro offsets of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro sensor offsets of Y axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_GYROFFS_Z:

INS\_GYROFFS\_Z: Gyro offsets of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro sensor offsets of Z axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_GYR2OFFS_X:

INS\_GYR2OFFS\_X: Gyro2 offsets of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro2 sensor offsets of X axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_GYR2OFFS_Y:

INS\_GYR2OFFS\_Y: Gyro2 offsets of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro2 sensor offsets of Y axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_GYR2OFFS_Z:

INS\_GYR2OFFS\_Z: Gyro2 offsets of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro2 sensor offsets of Z axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_GYR3OFFS_X:

INS\_GYR3OFFS\_X: Gyro3 offsets of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro3 sensor offsets of X axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_GYR3OFFS_Y:

INS\_GYR3OFFS\_Y: Gyro3 offsets of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro3 sensor offsets of Y axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_GYR3OFFS_Z:

INS\_GYR3OFFS\_Z: Gyro3 offsets of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Gyro3 sensor offsets of Z axis\. This is setup on each boot during gyro calibrations


+-------+
| Units |
+=======+
| rad/s |
+-------+




.. _INS_ACCSCAL_X:

INS\_ACCSCAL\_X: Accelerometer scaling of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer scaling of X axis\.  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACCSCAL_Y:

INS\_ACCSCAL\_Y: Accelerometer scaling of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer scaling of Y axis  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACCSCAL_Z:

INS\_ACCSCAL\_Z: Accelerometer scaling of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer scaling of Z axis  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACCOFFS_X:

INS\_ACCOFFS\_X: Accelerometer offsets of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer offsets of X axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_ACCOFFS_Y:

INS\_ACCOFFS\_Y: Accelerometer offsets of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer offsets of Y axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_ACCOFFS_Z:

INS\_ACCOFFS\_Z: Accelerometer offsets of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer offsets of Z axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_ACC2SCAL_X:

INS\_ACC2SCAL\_X: Accelerometer2 scaling of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer2 scaling of X axis\.  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACC2SCAL_Y:

INS\_ACC2SCAL\_Y: Accelerometer2 scaling of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer2 scaling of Y axis  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACC2SCAL_Z:

INS\_ACC2SCAL\_Z: Accelerometer2 scaling of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer2 scaling of Z axis  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACC2OFFS_X:

INS\_ACC2OFFS\_X: Accelerometer2 offsets of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer2 offsets of X axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_ACC2OFFS_Y:

INS\_ACC2OFFS\_Y: Accelerometer2 offsets of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer2 offsets of Y axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_ACC2OFFS_Z:

INS\_ACC2OFFS\_Z: Accelerometer2 offsets of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer2 offsets of Z axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_ACC3SCAL_X:

INS\_ACC3SCAL\_X: Accelerometer3 scaling of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer3 scaling of X axis\.  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACC3SCAL_Y:

INS\_ACC3SCAL\_Y: Accelerometer3 scaling of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer3 scaling of Y axis  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACC3SCAL_Z:

INS\_ACC3SCAL\_Z: Accelerometer3 scaling of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer3 scaling of Z axis  Calculated during acceleration calibration routine


+-----------+
| Range     |
+===========+
| 0.8 - 1.2 |
+-----------+




.. _INS_ACC3OFFS_X:

INS\_ACC3OFFS\_X: Accelerometer3 offsets of X axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer3 offsets of X axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_ACC3OFFS_Y:

INS\_ACC3OFFS\_Y: Accelerometer3 offsets of Y axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer3 offsets of Y axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_ACC3OFFS_Z:

INS\_ACC3OFFS\_Z: Accelerometer3 offsets of Z axis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Accelerometer3 offsets of Z axis\. This is setup using the acceleration calibration or level operations


+------------+-------+
| Range      | Units |
+============+=======+
| -3.5 - 3.5 | m/s/s |
+------------+-------+




.. _INS_GYRO_FILTER:

INS\_GYRO\_FILTER: Gyro filter cutoff frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Filter cutoff frequency for gyroscopes\. This can be set to a lower value to try to cope with very high vibration levels in aircraft\. This option takes effect on the next reboot\. A value of zero means no filtering \(not recommended\!\)


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 127 | Hz    |
+---------+-------+




.. _INS_ACCEL_FILTER:

INS\_ACCEL\_FILTER: Accel filter cutoff frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Filter cutoff frequency for accelerometers\. This can be set to a lower value to try to cope with very high vibration levels in aircraft\. This option takes effect on the next reboot\. A value of zero means no filtering \(not recommended\!\)


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 - 127 | Hz    |
+---------+-------+




.. _INS_USE:

INS\_USE: Use first IMU for attitude\, velocity and position estimates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Use first IMU for attitude\, velocity and position estimates


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _INS_USE2:

INS\_USE2: Use second IMU for attitude\, velocity and position estimates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Use second IMU for attitude\, velocity and position estimates


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _INS_USE3:

INS\_USE3: Use third IMU for attitude\, velocity and position estimates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Use third IMU for attitude\, velocity and position estimates


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _INS_STILL_THRESH:

INS\_STILL\_THRESH: Stillness threshold for detecting if we are moving
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Threshold to tolerate vibration to determine if vehicle is motionless\. This depends on the frame type and if there is a constant vibration due to motors before launch or after landing\. Total motionless is about 0\.05\. Suggested values\: Planes\/rover use 0\.1\, multirotors use 1\, tradHeli uses 5


+-----------+
| Range     |
+===========+
| 0.05 - 50 |
+-----------+




.. _INS_GYR_CAL:

INS\_GYR\_CAL: Gyro Calibration scheme
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Conrols when automatic gyro calibration is performed


+---------------------------+
| Values                    |
+===========================+
| +-------+---------------+ |
| | Value | Meaning       | |
| +=======+===============+ |
| | 0     | Never         | |
| +-------+---------------+ |
| | 1     | Start-up only | |
| +-------+---------------+ |
|                           |
+---------------------------+




.. _INS_TRIM_OPTION:

INS\_TRIM\_OPTION: Accel cal trim option
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Specifies how the accel cal routine determines the trims


+--------------------------------------------------------------------+
| Values                                                             |
+====================================================================+
| +-------+--------------------------------------------------------+ |
| | Value | Meaning                                                | |
| +=======+========================================================+ |
| | 0     | Don't adjust the trims                                 | |
| +-------+--------------------------------------------------------+ |
| | 1     | Assume first orientation was level                     | |
| +-------+--------------------------------------------------------+ |
| | 2     | Assume ACC_BODYFIX is perfectly aligned to the vehicle | |
| +-------+--------------------------------------------------------+ |
|                                                                    |
+--------------------------------------------------------------------+




.. _INS_ACC_BODYFIX:

INS\_ACC\_BODYFIX: Body\-fixed accelerometer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The body\-fixed accelerometer to be used for trim calculation


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 1     | IMU 1   | |
| +-------+---------+ |
| | 2     | IMU 2   | |
| +-------+---------+ |
| | 3     | IMU 3   | |
| +-------+---------+ |
|                     |
+---------------------+





.. _parameters_AHRS_:

AHRS\_ Parameters
-----------------


.. _AHRS_GPS_GAIN:

AHRS\_GPS\_GAIN: AHRS GPS gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how how much to use the GPS to correct the attitude\. This should never be set to zero for a plane as it would result in the plane losing control in turns\. For a plane please use the default value of 1\.0\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 1.0 | .01       |
+-----------+-----------+




.. _AHRS_GPS_USE:

AHRS\_GPS\_USE: AHRS use GPS for navigation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls whether to use dead\-reckoning or GPS based navigation\. If set to 0 then the GPS won\'t be used for navigation\, and only dead reckoning will be used\. A value of zero should never be used for normal flight\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _AHRS_YAW_P:

AHRS\_YAW\_P: Yaw P
~~~~~~~~~~~~~~~~~~~


This controls the weight the compass or GPS has on the heading\. A higher value means the heading will track the yaw source \(GPS or compass\) more rapidly\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 0.4 | .01       |
+-----------+-----------+




.. _AHRS_RP_P:

AHRS\_RP\_P: AHRS RP\_P
~~~~~~~~~~~~~~~~~~~~~~~


This controls how fast the accelerometers correct the attitude


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 0.4 | .01       |
+-----------+-----------+




.. _AHRS_WIND_MAX:

AHRS\_WIND\_MAX: Maximum wind
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum allowable difference between ground speed and airspeed\. This allows the plane to cope with a failing airspeed sensor\. A value of zero means to use the airspeed as is\.


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 127 | 1         | m/s   |
+---------+-----------+-------+




.. _AHRS_TRIM_X:

AHRS\_TRIM\_X: AHRS Trim Roll
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Compensates for the roll angle difference between the control board and the frame\. Positive values make the vehicle roll right\.


+-------------------+-----------+---------+
| Range             | Increment | Units   |
+===================+===========+=========+
| -0.1745 - +0.1745 | 0.01      | Radians |
+-------------------+-----------+---------+




.. _AHRS_TRIM_Y:

AHRS\_TRIM\_Y: AHRS Trim Pitch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Compensates for the pitch angle difference between the control board and the frame\. Positive values make the vehicle pitch up\/back\.


+-------------------+-----------+---------+
| Range             | Increment | Units   |
+===================+===========+=========+
| -0.1745 - +0.1745 | 0.01      | Radians |
+-------------------+-----------+---------+




.. _AHRS_TRIM_Z:

AHRS\_TRIM\_Z: AHRS Trim Yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Not Used


+-------------------+-----------+---------+
| Range             | Increment | Units   |
+===================+===========+=========+
| -0.1745 - +0.1745 | 0.01      | Radians |
+-------------------+-----------+---------+




.. _AHRS_ORIENTATION:

AHRS\_ORIENTATION: Board Orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Overall board orientation relative to the standard orientation for the board type\. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle\. This option takes affect on next boot\. After changing you will need to re\-level your vehicle\.


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Yaw45               | |
| +-------+---------------------+ |
| | 2     | Yaw90               | |
| +-------+---------------------+ |
| | 3     | Yaw135              | |
| +-------+---------------------+ |
| | 4     | Yaw180              | |
| +-------+---------------------+ |
| | 5     | Yaw225              | |
| +-------+---------------------+ |
| | 6     | Yaw270              | |
| +-------+---------------------+ |
| | 7     | Yaw315              | |
| +-------+---------------------+ |
| | 8     | Roll180             | |
| +-------+---------------------+ |
| | 9     | Roll180Yaw45        | |
| +-------+---------------------+ |
| | 10    | Roll180Yaw90        | |
| +-------+---------------------+ |
| | 11    | Roll180Yaw135       | |
| +-------+---------------------+ |
| | 12    | Pitch180            | |
| +-------+---------------------+ |
| | 13    | Roll180Yaw225       | |
| +-------+---------------------+ |
| | 14    | Roll180Yaw270       | |
| +-------+---------------------+ |
| | 15    | Roll180Yaw315       | |
| +-------+---------------------+ |
| | 16    | Roll90              | |
| +-------+---------------------+ |
| | 17    | Roll90Yaw45         | |
| +-------+---------------------+ |
| | 18    | Roll90Yaw90         | |
| +-------+---------------------+ |
| | 19    | Roll90Yaw135        | |
| +-------+---------------------+ |
| | 20    | Roll270             | |
| +-------+---------------------+ |
| | 21    | Roll270Yaw45        | |
| +-------+---------------------+ |
| | 22    | Roll270Yaw90        | |
| +-------+---------------------+ |
| | 23    | Roll270Yaw136       | |
| +-------+---------------------+ |
| | 24    | Pitch90             | |
| +-------+---------------------+ |
| | 25    | Pitch270            | |
| +-------+---------------------+ |
| | 26    | Pitch180Yaw90       | |
| +-------+---------------------+ |
| | 27    | Pitch180Yaw270      | |
| +-------+---------------------+ |
| | 28    | Roll90Pitch90       | |
| +-------+---------------------+ |
| | 29    | Roll180Pitch90      | |
| +-------+---------------------+ |
| | 30    | Roll270Pitch90      | |
| +-------+---------------------+ |
| | 31    | Roll90Pitch180      | |
| +-------+---------------------+ |
| | 32    | Roll270Pitch180     | |
| +-------+---------------------+ |
| | 33    | Roll90Pitch270      | |
| +-------+---------------------+ |
| | 34    | Roll180Pitch270     | |
| +-------+---------------------+ |
| | 35    | Roll270Pitch270     | |
| +-------+---------------------+ |
| | 36    | Roll90Pitch180Yaw90 | |
| +-------+---------------------+ |
| | 37    | Roll90Yaw270        | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _AHRS_COMP_BETA:

AHRS\_COMP\_BETA: AHRS Velocity Complementary Filter Beta Coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the time constant for the cross\-over frequency used to fuse AHRS \(airspeed and heading\) and GPS data to estimate ground velocity\. Time constant is 0\.1\/beta\. A larger time constant will use GPS data less and a small time constant will use air data less\.


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.001 - 0.5 | .01       |
+-------------+-----------+




.. _AHRS_GPS_MINSATS:

AHRS\_GPS\_MINSATS: AHRS GPS Minimum satellites
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum number of satellites visible to use GPS for velocity based corrections attitude correction\. This defaults to 6\, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers\.


+--------+-----------+
| Range  | Increment |
+========+===========+
| 0 - 10 | 1         |
+--------+-----------+




.. _AHRS_EKF_TYPE:

AHRS\_EKF\_TYPE: Use NavEKF Kalman filter for attitude and position estimation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls whether the NavEKF Kalman filter is used for attitude and position estimation and whether fallback to the DCM algorithm is allowed\. Note that on copters \"disabled\" is not available\, and will be the same as \"enabled \- no fallback\"


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | Disabled    | |
| +-------+-------------+ |
| | 1     | Enabled     | |
| +-------+-------------+ |
| | 2     | Enable EKF2 | |
| +-------+-------------+ |
|                         |
+-------------------------+





.. _parameters_ARSPD_:

ARSPD\_ Parameters
------------------


.. _ARSPD_ENABLE:

ARSPD\_ENABLE: Airspeed enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable airspeed sensor


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+




.. _ARSPD_USE:

ARSPD\_USE: Airspeed use
~~~~~~~~~~~~~~~~~~~~~~~~


use airspeed for flight control


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 1     | Use       | |
| +-------+-----------+ |
| | 0     | Don't Use | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _ARSPD_OFFSET:

ARSPD\_OFFSET: Airspeed offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Airspeed calibration offset


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD_RATIO:

ARSPD\_RATIO: Airspeed ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Airspeed calibration ratio


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD_PIN:

ARSPD\_PIN: Airspeed pin
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The analog pin number that the airspeed sensor is connected to\. Set this to 0\.\.9 for the APM2 analog pins\. Set to 64 on an APM1 for the dedicated airspeed port on the end of the board\. Set to 11 on PX4 for the analog airspeed port\. Set to 15 on the Pixhawk for the analog airspeed port\. Set to 65 on the PX4 or Pixhawk for an EagleTree or MEAS I2C airspeed sensor\.


.. _ARSPD_AUTOCAL:

ARSPD\_AUTOCAL: Automatic airspeed ratio calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If this is enabled then the APM will automatically adjust the ARSPD\_RATIO during flight\, based upon an estimation filter using ground speed and true airspeed\. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5\%\. This option should be enabled for a calibration flight then disabled again when calibration is complete\. Leaving it enabled all the time is not recommended\.


.. _ARSPD_TUBE_ORDER:

ARSPD\_TUBE\_ORDER: Control pitot tube order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters\. If you set this to 0 then the top connector on the sensor needs to be the dynamic pressure\. If set to 1 then the bottom connector needs to be the dynamic pressure\. If set to 2 \(the default\) then the airspeed driver will accept either order\. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft it receiving excessive pressure on the static port\, which would otherwise be seen as a positive airspeed\.


.. _ARSPD_SKIP_CAL:

ARSPD\_SKIP\_CAL: Skip airspeed calibration on startup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup\, instead using the offset from the last calibration\. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+





.. _parameters_NAVL1_:

NAVL1\_ Parameters
------------------


.. _NAVL1_PERIOD:

NAVL1\_PERIOD: L1 control period
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Period in seconds of L1 tracking loop\. This parameter is the primary control for agressiveness of turns in auto mode\. This needs to be larger for less responsive airframes\. The default of 20 is quite conservative\, but for most RC aircraft will lead to reasonable flight\. For smaller more agile aircraft a value closer to 15 is appropriate\, or even as low as 10 for some very agile aircraft\. When tuning\, change this value in small increments\, as a value that is much too small \(say 5 or 10 below the right value\) can lead to very radical turns\, and a risk of stalling\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 1 - 60 | 1         | seconds |
+--------+-----------+---------+




.. _NAVL1_DAMPING:

NAVL1\_DAMPING: L1 control damping ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Damping ratio for L1 control\. Increase this in increments of 0\.05 if you are getting overshoot in path tracking\. You should not need a value below 0\.7 or above 0\.85\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.6 - 1.0 | 0.05      |
+-----------+-----------+




.. _NAVL1_XTRACK_I:

NAVL1\_XTRACK\_I: L1 control crosstrack integrator gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Crosstrack error integrator gain\. This gain is applied to the crosstrack error to ensure it converges to zero\. Set to zero to disable\. Smaller values converge slower\, higher values will cause crosstrack error oscillation\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 0.1 | 0.01      |
+---------+-----------+





.. _parameters_TECS_:

TECS\_ Parameters
-----------------


.. _TECS_CLMB_MAX:

TECS\_CLMB\_MAX: Maximum Climb Rate \(metres\/sec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the best climb rate that the aircraft can achieve with the throttle set to THR\_MAX and the airspeed set to the default value\. For electric aircraft make sure this number can be achieved towards the end of flight when the battery voltage has reduced\. The setting of this parameter can be checked by commanding a positive altitude change of 100m in loiter\, RTL or guided mode\. If the throttle required to climb is close to THR\_MAX and the aircraft is maintaining airspeed\, then this parameter is set correctly\. If the airspeed starts to reduce\, then the parameter is set to high\, and if the throttle demand require to climb and maintain speed is noticeably less than THR\_MAX\, then either CLMB\_MAX should be increased or THR\_MAX reduced\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.1 - 20.0 | 0.1       |
+------------+-----------+




.. _TECS_SINK_MIN:

TECS\_SINK\_MIN: Minimum Sink Rate \(metres\/sec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the sink rate of the aircraft with the throttle set to THR\_MIN and the same airspeed as used to measure CLMB\_MAX\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.1 - 10.0 | 0.1       |
+------------+-----------+




.. _TECS_TIME_CONST:

TECS\_TIME\_CONST: Controller time constant \(sec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the time constant of the TECS control algorithm\. Smaller values make it faster to respond\, large values make it slower to respond\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 3.0 - 10.0 | 0.2       |
+------------+-----------+




.. _TECS_THR_DAMP:

TECS\_THR\_DAMP: Controller throttle damping
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the damping gain for the throttle demand loop\. Increase to add damping  to correct for oscillations in speed and height\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 1.0 | 0.1       |
+-----------+-----------+




.. _TECS_INTEG_GAIN:

TECS\_INTEG\_GAIN: Controller integrator
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the integrator gain on the control loop\. Increase to increase the rate at which speed and height offsets are trimmed out


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 0.5 | 0.02      |
+-----------+-----------+




.. _TECS_VERT_ACC:

TECS\_VERT\_ACC: Vertical Acceleration Limit \(metres\/sec\^2\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the maximum vertical acceleration either up or down that the  controller will use to correct speed or height errors\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 1.0 - 10.0 | 0.5       |
+------------+-----------+




.. _TECS_HGT_OMEGA:

TECS\_HGT\_OMEGA: Height complementary filter frequency \(radians\/sec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the cross\-over frequency of the complementary filter used to fuse vertical acceleration and baro alt to obtain an estimate of height rate and height\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 1.0 - 5.0 | 0.05      |
+-----------+-----------+




.. _TECS_SPD_OMEGA:

TECS\_SPD\_OMEGA: Speed complementary filter frequency \(radians\/sec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the cross\-over frequency of the complementary filter used to fuse longitudinal acceleration and airspeed to obtain a lower noise and lag estimate of airspeed\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.5 - 2.0 | 0.05      |
+-----------+-----------+




.. _TECS_RLL2THR:

TECS\_RLL2THR: Bank angle compensation gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Increasing this gain turn increases the amount of throttle that will be used to compensate for the additional drag created by turning\. Ideally this should be set to approximately 10 x the extra sink rate in m\/s created by a 45 degree bank turn\. Increase this gain if the aircraft initially loses energy in turns and reduce if the aircraft initially gains energy in turns\. Efficient high aspect\-ratio aircraft \(eg powered sailplanes\) can use a lower value\, whereas inefficient low aspect\-ratio models \(eg delta wings\) can use a higher value\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 5.0 - 30.0 | 1.0       |
+------------+-----------+




.. _TECS_SPDWEIGHT:

TECS\_SPDWEIGHT: Weighting applied to speed control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter adjusts the amount of weighting that the pitch control applies to speed vs height errors\. Setting it to 0\.0 will cause the pitch control to control height and ignore speed errors\. This will normally improve height accuracy but give larger airspeed errors\. Setting it to 2\.0 will cause the pitch control loop to control speed and ignore height errors\. This will normally reduce airsped errors\, but give larger height errors\.	A value of 1\.0 gives a balanced response and is the default\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 2.0 | 0.1       |
+-----------+-----------+




.. _TECS_PTCH_DAMP:

TECS\_PTCH\_DAMP: Controller pitch damping
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the damping gain for the pitch demand loop\. Increase to add damping  to correct for oscillations in speed and height\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 1.0 | 0.1       |
+-----------+-----------+




.. _TECS_SINK_MAX:

TECS\_SINK\_MAX: Maximum Descent Rate \(metres\/sec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum descent rate that the controller will use\.  If this value is too large\, the aircraft will reach the pitch angle limit first and be unable to achieve the descent rate\. This should be set to a value that can be achieved at the lower pitch angle limit\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.0 - 20.0 | 0.1       |
+------------+-----------+




.. _TECS_LAND_ARSPD:

TECS\_LAND\_ARSPD: Airspeed during landing approach \(m\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When performing an autonomus landing\, this value is used as the goal airspeed during approach\.  Note that this parameter is not useful if your platform does not have an airspeed sensor \(use TECS\_LAND\_THR instead\)\.  If negative then this value is not used during landing\.


+----------+-----------+
| Range    | Increment |
+==========+===========+
| -1 - 127 | 1         |
+----------+-----------+




.. _TECS_LAND_THR:

TECS\_LAND\_THR: Cruise throttle during landing approach \(percentage\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Use this parameter instead of LAND\_ARSPD if your platform does not have an airspeed sensor\.  It is the cruise throttle during landing approach\.  If this value is negative then it is disabled and TECS\_LAND\_ARSPD is used instead\.


+----------+-----------+
| Range    | Increment |
+==========+===========+
| -1 - 100 | 0.1       |
+----------+-----------+




.. _TECS_LAND_SPDWGT:

TECS\_LAND\_SPDWGT: Weighting applied to speed control during landing\.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Same as SPDWEIGHT parameter\, with the exception that this parameter is applied during landing flight stages\.  A value closer to 2 will result in the plane ignoring height error during landing and our experience has been that the plane will therefore keep the nose up \-\- sometimes good for a glider landing \(with the side effect that you will likely glide a ways past the landing point\)\.  A value closer to 0 results in the plane ignoring speed error \-\- use caution when lowering the value below 1 \-\- ignoring speed could result in a stall\. Values between 0 and 2 are valid values for a fixed landing weight\. When using \-1 the weight will be scaled during the landing\. At the start of the landing approach it starts with TECS\_SPDWEIGHT and scales down to 0 by the time you reach the land point\. Example\: Halfway down the landing approach you\'ll effectively have a weight of TECS\_SPDWEIGHT\/2\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| -1.0 - 2.0 | 0.1       |
+------------+-----------+




.. _TECS_PITCH_MAX:

TECS\_PITCH\_MAX: Maximum pitch in auto flight
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls maximum pitch up in automatic throttle modes\. If this is set to zero then LIM\_PITCH\_MAX is used instead\. The purpose of this parameter is to allow the use of a smaller pitch range when in automatic flight than what is used in FBWA mode\.


+--------+-----------+
| Range  | Increment |
+========+===========+
| 0 - 45 | 1         |
+--------+-----------+




.. _TECS_PITCH_MIN:

TECS\_PITCH\_MIN: Minimum pitch in auto flight
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls minimum pitch in automatic throttle modes\. If this is set to zero then LIM\_PITCH\_MIN is used instead\. The purpose of this parameter is to allow the use of a smaller pitch range when in automatic flight than what is used in FBWA mode\. Note that TECS\_PITCH\_MIN should be a negative number\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| -45 - 0 | 1         |
+---------+-----------+




.. _TECS_LAND_SINK:

TECS\_LAND\_SINK: Sink rate for final landing stage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The sink rate in meters\/second for the final stage of landing\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 2.0 | 0.1       |
+-----------+-----------+




.. _TECS_LAND_TCONST:

TECS\_LAND\_TCONST: Land controller time constant \(sec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the time constant of the TECS control algorithm when in final landing stage of flight\. It should be smaller than TECS\_TIME\_CONST to allow for faster flare


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 1.0 - 5.0 | 0.2       |
+-----------+-----------+




.. _TECS_LAND_DAMP:

TECS\_LAND\_DAMP: Controller sink rate to pitch gain during flare
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the sink rate gain for the pitch demand loop when in final landing stage of flight\. It should be larger than TECS\_PTCH\_DAMP to allow for better sink rate control during flare\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 1.0 | 0.1       |
+-----------+-----------+




.. _TECS_LAND_PMAX:

TECS\_LAND\_PMAX: Maximum pitch during final stage of landing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This limits the pitch used during the final stage of automatic landing\. During the final landing stage most planes need to keep their pitch small to avoid stalling\. A maximum of 10 degrees is usually good\. A value of zero means to use the normal pitch limits\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| -5 - 40 | 1         |
+---------+-----------+




.. _TECS_APPR_SMAX:

TECS\_APPR\_SMAX: Sink rate max for landing approach stage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The sink rate max for the landing approach stage of landing\. This will need to be large for steep landing approaches especially when using reverse thrust\. If 0\, then use TECS\_SINK\_MAX\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.0 - 20.0 | 0.1       | m/s   |
+------------+-----------+-------+




.. _TECS_LAND_SRC:

TECS\_LAND\_SRC: Land sink rate change
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

When zero\, the flare sink rate \(TECS\_LAND\_SINK\) is a fixed sink demand\. With this enabled the flare sinkrate will increase\/decrease the flare sink demand as you get further beyond the LAND waypoint\. Has no effect before the waypoint\. This value is added to TECS\_LAND\_SINK proportional to distance traveled after wp\. With an increasing sink rate you can still land in a given distance if you\'re traveling too fast and cruise passed the land point\. A positive value will force the plane to land sooner proportional to distance passed land point\. A negative number will tell the plane to slowly climb allowing for a pitched\-up stall landing\. Recommend 0\.2 as initial value\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| -2.0 - 2.0 | 0.1       | m/s/m |
+------------+-----------+-------+




.. _TECS_LAND_TDAMP:

TECS\_LAND\_TDAMP: Controller throttle damping when landing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the damping gain for the throttle demand loop during and auto\-landing\. Same as TECS\_THR\_DAMP but only in effect during an auto\-land\. Increase to add damping to correct for oscillations in speed and height\. When set to 0 landing throttle damp is controlled by TECS\_THR\_DAMP\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 1.0 | 0.1       |
+-----------+-----------+




.. _TECS_LAND_IGAIN:

TECS\_LAND\_IGAIN: Controller integrator during landing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the integrator gain on the control loop during landing\. When set to 0 then TECS\_INTEG\_GAIN is used\. Increase to increase the rate at which speed and height offsets are trimmed out\. Typically values lower than TECS\_INTEG\_GAIN work best


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 0.5 | 0.02      |
+-----------+-----------+




.. _TECS_TKOFF_IGAIN:

TECS\_TKOFF\_IGAIN: Controller integrator during takeoff
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the integrator gain on the control loop during takeoff\. When set to 0 then TECS\_INTEG\_GAIN is used\. Increase to increase the rate at which speed and height offsets are trimmed out\. Typically values higher than TECS\_INTEG\_GAIN work best


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 0.5 | 0.02      |
+-----------+-----------+





.. _parameters_MNT:

MNT Parameters
--------------


.. _MNT_DEFLT_MODE:

MNT\_DEFLT\_MODE: Mount default operating mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount default operating mode on startup and after control is returned from autopilot


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | Retracted         | |
| +-------+-------------------+ |
| | 1     | Neutral           | |
| +-------+-------------------+ |
| | 2     | MavLink Targeting | |
| +-------+-------------------+ |
| | 3     | RC Targeting      | |
| +-------+-------------------+ |
| | 4     | GPS Point         | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _MNT_RETRACT_X:

MNT\_RETRACT\_X: Mount roll angle when in retracted position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount roll angle when in retracted position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT_RETRACT_Y:

MNT\_RETRACT\_Y: Mount tilt\/pitch angle when in retracted position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount tilt\/pitch angle when in retracted position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT_RETRACT_Z:

MNT\_RETRACT\_Z: Mount yaw\/pan angle when in retracted position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount yaw\/pan angle when in retracted position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT_NEUTRAL_X:

MNT\_NEUTRAL\_X: Mount roll angle when in neutral position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount roll angle when in neutral position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT_NEUTRAL_Y:

MNT\_NEUTRAL\_Y: Mount tilt\/pitch angle when in neutral position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount tilt\/pitch angle when in neutral position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT_NEUTRAL_Z:

MNT\_NEUTRAL\_Z: Mount pan\/yaw angle when in neutral position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount pan\/yaw angle when in neutral position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT_STAB_ROLL:

MNT\_STAB\_ROLL: Stabilize mount\'s roll angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable roll stabilisation relative to Earth


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT_STAB_TILT:

MNT\_STAB\_TILT: Stabilize mount\'s pitch\/tilt angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable tilt\/pitch stabilisation relative to Earth


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT_STAB_PAN:

MNT\_STAB\_PAN: Stabilize mount pan\/yaw angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable pan\/yaw stabilisation relative to Earth


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT_RC_IN_ROLL:

MNT\_RC\_IN\_ROLL: roll RC input channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0 for none\, any other for the RC channel to be used to control roll movements


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 5     | RC5      | |
| +-------+----------+ |
| | 6     | RC6      | |
| +-------+----------+ |
| | 7     | RC7      | |
| +-------+----------+ |
| | 8     | RC8      | |
| +-------+----------+ |
| | 9     | RC9      | |
| +-------+----------+ |
| | 10    | RC10     | |
| +-------+----------+ |
| | 11    | RC11     | |
| +-------+----------+ |
| | 12    | RC12     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT_ANGMIN_ROL:

MNT\_ANGMIN\_ROL: Minimum roll angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum physical roll angular position of mount\.


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT_ANGMAX_ROL:

MNT\_ANGMAX\_ROL: Maximum roll angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum physical roll angular position of the mount


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT_RC_IN_TILT:

MNT\_RC\_IN\_TILT: tilt \(pitch\) RC input channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0 for none\, any other for the RC channel to be used to control tilt \(pitch\) movements


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 5     | RC5      | |
| +-------+----------+ |
| | 6     | RC6      | |
| +-------+----------+ |
| | 7     | RC7      | |
| +-------+----------+ |
| | 8     | RC8      | |
| +-------+----------+ |
| | 9     | RC9      | |
| +-------+----------+ |
| | 10    | RC10     | |
| +-------+----------+ |
| | 11    | RC11     | |
| +-------+----------+ |
| | 12    | RC12     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT_ANGMIN_TIL:

MNT\_ANGMIN\_TIL: Minimum tilt angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum physical tilt \(pitch\) angular position of mount\.


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT_ANGMAX_TIL:

MNT\_ANGMAX\_TIL: Maximum tilt angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum physical tilt \(pitch\) angular position of the mount


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT_RC_IN_PAN:

MNT\_RC\_IN\_PAN: pan \(yaw\) RC input channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0 for none\, any other for the RC channel to be used to control pan \(yaw\) movements


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 5     | RC5      | |
| +-------+----------+ |
| | 6     | RC6      | |
| +-------+----------+ |
| | 7     | RC7      | |
| +-------+----------+ |
| | 8     | RC8      | |
| +-------+----------+ |
| | 9     | RC9      | |
| +-------+----------+ |
| | 10    | RC10     | |
| +-------+----------+ |
| | 11    | RC11     | |
| +-------+----------+ |
| | 12    | RC12     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT_ANGMIN_PAN:

MNT\_ANGMIN\_PAN: Minimum pan angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum physical pan \(yaw\) angular position of mount\.


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT_ANGMAX_PAN:

MNT\_ANGMAX\_PAN: Maximum pan angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum physical pan \(yaw\) angular position of the mount


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT_JSTICK_SPD:

MNT\_JSTICK\_SPD: mount joystick speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0 for position control\, small for low speeds\, 100 for max speed\. A good general value is 10 which gives a movement speed of 3 degrees per second\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 0 - 100 | 1         |
+---------+-----------+




.. _MNT_LEAD_RLL:

MNT\_LEAD\_RLL: Roll stabilization lead time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate\, compensating for servo delay\. Increase until the servo is responsive but doesn\'t overshoot\. Does nothing with pan stabilization enabled\.


+-----------+-----------+---------+
| Range     | Increment | Units   |
+===========+===========+=========+
| 0.0 - 0.2 | .005      | Seconds |
+-----------+-----------+---------+




.. _MNT_LEAD_PTCH:

MNT\_LEAD\_PTCH: Pitch stabilization lead time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate\. Increase until the servo is responsive but doesn\'t overshoot\. Does nothing with pan stabilization enabled\.


+-----------+-----------+---------+
| Range     | Increment | Units   |
+===========+===========+=========+
| 0.0 - 0.2 | .005      | Seconds |
+-----------+-----------+---------+




.. _MNT_TYPE:

MNT\_TYPE: Mount Type
~~~~~~~~~~~~~~~~~~~~~


Mount Type \(None\, Servo or MAVLink\)


+-----------------------------+----------------+
| Values                      | RebootRequired |
+=============================+================+
| +-------+-----------------+ | True           |
| | Value | Meaning         | |                |
| +=======+=================+ |                |
| | 0     | None            | |                |
| +-------+-----------------+ |                |
| | 1     | Servo           | |                |
| +-------+-----------------+ |                |
| | 2     | 3DR Solo        | |                |
| +-------+-----------------+ |                |
| | 3     | Alexmos Serial  | |                |
| +-------+-----------------+ |                |
| | 4     | SToRM32 MAVLink | |                |
| +-------+-----------------+ |                |
| | 5     | SToRM32 Serial  | |                |
| +-------+-----------------+ |                |
|                             |                |
+-----------------------------+----------------+




.. _MNT2_DEFLT_MODE:

MNT2\_DEFLT\_MODE: Mount default operating mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount default operating mode on startup and after control is returned from autopilot


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | Retracted         | |
| +-------+-------------------+ |
| | 1     | Neutral           | |
| +-------+-------------------+ |
| | 2     | MavLink Targeting | |
| +-------+-------------------+ |
| | 3     | RC Targeting      | |
| +-------+-------------------+ |
| | 4     | GPS Point         | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _MNT2_RETRACT_X:

MNT2\_RETRACT\_X: Mount2 roll angle when in retracted position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2 roll angle when in retracted position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT2_RETRACT_Y:

MNT2\_RETRACT\_Y: Mount2 tilt\/pitch angle when in retracted position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2 tilt\/pitch angle when in retracted position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT2_RETRACT_Z:

MNT2\_RETRACT\_Z: Mount2 yaw\/pan angle when in retracted position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2 yaw\/pan angle when in retracted position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT2_NEUTRAL_X:

MNT2\_NEUTRAL\_X: Mount2 roll angle when in neutral position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2 roll angle when in neutral position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT2_NEUTRAL_Y:

MNT2\_NEUTRAL\_Y: Mount2 tilt\/pitch angle when in neutral position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2 tilt\/pitch angle when in neutral position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT2_NEUTRAL_Z:

MNT2\_NEUTRAL\_Z: Mount2 pan\/yaw angle when in neutral position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2 pan\/yaw angle when in neutral position


+------------------+-----------+---------+
| Range            | Increment | Units   |
+==================+===========+=========+
| -180.00 - 179.99 | 1         | Degrees |
+------------------+-----------+---------+




.. _MNT2_STAB_ROLL:

MNT2\_STAB\_ROLL: Stabilize Mount2\'s roll angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable roll stabilisation relative to Earth


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT2_STAB_TILT:

MNT2\_STAB\_TILT: Stabilize Mount2\'s pitch\/tilt angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable tilt\/pitch stabilisation relative to Earth


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT2_STAB_PAN:

MNT2\_STAB\_PAN: Stabilize mount2 pan\/yaw angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable pan\/yaw stabilisation relative to Earth


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT2_RC_IN_ROLL:

MNT2\_RC\_IN\_ROLL: Mount2\'s roll RC input channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0 for none\, any other for the RC channel to be used to control roll movements


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 5     | RC5      | |
| +-------+----------+ |
| | 6     | RC6      | |
| +-------+----------+ |
| | 7     | RC7      | |
| +-------+----------+ |
| | 8     | RC8      | |
| +-------+----------+ |
| | 9     | RC9      | |
| +-------+----------+ |
| | 10    | RC10     | |
| +-------+----------+ |
| | 11    | RC11     | |
| +-------+----------+ |
| | 12    | RC12     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT2_ANGMIN_ROL:

MNT2\_ANGMIN\_ROL: Mount2\'s minimum roll angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2\'s minimum physical roll angular position


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT2_ANGMAX_ROL:

MNT2\_ANGMAX\_ROL: Mount2\'s maximum roll angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2\'s maximum physical roll angular position


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT2_RC_IN_TILT:

MNT2\_RC\_IN\_TILT: Mount2\'s tilt \(pitch\) RC input channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0 for none\, any other for the RC channel to be used to control tilt \(pitch\) movements


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 5     | RC5      | |
| +-------+----------+ |
| | 6     | RC6      | |
| +-------+----------+ |
| | 7     | RC7      | |
| +-------+----------+ |
| | 8     | RC8      | |
| +-------+----------+ |
| | 9     | RC9      | |
| +-------+----------+ |
| | 10    | RC10     | |
| +-------+----------+ |
| | 11    | RC11     | |
| +-------+----------+ |
| | 12    | RC12     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT2_ANGMIN_TIL:

MNT2\_ANGMIN\_TIL: Mount2\'s minimum tilt angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2\'s minimum physical tilt \(pitch\) angular position


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT2_ANGMAX_TIL:

MNT2\_ANGMAX\_TIL: Mount2\'s maximum tilt angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2\'s maximum physical tilt \(pitch\) angular position


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT2_RC_IN_PAN:

MNT2\_RC\_IN\_PAN: Mount2\'s pan \(yaw\) RC input channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0 for none\, any other for the RC channel to be used to control pan \(yaw\) movements


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 5     | RC5      | |
| +-------+----------+ |
| | 6     | RC6      | |
| +-------+----------+ |
| | 7     | RC7      | |
| +-------+----------+ |
| | 8     | RC8      | |
| +-------+----------+ |
| | 9     | RC9      | |
| +-------+----------+ |
| | 10    | RC10     | |
| +-------+----------+ |
| | 11    | RC11     | |
| +-------+----------+ |
| | 12    | RC12     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _MNT2_ANGMIN_PAN:

MNT2\_ANGMIN\_PAN: Mount2\'s minimum pan angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mount2\'s minimum physical pan \(yaw\) angular position


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT2_ANGMAX_PAN:

MNT2\_ANGMAX\_PAN: Mount2\'s maximum pan angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


MOunt2\'s maximum physical pan \(yaw\) angular position


+----------------+-----------+---------------+
| Range          | Increment | Units         |
+================+===========+===============+
| -18000 - 17999 | 1         | Centi-Degrees |
+----------------+-----------+---------------+




.. _MNT2_LEAD_RLL:

MNT2\_LEAD\_RLL: Mount2\'s Roll stabilization lead time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate\, compensating for servo delay\. Increase until the servo is responsive but doesn\'t overshoot\. Does nothing with pan stabilization enabled\.


+-----------+-----------+---------+
| Range     | Increment | Units   |
+===========+===========+=========+
| 0.0 - 0.2 | .005      | Seconds |
+-----------+-----------+---------+




.. _MNT2_LEAD_PTCH:

MNT2\_LEAD\_PTCH: Mount2\'s Pitch stabilization lead time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate\. Increase until the servo is responsive but doesn\'t overshoot\. Does nothing with pan stabilization enabled\.


+-----------+-----------+---------+
| Range     | Increment | Units   |
+===========+===========+=========+
| 0.0 - 0.2 | .005      | Seconds |
+-----------+-----------+---------+




.. _MNT2_TYPE:

MNT2\_TYPE: Mount2 Type
~~~~~~~~~~~~~~~~~~~~~~~


Mount Type \(None\, Servo or MAVLink\)


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | 0     | None            | |
| +-------+-----------------+ |
| | 1     | Servo           | |
| +-------+-----------------+ |
| | 2     | 3DR Solo        | |
| +-------+-----------------+ |
| | 3     | Alexmos Serial  | |
| +-------+-----------------+ |
| | 4     | SToRM32 MAVLink | |
| +-------+-----------------+ |
| | 5     | SToRM32 Serial  | |
| +-------+-----------------+ |
|                             |
+-----------------------------+





.. _parameters_LOG:

LOG Parameters
--------------


.. _LOG_BACKEND_TYPE:

LOG\_BACKEND\_TYPE: DataFlash Backend Storage type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


0 for None\, 1 for File\, 2 for dataflash mavlink\, 3 for both file and dataflash


+--------------------------------+
| Values                         |
+================================+
| +-------+--------------------+ |
| | Value | Meaning            | |
| +=======+====================+ |
| | 0     | None               | |
| +-------+--------------------+ |
| | 1     | File               | |
| +-------+--------------------+ |
| | 2     | MAVLink            | |
| +-------+--------------------+ |
| | 3     | BothFileAndMAVLink | |
| +-------+--------------------+ |
|                                |
+--------------------------------+




.. _LOG_FILE_BUFSIZE:

LOG\_FILE\_BUFSIZE: Maximum DataFlash File Backend buffer size \(in kilobytes\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The DataFlash\_File backend uses a buffer to store data before writing to the block device\.  Raising this value may reduce \"gaps\" in your SD card logging\.  This buffer size may be reduced depending on available memory\.  PixHawk requires at least 4 kilobytes\.  Maximum value available here is 64 kilobytes\.


.. _LOG_DISARMED:

LOG\_DISARMED: Enable logging while disarmed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If LOG\_DISARMED is set to 1 then logging will be enabled while disarmed\. This can make for very large logfiles but can help a lot when tracking down startup issues


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _LOG_REPLAY:

LOG\_REPLAY: Enable logging of information needed for Replay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If LOG\_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter\. It is suggested that you also raise LOG\_FILE\_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+





.. _parameters_BATT:

BATT Parameters
---------------


.. _BATT_MONITOR:

BATT\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | SMBus                      | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT_VOLT_PIN:

BATT\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to 0 \~ 13 will enable battery voltage sensing on pins A0 \~ A13\. For the 3DR power brick on APM2\.5 it should be set to 13\. On the PX4 it should be set to 100\. On the Pixhawk powered from the PM connector it should be set to 2\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 0     | A0       | |
| +-------+----------+ |
| | 1     | A1       | |
| +-------+----------+ |
| | 2     | Pixhawk  | |
| +-------+----------+ |
| | 13    | A13      | |
| +-------+----------+ |
| | 100   | PX4      | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BATT_CURR_PIN:

BATT\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to 0 \~ 13 will enable battery current sensing on pins A0 \~ A13\. For the 3DR power brick on APM2\.5 it should be set to 12\. On the PX4 it should be set to 101\. On the Pixhawk powered from the PM connector it should be set to 3\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 1     | A1       | |
| +-------+----------+ |
| | 2     | A2       | |
| +-------+----------+ |
| | 3     | Pixhawk  | |
| +-------+----------+ |
| | 12    | A12      | |
| +-------+----------+ |
| | 101   | PX4      | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BATT_VOLT_MULT:

BATT\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick on APM2 or Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX4 using the PX4IO power supply this should be set to 1\.


.. _BATT_AMP_PERVOLT:

BATT\_AMP\_PERVOLT: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------+
| Units     |
+===========+
| Amps/Volt |
+-----------+




.. _BATT_AMP_OFFSET:

BATT\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| Volts |
+-------+




.. _BATT_CAPACITY:

BATT\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+-------+
| Increment | Units |
+===========+=======+
| 50        | mAh   |
+-----------+-------+




.. _BATT_WATT_MAX:

BATT\_WATT\_MAX: Maximum allowed power \(Watts\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If battery wattage \(voltage \* current\) exceeds this value then the system will reduce max throttle \(THR\_MAX\, TKOFF\_THR\_MAX and THR\_MIN for reverse thrust\) to satisfy this limit\. This helps limit high current to low C rated batteries regardless of battery voltage\. The max throttle will slowly grow back to THR\_MAX \(or TKOFF\_THR\_MAX \) and THR\_MIN if demanding the current max and under the watt max\. Use 0 to disable\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 1         | Watts |
+-----------+-------+




.. _BATT2_MONITOR:

BATT2\_MONITOR: Battery monitoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls enabling monitoring of the battery\'s voltage and current


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | Disabled                   | |
| +-------+----------------------------+ |
| | 3     | Analog Voltage Only        | |
| +-------+----------------------------+ |
| | 4     | Analog Voltage and Current | |
| +-------+----------------------------+ |
| | 5     | SMBus                      | |
| +-------+----------------------------+ |
| | 6     | Bebop                      | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BATT2_VOLT_PIN:

BATT2\_VOLT\_PIN: Battery Voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to 0 \~ 13 will enable battery voltage sensing on pins A0 \~ A13\. For the 3DR power brick on APM2\.5 it should be set to 13\. On the PX4 it should be set to 100\. On the Pixhawk powered from the PM connector it should be set to 2\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 0     | A0       | |
| +-------+----------+ |
| | 1     | A1       | |
| +-------+----------+ |
| | 2     | Pixhawk  | |
| +-------+----------+ |
| | 13    | A13      | |
| +-------+----------+ |
| | 100   | PX4      | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BATT2_CURR_PIN:

BATT2\_CURR\_PIN: Battery Current sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to 0 \~ 13 will enable battery current sensing on pins A0 \~ A13\. For the 3DR power brick on APM2\.5 it should be set to 12\. On the PX4 it should be set to 101\. On the Pixhawk powered from the PM connector it should be set to 3\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | -1    | Disabled | |
| +-------+----------+ |
| | 1     | A1       | |
| +-------+----------+ |
| | 2     | A2       | |
| +-------+----------+ |
| | 3     | Pixhawk  | |
| +-------+----------+ |
| | 12    | A12      | |
| +-------+----------+ |
| | 101   | PX4      | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BATT2_VOLT_MULT:

BATT2\_VOLT\_MULT: Voltage Multiplier
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to convert the voltage of the voltage sensing pin \(BATT\_VOLT\_PIN\) to the actual battery\'s voltage \(pin\_voltage \* VOLT\_MULT\)\. For the 3DR Power brick on APM2 or Pixhawk\, this should be set to 10\.1\. For the Pixhawk with the 3DR 4in1 ESC this should be 12\.02\. For the PX4 using the PX4IO power supply this should be set to 1\.


.. _BATT2_AMP_PERVOL:

BATT2\_AMP\_PERVOL: Amps per volt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of amps that a 1V reading on the current sensor corresponds to\. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17\. For the Pixhawk with the 3DR 4in1 ESC this should be 17\.


+-----------+
| Units     |
+===========+
| Amps/Volt |
+-----------+




.. _BATT2_AMP_OFFSET:

BATT2\_AMP\_OFFSET: AMP offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Voltage offset at zero current on current sensor


+-------+
| Units |
+=======+
| Volts |
+-------+




.. _BATT2_CAPACITY:

BATT2\_CAPACITY: Battery capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Capacity of the battery in mAh when full


+-----------+-------+
| Increment | Units |
+===========+=======+
| 50        | mAh   |
+-----------+-------+




.. _BATT2_WATT_MAX:

BATT2\_WATT\_MAX: Maximum allowed current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If battery wattage \(voltage \* current\) exceeds this value then the system will reduce max throttle \(THR\_MAX\, TKOFF\_THR\_MAX and THR\_MIN for reverse thrust\) to satisfy this limit\. This helps limit high current to low C rated batteries regardless of battery voltage\. The max throttle will slowly grow back to THR\_MAX \(or TKOFF\_THR\_MAX \) and THR\_MIN if demanding the current max and under the watt max\. Use 0 to disable\.


+-----------+-------+
| Increment | Units |
+===========+=======+
| 1         | Amps  |
+-----------+-------+





.. _parameters_BRD_:

BRD\_ Parameters
----------------


.. _BRD_PWM_COUNT:

BRD\_PWM\_COUNT: Auxiliary pin config
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Control assigning of FMU pins to PWM output\, timer capture and GPIO\. All unassigned pins can be used for GPIO


+----------------------------------------+
| Values                                 |
+========================================+
| +-------+----------------------------+ |
| | Value | Meaning                    | |
| +=======+============================+ |
| | 0     | No PWMs                    | |
| +-------+----------------------------+ |
| | 2     | Two PWMs                   | |
| +-------+----------------------------+ |
| | 4     | Four PWMs                  | |
| +-------+----------------------------+ |
| | 6     | Six PWMs                   | |
| +-------+----------------------------+ |
| | 7     | Three PWMs and One Capture | |
| +-------+----------------------------+ |
|                                        |
+----------------------------------------+




.. _BRD_SER1_RTSCTS:

BRD\_SER1\_RTSCTS: Serial 1 flow control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable flow control on serial 1 \(telemetry 1\) on Pixhawk\. You must have the RTS and CTS pins connected to your radio\. The standard DF13 6 pin connector for a 3DR radio does have those pins connected\. If this is set to 2 then flow control will be auto\-detected by checking for the output buffer filling on startup\. Note that the PX4v1 does not have hardware flow control pins on this port\, so you should leave this disabled\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
| | 2     | Auto     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BRD_SER2_RTSCTS:

BRD\_SER2\_RTSCTS: Serial 2 flow control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable flow control on serial 2 \(telemetry 2\) on Pixhawk and PX4\. You must have the RTS and CTS pins connected to your radio\. The standard DF13 6 pin connector for a 3DR radio does have those pins connected\. If this is set to 2 then flow control will be auto\-detected by checking for the output buffer filling on startup\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
| | 2     | Auto     | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BRD_SAFETYENABLE:

BRD\_SAFETYENABLE:  Enable use of safety arming switch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Disabling this option will disable the use of the safety switch on PX4 for arming\. Use of the safety switch is highly recommended\, so you should leave this option set to 1 except in unusual circumstances\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BRD_SBUS_OUT:

BRD\_SBUS\_OUT:  SBUS output rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the SBUS output frame rate in Hz


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | 50Hz     | |
| +-------+----------+ |
| | 2     | 75Hz     | |
| +-------+----------+ |
| | 3     | 100Hz    | |
| +-------+----------+ |
| | 4     | 150Hz    | |
| +-------+----------+ |
| | 5     | 200Hz    | |
| +-------+----------+ |
| | 6     | 250Hz    | |
| +-------+----------+ |
| | 7     | 300Hz    | |
| +-------+----------+ |
|                      |
+----------------------+




.. _BRD_SERIAL_NUM:

BRD\_SERIAL\_NUM: User\-defined serial number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


User\-defined serial number of this vehicle\, it can be any arbitrary number you want and has no effect on the autopilot


+----------------+
| Range          |
+================+
| -32767 - 32768 |
+----------------+




.. _BRD_CAN_ENABLE:

BRD\_CAN\_ENABLE:  Enable use of UAVCAN devices
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enabling this option on a Pixhawk enables UAVCAN devices\. Note that this uses about 25k of memory


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+





.. _parameters_AFS_:

AFS\_ Parameters
----------------


.. _AFS_MAN_PIN:

AFS\_MAN\_PIN: Manual Pin
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets a digital output pin to set high when in manual mode


.. _AFS_HB_PIN:

AFS\_HB\_PIN: Heartbeat Pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets a digital output pin which is cycled at 10Hz when termination is not activated\. Note that if a FS\_TERM\_PIN is set then the heartbeat pin will continue to cycle at 10Hz when termination is activated\, to allow the termination board to distinguish between autopilot crash and termination\.


.. _AFS_WP_COMMS:

AFS\_WP\_COMMS: Comms Waypoint
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Waypoint number to navigate to on comms loss


.. _AFS_GPS_LOSS:

AFS\_GPS\_LOSS: GPS Loss Waypoint
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Waypoint number to navigate to on GPS lock loss


.. _AFS_TERMINATE:

AFS\_TERMINATE: Force Terminate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Can be set in flight to force termination of the heartbeat signal


.. _AFS_TERM_ACTION:

AFS\_TERM\_ACTION: Terminate action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This can be used to force an action on flight termination\. Normally this is handled by an external failsafe board\, but you can setup APM to handle it here\. If set to 0 \(which is the default\) then no extra action is taken\. If set to the magic value 42 then the plane will deliberately crash itself by setting maximum throws on all surfaces\, and zero throttle


.. _AFS_TERM_PIN:

AFS\_TERM\_PIN: Terminate Pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets a digital output pin to set high on flight termination


.. _AFS_AMSL_LIMIT:

AFS\_AMSL\_LIMIT: AMSL limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the AMSL \(above mean sea level\) altitude limit\. If the pressure altitude determined by QNH exceeds this limit then flight termination will be forced\. Note that this limit is in meters\, whereas pressure altitude limits are often quoted in feet\. A value of zero disables the pressure altitude limit\.


+--------+
| Units  |
+========+
| meters |
+--------+




.. _AFS_AMSL_ERR_GPS:

AFS\_AMSL\_ERR\_GPS: Error margin for GPS based AMSL limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets margin for error in GPS derived altitude limit\. This error margin is only used if the barometer has failed\. If the barometer fails then the GPS will be used to enforce the AMSL\_LIMIT\, but this margin will be subtracted from the AMSL\_LIMIT first\, to ensure that even with the given amount of GPS altitude error the pressure altitude is not breached\. OBC users should set this to comply with their D2 safety case\. A value of \-1 will mean that barometer failure will lead to immediate termination\.


+--------+
| Units  |
+========+
| meters |
+--------+




.. _AFS_QNH_PRESSURE:

AFS\_QNH\_PRESSURE: QNH pressure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the QNH pressure in millibars to be used for pressure altitude in the altitude limit\. A value of zero disables the altitude limit\.


+----------+
| Units    |
+==========+
| millibar |
+----------+




.. _AFS_ENABLE:

AFS\_ENABLE: Enable Advanced Failsafe
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables the advanced failsafe system\. If this is set to zero \(disable\) then all the other AFS options have no effect


.. _AFS_MAX_GPS_LOSS:

AFS\_MAX\_GPS\_LOSS: Maximum number of GPS loss events
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Maximum number of GPS loss events before the aircraft stops returning to mission on GPS recovery\. Use zero to allow for any number of GPS loss events\.


.. _AFS_MAX_COM_LOSS:

AFS\_MAX\_COM\_LOSS: Maximum number of comms loss events
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Maximum number of comms loss events before the aircraft stops returning to mission on comms recovery\. Use zero to allow for any number of comms loss events\.


.. _AFS_GEOFENCE:

AFS\_GEOFENCE: Enable geofence Advanced Failsafe
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables the geofence part of the AFS\. Will only be in effect if AFS\_ENABLE is also 1


.. _AFS_RC:

AFS\_RC: Enable RC Advanced Failsafe
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables the RC part of the AFS\. Will only be in effect if AFS\_ENABLE is also 1


.. _AFS_RC_MAN_ONLY:

AFS\_RC\_MAN\_ONLY: Enable RC Termination only in manual control modes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If this parameter is set to 1\, then an RC loss will only cause the plane to terminate in manual control modes\. If it is 0\, then the plane will terminate in any flight mode\.


.. _AFS_DUAL_LOSS:

AFS\_DUAL\_LOSS: Enable dual loss terminate due to failure of both GCS and GPS simultaneously
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables the dual loss termination part of the AFS system\. If this parameter is 1 and both GPS and the ground control station fail simultaneously\, this will be considered a \"dual loss\" and cause termination\.


.. _AFS_RC_FAIL_TIME:

AFS\_RC\_FAIL\_TIME: RC failure time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the time in seconds in manual mode that failsafe termination will activate if RC input is lost\. For the OBC rules this should be \(1\.5\)\. Use 0 to disable\.


+---------+
| Units   |
+=========+
| seconds |
+---------+





.. _parameters_FLOW:

FLOW Parameters
---------------


.. _FLOW_ENABLE:

FLOW\_ENABLE: Optical flow enable\/disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Enabled\(1\) will enable optical flow\. Setting this to Disabled\(0\) will disable optical flow


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _FLOW_FXSCALER:

FLOW\_FXSCALER: X axis optical flow scale factor correction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate\. It can be used to correct for variations in effective focal length\. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0\.1\%\. Negative values reduce the scale factor\.


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| -200 - +200 | 1         |
+-------------+-----------+




.. _FLOW_FYSCALER:

FLOW\_FYSCALER: Y axis optical flow scale factor correction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate\. It can be used to correct for variations in effective focal length\. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0\.1\%\. Negative values reduce the scale factor\.


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| -200 - +200 | 1         |
+-------------+-----------+




.. _FLOW_ORIENT_YAW:

FLOW\_ORIENT\_YAW: Flow sensor yaw alignment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Specifies the number of centi\-degrees that the flow sensor is yawed relative to the vehicle\. A sensor with its X\-axis pointing to the right of the vehicle X axis has a positive yaw angle\.


+-----------------+-----------+
| Range           | Increment |
+=================+===========+
| -18000 - +18000 | 1         |
+-----------------+-----------+





.. _parameters_MIS_:

MIS\_ Parameters
----------------


.. _MIS_TOTAL:

MIS\_TOTAL: Total mission commands
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The number of mission mission items that has been loaded by the ground station\. Do not change this manually\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0 - 32766 | 1         |
+-----------+-----------+




.. _MIS_RESTART:

MIS\_RESTART: Mission Restart when entering Auto mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls mission starting point when entering Auto mode \(either restart from beginning of mission or resume from last command run\)


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | 0     | Resume Mission  | |
| +-------+-----------------+ |
| | 1     | Restart Mission | |
| +-------+-----------------+ |
|                             |
+-----------------------------+





.. _parameters_RALLY_:

RALLY\_ Parameters
------------------


.. _RALLY_TOTAL:

RALLY\_TOTAL: Rally Total
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Number of rally points currently loaded


.. _RALLY_LIMIT_KM:

RALLY\_LIMIT\_KM: Rally Limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Maximum distance to rally point\. If the closest rally point is more than this number of kilometers from the current position and the home location is closer than any of the rally points from the current position then do RTL to home rather than to the closest rally point\. This prevents a leftover rally point from a different airfield being used accidentally\. If this is set to 0 then the closest rally point is always used\.


+-----------+------------+
| Increment | Units      |
+===========+============+
| 0.1       | kilometers |
+-----------+------------+




.. _RALLY_INCL_HOME:

RALLY\_INCL\_HOME: Rally Include Home
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls if Home is included as a Rally point \(i\.e\. as a safe landing place\) for RTL


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | DoNotIncludeHome | |
| +-------+------------------+ |
| | 1     | IncludeHome      | |
| +-------+------------------+ |
|                              |
+------------------------------+





.. _parameters_EKF_:

EKF\_ Parameters
----------------


.. _EKF_ENABLE:

EKF\_ENABLE: Enable EKF1
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables EKF1 to be disabled when using alternative algorithms\. When disabling it\, the alternate EKF2 estimator must be enabled by setting EK2\_ENABLED \= 1 and flight control algorithms must be set to use the alternative estimator by setting AHRS\_EKF\_TYPE \= 2\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _EKF_VELNE_NOISE:

EKF\_VELNE\_NOISE: GPS horizontal velocity measurement noise scaler
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the horizontal velocity observation noise\. If the model of receiver used does not provide a speed accurcy estimate\, then a speed accuracy of 1 is assumed\. Increasing it reduces the weighting on these measurements\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.05 - 5.0 | 0.05      |
+------------+-----------+




.. _EKF_VELD_NOISE:

EKF\_VELD\_NOISE: GPS vertical velocity measurement noise scaler
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the vertical velocity observation noise\. If the model of receiver used does not provide a speed accurcy estimate\, then a speed accuracy of 1 is assumed\. Increasing it reduces the weighting on this measurement\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.05 - 5.0 | 0.05      |
+------------+-----------+




.. _EKF_POSNE_NOISE:

EKF\_POSNE\_NOISE: GPS horizontal position measurement noise \(m\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise in the GPS horizontal position measurements\. Increasing it reduces the weighting on these measurements\.


+------------+-----------+--------+
| Range      | Increment | Units  |
+============+===========+========+
| 0.1 - 10.0 | 0.1       | meters |
+------------+-----------+--------+




.. _EKF_ALT_NOISE:

EKF\_ALT\_NOISE: Altitude measurement noise \(m\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise in the altitude measurement\. Increasing it reduces the weighting on this measurement\.


+------------+-----------+--------+
| Range      | Increment | Units  |
+============+===========+========+
| 0.1 - 10.0 | 0.1       | meters |
+------------+-----------+--------+




.. _EKF_MAG_NOISE:

EKF\_MAG\_NOISE: Magnetometer measurement noise \(Gauss\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise in magnetometer measurements\. Increasing it reduces the weighting on these measurements\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.01 - 0.5 | 0.01      |
+------------+-----------+




.. _EKF_EAS_NOISE:

EKF\_EAS\_NOISE: Equivalent airspeed measurement noise \(m\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise in equivalent airspeed measurements\. Increasing it reduces the weighting on these measurements\.


+-----------+-----------+-------+
| Range     | Increment | Units |
+===========+===========+=======+
| 0.5 - 5.0 | 0.1       | m/s   |
+-----------+-----------+-------+




.. _EKF_WIND_PNOISE:

EKF\_WIND\_PNOISE: Wind velocity process noise \(m\/s\^2\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the growth of wind state error estimates\. Increasing it makes wind estimation faster and noisier\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.01 - 1.0 | 0.1       |
+------------+-----------+




.. _EKF_WIND_PSCALE:

EKF\_WIND\_PSCALE: Height rate to wind procss noise scaler
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Increasing this parameter increases how rapidly the wind states adapt when changing altitude\, but does make wind speed estimation noiser\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 1.0 | 0.1       |
+-----------+-----------+




.. _EKF_GYRO_PNOISE:

EKF\_GYRO\_PNOISE: Rate gyro noise \(rad\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the growth of estimated error due to gyro measurement errors excluding bias\. Increasing it makes the flter trust the gyro measurements less and other measurements more\.


+--------------+-----------+-------+
| Range        | Increment | Units |
+==============+===========+=======+
| 0.001 - 0.05 | 0.001     | rad/s |
+--------------+-----------+-------+




.. _EKF_ACC_PNOISE:

EKF\_ACC\_PNOISE: Accelerometer noise \(m\/s\^2\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias\. Increasing it makes the flter trust the accelerometer measurements less and other measurements more\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.05 - 1.0 | 0.01      | m/s/s |
+------------+-----------+-------+




.. _EKF_GBIAS_PNOISE:

EKF\_GBIAS\_PNOISE: Rate gyro bias process noise \(rad\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the growth of gyro bias state error estimates\. Increasing it makes rate gyro bias estimation faster and noisier\.


+---------------------+-------+
| Range               | Units |
+=====================+=======+
| 0.0000001 - 0.00001 | rad/s |
+---------------------+-------+




.. _EKF_ABIAS_PNOISE:

EKF\_ABIAS\_PNOISE: Accelerometer bias process noise \(m\/s\^2\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the growth of the vertical acelerometer bias state error estimate\. Increasing it makes accelerometer bias estimation faster and noisier\.


+-----------------+-------+
| Range           | Units |
+=================+=======+
| 0.00001 - 0.001 | m/s/s |
+-----------------+-------+




.. _EKF_MAGE_PNOISE:

EKF\_MAGE\_PNOISE: Earth magnetic field process noise \(gauss\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the growth of earth magnetic field state error estimates\. Increasing it makes earth magnetic field bias estimation faster and noisier\.


+---------------+---------+
| Range         | Units   |
+===============+=========+
| 0.0001 - 0.01 | gauss/s |
+---------------+---------+




.. _EKF_MAGB_PNOISE:

EKF\_MAGB\_PNOISE: Body magnetic field process noise \(gauss\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the growth of body magnetic field state error estimates\. Increasing it makes compass offset estimation faster and noisier\.


+---------------+---------+
| Range         | Units   |
+===============+=========+
| 0.0001 - 0.01 | gauss/s |
+---------------+---------+




.. _EKF_VEL_DELAY:

EKF\_VEL\_DELAY: GPS velocity measurement delay \(msec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the number of msec that the GPS velocity measurements lag behind the inertial measurements\.


+---------+-----------+--------------+
| Range   | Increment | Units        |
+=========+===========+==============+
| 0 - 500 | 10        | milliseconds |
+---------+-----------+--------------+




.. _EKF_POS_DELAY:

EKF\_POS\_DELAY: GPS position measurement delay \(msec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the number of msec that the GPS position measurements lag behind the inertial measurements\.


+---------+-----------+--------------+
| Range   | Increment | Units        |
+=========+===========+==============+
| 0 - 500 | 10        | milliseconds |
+---------+-----------+--------------+




.. _EKF_GPS_TYPE:

EKF\_GPS\_TYPE: GPS mode control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter controls use of GPS measurements \: 0 \= use 3D velocity \& 2D position\, 1 \= use 2D velocity and 2D position\, 2 \= use 2D position\, 3 \= use no GPS \(optical flow will be used if available\)


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | GPS 3D Vel and 2D Pos   | |
| +-------+-------------------------+ |
| | 1     | GPS 2D vel and 2D pos   | |
| +-------+-------------------------+ |
| | 2     | GPS 2D pos              | |
| +-------+-------------------------+ |
| | 3     | No GPS use optical flow | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _EKF_VEL_GATE:

EKF\_VEL\_GATE: GPS velocity measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check\. Decreasing it makes it more likely that good measurements willbe rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 1 - 100 | 1         |
+---------+-----------+




.. _EKF_POS_GATE:

EKF\_POS\_GATE: GPS position measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 1 - 100 | 1         |
+---------+-----------+




.. _EKF_HGT_GATE:

EKF\_HGT\_GATE: Height measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the number of standard deviations applied to the height measurement innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 1 - 100 | 1         |
+---------+-----------+




.. _EKF_MAG_GATE:

EKF\_MAG\_GATE: Magnetometer measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 1 - 100 | 1         |
+---------+-----------+




.. _EKF_EAS_GATE:

EKF\_EAS\_GATE: Airspeed measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 1 - 100 | 1         |
+---------+-----------+




.. _EKF_MAG_CAL:

EKF\_MAG\_CAL: Magnetometer calibration mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

EKF\_MAG\_CAL \= 0 enables calibration based on flying speed and altitude and is the default setting for Plane users\. EKF\_MAG\_CAL \= 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users\. EKF\_MAG\_CAL \= 2 prevents magnetometer calibration regardless of flight condition and is recommended if in\-flight magnetometer calibration is unreliable\.


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | Speed and Height | |
| +-------+------------------+ |
| | 1     | Acceleration     | |
| +-------+------------------+ |
| | 2     | Never            | |
| +-------+------------------+ |
| | 3     | Always           | |
| +-------+------------------+ |
|                              |
+------------------------------+




.. _EKF_GLITCH_ACCEL:

EKF\_GLITCH\_ACCEL: GPS glitch accel gate size \(cm\/s\^2\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter controls the maximum amount of difference in horizontal acceleration between the value predicted by the filter and the value measured by the GPS before the GPS position data is rejected\. If this value is set too low\, then valid GPS data will be regularly discarded\, and the position accuracy will degrade\. If this parameter is set too high\, then large GPS glitches will cause large rapid changes in position\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 100 - 500 | 50        |
+-----------+-----------+




.. _EKF_GLITCH_RAD:

EKF\_GLITCH\_RAD: GPS glitch radius gate size \(m\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter controls the maximum amount of difference in horizontal position \(in m\) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and the filter states are reset to the new GPS position\. Position steps smaller than this value will be temporarily ignored\, but will then be accepted and the filter will move to the new position\. Position steps larger than this value will be ignored initially\, but the filter will then apply an offset to the GPS position measurement\.


+---------+-----------+--------+
| Range   | Increment | Units  |
+=========+===========+========+
| 10 - 50 | 5         | meters |
+---------+-----------+--------+




.. _EKF_GND_GRADIENT:

EKF\_GND\_GRADIENT: Terrain Gradient \% RMS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the RMS terrain gradient percentage assumed by the terrain height estimation\. Terrain height can be estimated using optical flow and\/or range finder sensor data if fitted\. Smaller values cause the terrain height estimate to be slower to respond to changes in measurement\. Larger values cause the terrain height estimate to be faster to respond\, but also more noisy\. Generally this value can be reduced if operating over very flat terrain and increased if operating over uneven terrain\.


+--------+-----------+
| Range  | Increment |
+========+===========+
| 1 - 50 | 1         |
+--------+-----------+




.. _EKF_FLOW_NOISE:

EKF\_FLOW\_NOISE: Optical flow measurement noise \(rad\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise and errors in optical flow measurements\. Increasing it reduces the weighting on these measurements\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.05 - 1.0 | 0.05      | rad/s |
+------------+-----------+-------+




.. _EKF_FLOW_GATE:

EKF\_FLOW\_GATE: Optical Flow measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the number of standard deviations applied to the optical flow innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 1 - 100 | 1         |
+---------+-----------+




.. _EKF_FLOW_DELAY:

EKF\_FLOW\_DELAY: Optical Flow measurement delay \(msec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the number of msec that the optical flow measurements lag behind the inertial measurements\. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor\.


+---------+-----------+--------------+
| Range   | Increment | Units        |
+=========+===========+==============+
| 0 - 500 | 10        | milliseconds |
+---------+-----------+--------------+




.. _EKF_RNG_GATE:

EKF\_RNG\_GATE: Range finder measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the number of standard deviations applied to the range finder innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+---------+-----------+
| Range   | Increment |
+=========+===========+
| 1 - 100 | 1         |
+---------+-----------+




.. _EKF_MAX_FLOW:

EKF\_MAX\_FLOW: Maximum valid optical flow rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter sets the magnitude maximum optical flow rate in rad\/sec that will be accepted by the filter


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 1.0 - 4.0 | 0.1       |
+-----------+-----------+




.. _EKF_FALLBACK:

EKF\_FALLBACK: Fallback strictness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter controls the conditions necessary to trigger a fallback to DCM and INAV\. A value of 1 will cause fallbacks to occur on loss of GPS and other conditions\. A value of 0 will trust the EKF more\.


+----------------------------+
| Values                     |
+============================+
| +-------+----------------+ |
| | Value | Meaning        | |
| +=======+================+ |
| | 0     | Trust EKF more | |
| +-------+----------------+ |
| | 1     | Trust DCM more | |
| +-------+----------------+ |
|                            |
+----------------------------+




.. _EKF_ALT_SOURCE:

EKF\_ALT\_SOURCE: Primary height source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter controls which height sensor is used by the EKF during optical flow navigation \(when EKF\_GPS\_TYPE \= 3\)\. A value of will 0 cause it to always use baro altitude\. A value of 1 will cause it to use range finder if available\.


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | Use Baro         | |
| +-------+------------------+ |
| | 1     | Use Range Finder | |
| +-------+------------------+ |
|                              |
+------------------------------+




.. _EKF_GPS_CHECK:

EKF\_GPS\_CHECK: GPS preflight check
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

1 byte bitmap of GPS preflight checks to perform\. Set to 0 to bypass all checks\. Set to 255 perform all checks\. Set to 3 to check just the number of satellites and HDoP\. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving\, eg launch from a boat\.


+---------------------------+
| Bitmask                   |
+===========================+
| +-----+-----------------+ |
| | Bit | Meaning         | |
| +=====+=================+ |
| | 0   | NSats           | |
| +-----+-----------------+ |
| | 1   | HDoP            | |
| +-----+-----------------+ |
| | 2   | speed error     | |
| +-----+-----------------+ |
| | 3   | horiz pos error | |
| +-----+-----------------+ |
| | 4   | yaw error       | |
| +-----+-----------------+ |
| | 5   | pos drift       | |
| +-----+-----------------+ |
| | 6   | vert speed      | |
| +-----+-----------------+ |
| | 7   | horiz speed     | |
| +-----+-----------------+ |
|                           |
+---------------------------+





.. _parameters_EK2_:

EK2\_ Parameters
----------------


.. _EK2_ENABLE:

EK2\_ENABLE: Enable EKF2
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables EKF2\. Enabling EKF2 only makes the maths run\, it does not mean it will be used for flight control\. To use it for flight control set AHRS\_EKF\_TYPE\=2\. A reboot or restart will need to be performed after changing the value of EK2\_ENABLE for it to take effect\.


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Enabled  | |
| +-------+----------+ |
|                      |
+----------------------+




.. _EK2_GPS_TYPE:

EK2\_GPS\_TYPE: GPS mode control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls use of GPS measurements \: 0 \= use 3D velocity \& 2D position\, 1 \= use 2D velocity and 2D position\, 2 \= use 2D position\, 3 \= use no GPS \(optical flow will be used if available\)


+-------------------------------------+
| Values                              |
+=====================================+
| +-------+-------------------------+ |
| | Value | Meaning                 | |
| +=======+=========================+ |
| | 0     | GPS 3D Vel and 2D Pos   | |
| +-------+-------------------------+ |
| | 1     | GPS 2D vel and 2D pos   | |
| +-------+-------------------------+ |
| | 2     | GPS 2D pos              | |
| +-------+-------------------------+ |
| | 3     | No GPS use optical flow | |
| +-------+-------------------------+ |
|                                     |
+-------------------------------------+




.. _EK2_VELNE_NOISE:

EK2\_VELNE\_NOISE: GPS horizontal velocity measurement noise \(m\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set horizontal velocity observation noise\. If the model of receiver used does not provide a speed accurcy estimate\, then the parameter value will be used\. Increasing it reduces the weighting of the GPS horizontal velocity measurements\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.05 - 5.0 | 0.05      | m/s   |
+------------+-----------+-------+




.. _EK2_VELD_NOISE:

EK2\_VELD\_NOISE: GPS vertical velocity measurement noise \(m\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set vertical velocity observation noise\. If the model of receiver used does not provide a speed accurcy estimate\, then the parameter value will be used\. Increasing it reduces the weighting of the GPS vertical velocity measurements\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.05 - 5.0 | 0.05      | m/s   |
+------------+-----------+-------+




.. _EK2_VEL_GATE:

EK2\_VEL\_GATE: GPS velocity innovation gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check\. Decreasing it makes it more likely that good measurements willbe rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 100 - 1000 | 25        |
+------------+-----------+




.. _EK2_POSNE_NOISE:

EK2\_POSNE\_NOISE: GPS horizontal position measurement noise \(m\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the GPS horizontal position observation noise\. Increasing it reduces the weighting of GPS horizontal position measurements\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.1 - 10.0 | 0.1       | m     |
+------------+-----------+-------+




.. _EK2_POS_GATE:

EK2\_POS\_GATE: GPS position measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 100 - 1000 | 25        |
+------------+-----------+




.. _EK2_GLITCH_RAD:

EK2\_GLITCH\_RAD: GPS glitch radius gate size \(m\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS\. Making this value larger allows the filter to ignore larger GPS glitches but also means that non\-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position\.


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 10 - 100 | 5         | m     |
+----------+-----------+-------+




.. _EK2_GPS_DELAY:

EK2\_GPS\_DELAY: GPS measurement delay \(msec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the number of msec that the GPS measurements lag behind the inertial measurements\.


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 250 | 10        | msec  |
+---------+-----------+-------+




.. _EK2_ALT_SOURCE:

EK2\_ALT\_SOURCE: Primary height source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter controls which height sensor is used by the EKF\. If the selected optionn cannot be used\, it will default to Baro as the primary height source\. Setting 0 will use the baro altitude at all times\. Setting 1 uses the range finder and is only available in combination with optical flow navigation \(EK2\_GPS\_TYPE \= 3\)\. Setting 2 uses GPS\.


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | Use Baro         | |
| +-------+------------------+ |
| | 1     | Use Range Finder | |
| +-------+------------------+ |
| | 2     | Use GPS          | |
| +-------+------------------+ |
|                              |
+------------------------------+




.. _EK2_ALT_NOISE:

EK2\_ALT\_NOISE: Altitude measurement noise \(m\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise in the altitude measurement\. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors\, but will make it more sensitive to GPS and accelerometer errors\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.1 - 10.0 | 0.1       | m     |
+------------+-----------+-------+




.. _EK2_HGT_GATE:

EK2\_HGT\_GATE: Height measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the height measurement innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 100 - 1000 | 25        |
+------------+-----------+




.. _EK2_HGT_DELAY:

EK2\_HGT\_DELAY: Height measurement delay \(msec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the number of msec that the Height measurements lag behind the inertial measurements\.


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 250 | 10        | msec  |
+---------+-----------+-------+




.. _EK2_MAG_NOISE:

EK2\_MAG\_NOISE: Magnetometer measurement noise \(Gauss\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise in magnetometer measurements\. Increasing it reduces the weighting on these measurements\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.01 - 0.5 | 0.01      | gauss |
+------------+-----------+-------+




.. _EK2_MAG_CAL:

EK2\_MAG\_CAL: Magnetometer calibration mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

EKF\_MAG\_CAL \= 0 enables calibration when airborne and is the default setting for Plane users\. EKF\_MAG\_CAL \= 1 enables calibration when manoeuvreing\. EKF\_MAG\_CAL \= 2 prevents magnetometer calibration regardless of flight condition\, is recommended if the external magnetic field is varying and is the default for rovers\. EKF\_MAG\_CAL \= 3 enables calibration when the first in\-air field and yaw reset has completed and is the default for copters\. EKF\_MAG\_CAL \= 4 enables calibration all the time\. This determines when the filter will use the 3\-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states\. This model is only suitable for use when the external magnetic field environment is stable\.


+-----------------------------------------+
| Values                                  |
+=========================================+
| +-------+-----------------------------+ |
| | Value | Meaning                     | |
| +=======+=============================+ |
| | 0     | When flying                 | |
| +-------+-----------------------------+ |
| | 1     | When manoeuvring            | |
| +-------+-----------------------------+ |
| | 2     | Never                       | |
| +-------+-----------------------------+ |
| | 3     | After first climb yaw reset | |
| +-------+-----------------------------+ |
| | 4     | Always                      | |
| +-------+-----------------------------+ |
|                                         |
+-----------------------------------------+




.. _EK2_MAG_GATE:

EK2\_MAG\_GATE: Magnetometer measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the magnetometer measurement innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 100 - 1000 | 25        |
+------------+-----------+




.. _EK2_EAS_NOISE:

EK2\_EAS\_NOISE: Equivalent airspeed measurement noise \(m\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise in equivalent airspeed measurements used by planes\. Increasing it reduces the weighting of airspeed measurements and will make wind speed estimates less noisy and slower to converge\. Increasing also increases navigation errors when dead\-reckoning without GPS measurements\.


+-----------+-----------+-------+
| Range     | Increment | Units |
+===========+===========+=======+
| 0.5 - 5.0 | 0.1       | m/s   |
+-----------+-----------+-------+




.. _EK2_EAS_GATE:

EK2\_EAS\_GATE: Airspeed measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the airspeed measurement innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 100 - 1000 | 25        |
+------------+-----------+




.. _EK2_RNG_NOISE:

EK2\_RNG\_NOISE: Range finder measurement noise \(m\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise in the range finder measurement\. Increasing it reduces the weighting on this measurement\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.1 - 10.0 | 0.1       | m     |
+------------+-----------+-------+




.. _EK2_RNG_GATE:

EK2\_RNG\_GATE: Range finder measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the range finder innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 100 - 1000 | 25        |
+------------+-----------+




.. _EK2_MAX_FLOW:

EK2\_MAX\_FLOW: Maximum valid optical flow rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the magnitude maximum optical flow rate in rad\/sec that will be accepted by the filter


+-----------+-----------+-------+
| Range     | Increment | Units |
+===========+===========+=======+
| 1.0 - 4.0 | 0.1       | rad/s |
+-----------+-----------+-------+




.. _EK2_FLOW_NOISE:

EK2\_FLOW\_NOISE: Optical flow measurement noise \(rad\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the RMS value of noise and errors in optical flow measurements\. Increasing it reduces the weighting on these measurements\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.05 - 1.0 | 0.05      | rad/s |
+------------+-----------+-------+




.. _EK2_FLOW_GATE:

EK2\_FLOW\_GATE: Optical Flow measurement gate size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the percentage number of standard deviations applied to the optical flow innovation consistency check\. Decreasing it makes it more likely that good measurements will be rejected\. Increasing it makes it more likely that bad measurements will be accepted\.


+------------+-----------+
| Range      | Increment |
+============+===========+
| 100 - 1000 | 25        |
+------------+-----------+




.. _EK2_FLOW_DELAY:

EK2\_FLOW\_DELAY: Optical Flow measurement delay \(msec\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the number of msec that the optical flow measurements lag behind the inertial measurements\. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor\.


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 250 | 10        | msec  |
+---------+-----------+-------+




.. _EK2_GYRO_PNOISE:

EK2\_GYRO\_PNOISE: Rate gyro noise \(rad\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This control disturbance noise controls the growth of estimated error due to gyro measurement errors excluding bias\. Increasing it makes the flter trust the gyro measurements less and other measurements more\.


+--------------+-----------+-------+
| Range        | Increment | Units |
+==============+===========+=======+
| 0.0001 - 0.1 | 0.0001    | rad/s |
+--------------+-----------+-------+




.. _EK2_ACC_PNOISE:

EK2\_ACC\_PNOISE: Accelerometer noise \(m\/s\^2\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This control disturbance noise controls the growth of estimated error due to accelerometer measurement errors excluding bias\. Increasing it makes the flter trust the accelerometer measurements less and other measurements more\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.01 - 1.0 | 0.01      | m/s/s |
+------------+-----------+-------+




.. _EK2_GBIAS_PNOISE:

EK2\_GBIAS\_PNOISE: Rate gyro bias stability \(rad\/s\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This state  process noise controls growth of the gyro delta angle bias state error estimate\. Increasing it makes rate gyro bias estimation faster and noisier\.


+-----------------+---------+
| Range           | Units   |
+=================+=========+
| 0.00001 - 0.001 | rad/s/s |
+-----------------+---------+




.. _EK2_GSCL_PNOISE:

EK2\_GSCL\_PNOISE: Rate gyro scale factor stability \(1\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the rate of gyro scale factor learning\. Increasing it makes rate gyro scale factor estimation faster and noisier\.


+------------------+-------+
| Range            | Units |
+==================+=======+
| 0.000001 - 0.001 | 1/s   |
+------------------+-------+




.. _EK2_ABIAS_PNOISE:

EK2\_ABIAS\_PNOISE: Accelerometer bias stability \(m\/s\^3\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate\. Increasing it makes accelerometer bias estimation faster and noisier\.


+-----------------+---------+
| Range           | Units   |
+=================+=========+
| 0.00001 - 0.001 | m/s/s/s |
+-----------------+---------+




.. _EK2_MAG_PNOISE:

EK2\_MAG\_PNOISE: Magnetic field process noise \(gauss\/s\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This state process noise controls the growth of magnetic field state error estimates\. Increasing it makes magnetic field bias estimation faster and noisier\.


+---------------+---------+
| Range         | Units   |
+===============+=========+
| 0.0001 - 0.01 | gauss/s |
+---------------+---------+




.. _EK2_WIND_PNOISE:

EK2\_WIND\_PNOISE: Wind velocity process noise \(m\/s\^2\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This state process noise controls the growth of wind state error estimates\. Increasing it makes wind estimation faster and noisier\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 0.01 - 1.0 | 0.1       | m/s/s |
+------------+-----------+-------+




.. _EK2_WIND_PSCALE:

EK2\_WIND\_PSCALE: Height rate to wind procss noise scaler
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude\. Increasing this parameter increases how rapidly the wind states adapt when changing altitude\, but does make wind velocity estimation noiser\.


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 1.0 | 0.1       |
+-----------+-----------+




.. _EK2_GPS_CHECK:

EK2\_GPS\_CHECK: GPS preflight check
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is a 1 byte bitmap controlling which GPS preflight checks are performed\. Set to 0 to bypass all checks\. Set to 255 perform all checks\. Set to 3 to check just the number of satellites and HDoP\. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving\, eg launch from a boat\.


+---------------------------+
| Bitmask                   |
+===========================+
| +-----+-----------------+ |
| | Bit | Meaning         | |
| +=====+=================+ |
| | 0   | NSats           | |
| +-----+-----------------+ |
| | 1   | HDoP            | |
| +-----+-----------------+ |
| | 2   | speed error     | |
| +-----+-----------------+ |
| | 3   | horiz pos error | |
| +-----+-----------------+ |
| | 4   | yaw error       | |
| +-----+-----------------+ |
| | 5   | pos drift       | |
| +-----+-----------------+ |
| | 6   | vert speed      | |
| +-----+-----------------+ |
| | 7   | horiz speed     | |
| +-----+-----------------+ |
|                           |
+---------------------------+




.. _EK2_IMU_MASK:

EK2\_IMU\_MASK: Bitmask of active IMUs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

1 byte bitmap of IMUs to use in EKF2\. A separate instance of EKF2 will be started for each IMU selected\. Set to 1 to use the first IMU only \(default\)\, set to 2 to use the second IMU only\, set to 3 to use the first and second IMU\. Additional IMU\'s can be used up to a maximum of 6 if memory and processing resources permit\. There may be insufficient memory and processing resources to run multiple instances\. If this occurs EKF2 will fail to start\.


+---------+
| Range   |
+=========+
| 1 - 127 |
+---------+




.. _EK2_CHECK_SCALE:

EK2\_CHECK\_SCALE: GPS accuracy check scaler \(\%\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This scales the thresholds that are used to check GPS accuracy before it is used by the EKF\. A value of 100 is the default\. Values greater than 100 increase and values less than 100 reduce the maximum GPS error the EKF will accept\. A value of 200 will double the allowable GPS error\.


+----------+-------+
| Range    | Units |
+==========+=======+
| 50 - 200 | %     |
+----------+-------+




.. _EK2_NOAID_NOISE:

EK2\_NOAID\_NOISE: Non\-GPS operation position uncertainty \(m\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the amount of position variation that the EKF allows for when operating without external measurements \(eg GPS or optical flow\)\. Increasing this parameter makes the EKF attitude estimate less sensitive to vehicle manoeuvres but more sensitive to IMU errors\.


+------------+-------+
| Range      | Units |
+============+=======+
| 0.5 - 50.0 | m/s   |
+------------+-------+




.. _EK2_LOG_MASK:

EK2\_LOG\_MASK: EKF sensor logging IMU mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets the IMU mask of sensors to do full logging for


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | Disabled          | |
| +-------+-------------------+ |
| | 1     | FirstIMU          | |
| +-------+-------------------+ |
| | 3     | FirstAndSecondIMU | |
| +-------+-------------------+ |
| | 7     | AllIMUs           | |
| +-------+-------------------+ |
|                               |
+-------------------------------+





.. _parameters_RPM:

RPM Parameters
--------------


.. _RPM_TYPE:

RPM\_TYPE: RPM type
~~~~~~~~~~~~~~~~~~~


What type of RPM sensor is connected


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | None    | |
| +-------+---------+ |
| | 1     | PX4-PWM | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RPM_SCALING:

RPM\_SCALING: RPM scaling
~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between sensor reading and RPM\.


+-----------+
| Increment |
+===========+
| 0.001     |
+-----------+




.. _RPM_MAX:

RPM\_MAX: Maximum RPM
~~~~~~~~~~~~~~~~~~~~~


Maximum RPM to report


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM_MIN:

RPM\_MIN: Minimum RPM
~~~~~~~~~~~~~~~~~~~~~


Minimum RPM to report


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM_MIN_QUAL:

RPM\_MIN\_QUAL: Minimum Quality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum data quality to be used


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _RPM2_TYPE:

RPM2\_TYPE: Second RPM type
~~~~~~~~~~~~~~~~~~~~~~~~~~~


What type of RPM sensor is connected


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | None    | |
| +-------+---------+ |
| | 1     | PX4-PWM | |
| +-------+---------+ |
|                     |
+---------------------+




.. _RPM2_SCALING:

RPM2\_SCALING: RPM scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between sensor reading and RPM\.


+-----------+
| Increment |
+===========+
| 0.001     |
+-----------+





.. _parameters_RSSI_:

RSSI\_ Parameters
-----------------


.. _RSSI_TYPE:

RSSI\_TYPE: RSSI Type
~~~~~~~~~~~~~~~~~~~~~


Radio Receiver RSSI type\. If your radio receiver supports RSSI of some kind\, set it here\, then set its associated RSSI\_XXXXX parameters\, if any\.


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | Disabled          | |
| +-------+-------------------+ |
| | 1     | AnalogPin         | |
| +-------+-------------------+ |
| | 2     | RCChannelPwmValue | |
| +-------+-------------------+ |
|                               |
+-------------------------------+




.. _RSSI_ANA_PIN:

RSSI\_ANA\_PIN: Receiver RSSI analog sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This selects an analog pin where the receiver RSSI voltage will be read\.


+--------------------------+
| Values                   |
+==========================+
| +-------+--------------+ |
| | Value | Meaning      | |
| +=======+==============+ |
| | 0     | APM2 A0      | |
| +-------+--------------+ |
| | 1     | APM2 A1      | |
| +-------+--------------+ |
| | 13    | APM2 A13     | |
| +-------+--------------+ |
| | 11    | Pixracer     | |
| +-------+--------------+ |
| | 13    | Pixhawk ADC4 | |
| +-------+--------------+ |
| | 14    | Pixhawk ADC3 | |
| +-------+--------------+ |
| | 15    | Pixhawk ADC6 | |
| +-------+--------------+ |
| | 103   | Pixhawk SBUS | |
| +-------+--------------+ |
|                          |
+--------------------------+




.. _RSSI_PIN_LOW:

RSSI\_PIN\_LOW: Receiver RSSI voltage low
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the voltage value that the radio receiver will put on the RSSI\_ANA\_PIN when the signal strength is the weakest\. Since some radio receivers put out inverted values from what you might otherwise expect\, this isn\'t necessarily a lower value than RSSI\_PIN\_HIGH\. 


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 5.0 | 0.01      | Volt  |
+---------+-----------+-------+




.. _RSSI_PIN_HIGH:

RSSI\_PIN\_HIGH: Receiver RSSI voltage high
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the voltage value that the radio receiver will put on the RSSI\_ANA\_PIN when the signal strength is the strongest\. Since some radio receivers put out inverted values from what you might otherwise expect\, this isn\'t necessarily a higher value than RSSI\_PIN\_LOW\. 


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 5.0 | 0.01      | Volt  |
+---------+-----------+-------+




.. _RSSI_CHANNEL:

RSSI\_CHANNEL: Receiver RSSI channel number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The channel number where RSSI will be output by the radio receiver \(5 and above\)\.


+-------+
| Units |
+=======+
|       |
+-------+




.. _RSSI_CHAN_LOW:

RSSI\_CHAN\_LOW: Receiver RSSI PWM low value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the PWM value that the radio receiver will put on the RSSI\_CHANNEL when the signal strength is the weakest\. Since some radio receivers put out inverted values from what you might otherwise expect\, this isn\'t necessarily a lower value than RSSI\_CHAN\_HIGH\. 


+----------+--------------+
| Range    | Units        |
+==========+==============+
| 0 - 2000 | Microseconds |
+----------+--------------+




.. _RSSI_CHAN_HIGH:

RSSI\_CHAN\_HIGH: Receiver RSSI PWM high value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the PWM value that the radio receiver will put on the RSSI\_CHANNEL when the signal strength is the strongest\. Since some radio receivers put out inverted values from what you might otherwise expect\, this isn\'t necessarily a higher value than RSSI\_CHAN\_LOW\. 


+----------+--------------+
| Range    | Units        |
+==========+==============+
| 0 - 2000 | Microseconds |
+----------+--------------+





.. _parameters_NTF_:

NTF\_ Parameters
----------------


.. _NTF_LED_BRIGHT:

NTF\_LED\_BRIGHT: LED Brightness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Select the RGB LED brightness level\. When USB is connected brightness will never be higher than low regardless of the setting\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Off     | |
| +-------+---------+ |
| | 1     | Low     | |
| +-------+---------+ |
| | 2     | Medium  | |
| +-------+---------+ |
| | 3     | High    | |
| +-------+---------+ |
|                     |
+---------------------+




.. _NTF_BUZZ_ENABLE:

NTF\_BUZZ\_ENABLE: Buzzer enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable or disable the buzzer\. Only for Linux and PX4 based boards\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Disable | |
| +-------+---------+ |
| | 1     | Enable  | |
| +-------+---------+ |
|                     |
+---------------------+



