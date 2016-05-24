.. Dynamically generated list of documented parameters
.. This page was generated using Tools\/autotest\/param\_metadata\/param\_parse\.py

.. DO NOT EDIT


.. _parameters:

Complete Parameter List
=======================

This is a complete list of the parameters which can be set \(e\.g\. via the MAVLink protocol\) to control vehicle behaviour\. They are stored in persistent storage on the vehicle\.

This list is automatically generated from the latest ardupilot source code\, and so may contain parameters which are not yet in the stable released versions of the code\.




.. _parameters_ArduCopter:

ArduCopter Parameters
---------------------


.. _SYSID_SW_MREV:

SYSID\_SW\_MREV: Eeprom format version number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This value is incremented when changes are made to the eeprom format


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _SYSID_SW_TYPE:

SYSID\_SW\_TYPE: Software Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is used by the ground station to recognise the software type \(eg ArduPlane vs ArduCopter\)


+----------+----------------------------+
| ReadOnly | Values                     |
+==========+============================+
| True     | +-------+----------------+ |
|          | | Value | Meaning        | |
|          | +=======+================+ |
|          | | 0     | ArduPlane      | |
|          | +-------+----------------+ |
|          | | 4     | AntennaTracker | |
|          | +-------+----------------+ |
|          | | 10    | Copter         | |
|          | +-------+----------------+ |
|          | | 20    | Rover          | |
|          | +-------+----------------+ |
|          |                            |
+----------+----------------------------+




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

SYSID\_MYGCS: My ground station number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Allows restricting radio overrides to only come from my ground station


+----------------------------------------------+
| Values                                       |
+==============================================+
| +-------+----------------------------------+ |
| | Value | Meaning                          | |
| +=======+==================================+ |
| | 255   | Mission Planner and DroidPlanner | |
| +-------+----------------------------------+ |
| | 252   | AP Planner 2                     | |
| +-------+----------------------------------+ |
|                                              |
+----------------------------------------------+




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




.. _PILOT_THR_FILT:

PILOT\_THR\_FILT: Throttle filter cutoff
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Throttle filter cutoff \(Hz\) \- active whenever altitude control is inactive \- 0 to disable


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | .5        | Hz    |
+--------+-----------+-------+




.. _PILOT_TKOFF_ALT:

PILOT\_TKOFF\_ALT: Pilot takeoff altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick\.


+--------------+-----------+-------------+
| Range        | Increment | Units       |
+==============+===========+=============+
| 0.0 - 1000.0 | 10        | Centimeters |
+--------------+-----------+-------------+




.. _PILOT_TKOFF_DZ:

PILOT\_TKOFF\_DZ: Takeoff trigger deadzone
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset from mid stick at which takeoff is triggered


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.0 - 500.0 | 10        |
+-------------+-----------+




.. _PILOT_THR_BHV:

PILOT\_THR\_BHV: Throttle stick behavior
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Bitmask containing various throttle stick options\. Add up the values for options that you want\.


+-----------------------------------------+-------------------------------------------+
| Bitmask                                 | Values                                    |
+=========================================+===========================================+
| +-----+-------------------------------+ | +-------+-------------------------------+ |
| | Bit | Meaning                       | | | Value | Meaning                       | |
| +=====+===============================+ | +=======+===============================+ |
| | 0   | Feedback from mid stick       | | | 0     | None                          | |
| +-----+-------------------------------+ | +-------+-------------------------------+ |
| | 1   | High throttle cancels landing | | | 1     | Feedback from mid stick       | |
| +-----+-------------------------------+ | +-------+-------------------------------+ |
| | 2   | Disarm on land detection      | | | 2     | High throttle cancels landing | |
| +-----+-------------------------------+ | +-------+-------------------------------+ |
|                                         | | 4     | Disarm on land detection      | |
|                                         | +-------+-------------------------------+ |
|                                         |                                           |
+-----------------------------------------+-------------------------------------------+




.. _TELEM_DELAY:

TELEM\_DELAY: Telemetry startup delay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

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




.. _RTL_ALT:

RTL\_ALT: RTL Altitude
~~~~~~~~~~~~~~~~~~~~~~


The minimum relative altitude the model will move to before Returning to Launch\.  Set to zero to return at current altitude\.


+----------+-----------+-------------+
| Range    | Increment | Units       |
+==========+===========+=============+
| 0 - 8000 | 1         | Centimeters |
+----------+-----------+-------------+




.. _RTL_CONE_SLOPE:

RTL\_CONE\_SLOPE: RTL cone slope
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines a cone above home which determines maximum climb


+------------+----------------------+-----------+
| Range      | Values               | Increment |
+============+======================+===========+
| 0.5 - 10.0 | +-------+----------+ | .1        |
|            | | Value | Meaning  | |           |
|            | +=======+==========+ |           |
|            | | 0     | Disabled | |           |
|            | +-------+----------+ |           |
|            | | 1     | Shallow  | |           |
|            | +-------+----------+ |           |
|            | | 3     | Steep    | |           |
|            | +-------+----------+ |           |
|            |                      |           |
+------------+----------------------+-----------+




.. _RTL_SPEED:

RTL\_SPEED: RTL speed
~~~~~~~~~~~~~~~~~~~~~


Defines the speed in cm\/s which the aircraft will attempt to maintain horizontally while flying home\. If this is set to zero\, WPNAV\_SPEED will be used instead\.


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 0 - 2000 | 50        | cm/s  |
+----------+-----------+-------+




.. _RNGFND_GAIN:

RNGFND\_GAIN: Rangefinder gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.01 - 2.0 | 0.01      |
+------------+-----------+




.. _FS_BATT_ENABLE:

FS\_BATT\_ENABLE: Battery Failsafe Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls whether failsafe will be invoked when battery voltage or current runs low


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | Disabled | |
| +-------+----------+ |
| | 1     | Land     | |
| +-------+----------+ |
| | 2     | RTL      | |
| +-------+----------+ |
|                      |
+----------------------+




.. _FS_BATT_VOLTAGE:

FS\_BATT\_VOLTAGE: Failsafe battery voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery voltage to trigger failsafe\. Set to 0 to disable battery voltage failsafe\. If the battery voltage drops below this voltage then the copter will RTL


+-----------+-------+
| Increment | Units |
+===========+=======+
| 0.1       | Volts |
+-----------+-------+




.. _FS_BATT_MAH:

FS\_BATT\_MAH: Failsafe battery milliAmpHours
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery capacity remaining to trigger failsafe\. Set to 0 to disable battery remaining failsafe\. If the battery remaining drops below this level then the copter will RTL


+-----------+-------+
| Increment | Units |
+===========+=======+
| 50        | mAh   |
+-----------+-------+




.. _FS_GCS_ENABLE:

FS\_GCS\_ENABLE: Ground Station Failsafe Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls whether failsafe will be invoked \(and what action to take\) when connection with Ground station is lost for at least 5 seconds\. NB\. The GCS Failsafe is only active when RC\_OVERRIDE is being used to control the vehicle\.


+--------------------------------------------------------+
| Values                                                 |
+========================================================+
| +-------+--------------------------------------------+ |
| | Value | Meaning                                    | |
| +=======+============================================+ |
| | 0     | Disabled                                   | |
| +-------+--------------------------------------------+ |
| | 1     | Enabled always RTL                         | |
| +-------+--------------------------------------------+ |
| | 2     | Enabled Continue with Mission in Auto Mode | |
| +-------+--------------------------------------------+ |
|                                                        |
+--------------------------------------------------------+




.. _GPS_HDOP_GOOD:

GPS\_HDOP\_GOOD: GPS Hdop Good
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

GPS Hdop value at or below this value represent a good position\.  Used for pre\-arm checks


+-----------+
| Range     |
+===========+
| 100 - 900 |
+-----------+




.. _MAG_ENABLE:

MAG\_ENABLE: Compass enable\/disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting this to Enabled\(1\) will enable the compass\. Setting this to Disabled\(0\) will disable the compass


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




.. _SUPER_SIMPLE:

SUPER\_SIMPLE: Super Simple Mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Bitmask to enable Super Simple mode for some flight modes\. Setting this to Disabled\(0\) will disable Super Simple Mode


+-----------------------------+
| Values                      |
+=============================+
| +-------+-----------------+ |
| | Value | Meaning         | |
| +=======+=================+ |
| | 0     | Disabled        | |
| +-------+-----------------+ |
| | 1     | Mode1           | |
| +-------+-----------------+ |
| | 2     | Mode2           | |
| +-------+-----------------+ |
| | 3     | Mode1+2         | |
| +-------+-----------------+ |
| | 4     | Mode3           | |
| +-------+-----------------+ |
| | 5     | Mode1+3         | |
| +-------+-----------------+ |
| | 6     | Mode2+3         | |
| +-------+-----------------+ |
| | 7     | Mode1+2+3       | |
| +-------+-----------------+ |
| | 8     | Mode4           | |
| +-------+-----------------+ |
| | 9     | Mode1+4         | |
| +-------+-----------------+ |
| | 10    | Mode2+4         | |
| +-------+-----------------+ |
| | 11    | Mode1+2+4       | |
| +-------+-----------------+ |
| | 12    | Mode3+4         | |
| +-------+-----------------+ |
| | 13    | Mode1+3+4       | |
| +-------+-----------------+ |
| | 14    | Mode2+3+4       | |
| +-------+-----------------+ |
| | 15    | Mode1+2+3+4     | |
| +-------+-----------------+ |
| | 16    | Mode5           | |
| +-------+-----------------+ |
| | 17    | Mode1+5         | |
| +-------+-----------------+ |
| | 18    | Mode2+5         | |
| +-------+-----------------+ |
| | 19    | Mode1+2+5       | |
| +-------+-----------------+ |
| | 20    | Mode3+5         | |
| +-------+-----------------+ |
| | 21    | Mode1+3+5       | |
| +-------+-----------------+ |
| | 22    | Mode2+3+5       | |
| +-------+-----------------+ |
| | 23    | Mode1+2+3+5     | |
| +-------+-----------------+ |
| | 24    | Mode4+5         | |
| +-------+-----------------+ |
| | 25    | Mode1+4+5       | |
| +-------+-----------------+ |
| | 26    | Mode2+4+5       | |
| +-------+-----------------+ |
| | 27    | Mode1+2+4+5     | |
| +-------+-----------------+ |
| | 28    | Mode3+4+5       | |
| +-------+-----------------+ |
| | 29    | Mode1+3+4+5     | |
| +-------+-----------------+ |
| | 30    | Mode2+3+4+5     | |
| +-------+-----------------+ |
| | 31    | Mode1+2+3+4+5   | |
| +-------+-----------------+ |
| | 32    | Mode6           | |
| +-------+-----------------+ |
| | 33    | Mode1+6         | |
| +-------+-----------------+ |
| | 34    | Mode2+6         | |
| +-------+-----------------+ |
| | 35    | Mode1+2+6       | |
| +-------+-----------------+ |
| | 36    | Mode3+6         | |
| +-------+-----------------+ |
| | 37    | Mode1+3+6       | |
| +-------+-----------------+ |
| | 38    | Mode2+3+6       | |
| +-------+-----------------+ |
| | 39    | Mode1+2+3+6     | |
| +-------+-----------------+ |
| | 40    | Mode4+6         | |
| +-------+-----------------+ |
| | 41    | Mode1+4+6       | |
| +-------+-----------------+ |
| | 42    | Mode2+4+6       | |
| +-------+-----------------+ |
| | 43    | Mode1+2+4+6     | |
| +-------+-----------------+ |
| | 44    | Mode3+4+6       | |
| +-------+-----------------+ |
| | 45    | Mode1+3+4+6     | |
| +-------+-----------------+ |
| | 46    | Mode2+3+4+6     | |
| +-------+-----------------+ |
| | 47    | Mode1+2+3+4+6   | |
| +-------+-----------------+ |
| | 48    | Mode5+6         | |
| +-------+-----------------+ |
| | 49    | Mode1+5+6       | |
| +-------+-----------------+ |
| | 50    | Mode2+5+6       | |
| +-------+-----------------+ |
| | 51    | Mode1+2+5+6     | |
| +-------+-----------------+ |
| | 52    | Mode3+5+6       | |
| +-------+-----------------+ |
| | 53    | Mode1+3+5+6     | |
| +-------+-----------------+ |
| | 54    | Mode2+3+5+6     | |
| +-------+-----------------+ |
| | 55    | Mode1+2+3+5+6   | |
| +-------+-----------------+ |
| | 56    | Mode4+5+6       | |
| +-------+-----------------+ |
| | 57    | Mode1+4+5+6     | |
| +-------+-----------------+ |
| | 58    | Mode2+4+5+6     | |
| +-------+-----------------+ |
| | 59    | Mode1+2+4+5+6   | |
| +-------+-----------------+ |
| | 60    | Mode3+4+5+6     | |
| +-------+-----------------+ |
| | 61    | Mode1+3+4+5+6   | |
| +-------+-----------------+ |
| | 62    | Mode2+3+4+5+6   | |
| +-------+-----------------+ |
| | 63    | Mode1+2+3+4+5+6 | |
| +-------+-----------------+ |
|                             |
+-----------------------------+




.. _RTL_ALT_FINAL:

RTL\_ALT\_FINAL: RTL Final Altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission\.  Set to zero to land\.


+-----------+-----------+-------------+
| Range     | Increment | Units       |
+===========+===========+=============+
| -1 - 1000 | 1         | Centimeters |
+-----------+-----------+-------------+




.. _RTL_CLIMB_MIN:

RTL\_CLIMB\_MIN: RTL minimum climb
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The vehicle will climb this many cm during the initial climb portion of the RTL


+----------+-----------+-------------+
| Range    | Increment | Units       |
+==========+===========+=============+
| 0 - 3000 | 10        | Centimeters |
+----------+-----------+-------------+




.. _WP_YAW_BEHAVIOR:

WP\_YAW\_BEHAVIOR: Yaw behaviour during missions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Determines how the autopilot controls the yaw during missions and RTL


+-------------------------------------------+
| Values                                    |
+===========================================+
| +-------+-------------------------------+ |
| | Value | Meaning                       | |
| +=======+===============================+ |
| | 0     | Never change yaw              | |
| +-------+-------------------------------+ |
| | 1     | Face next waypoint            | |
| +-------+-------------------------------+ |
| | 2     | Face next waypoint except RTL | |
| +-------+-------------------------------+ |
| | 3     | Face along GPS course         | |
| +-------+-------------------------------+ |
|                                           |
+-------------------------------------------+




.. _RTL_LOIT_TIME:

RTL\_LOIT\_TIME: RTL loiter time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Time \(in milliseconds\) to loiter above home before beginning final descent


+-----------+-----------+-------+
| Range     | Increment | Units |
+===========+===========+=======+
| 0 - 60000 | 1000      | ms    |
+-----------+-----------+-------+




.. _LAND_SPEED:

LAND\_SPEED: Land speed
~~~~~~~~~~~~~~~~~~~~~~~


The descent speed for the final stage of landing in cm\/s


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 30 - 200 | 10        | cm/s  |
+----------+-----------+-------+




.. _LAND_SPEED_HIGH:

LAND\_SPEED\_HIGH: Land speed high
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The descent speed for the first stage of landing in cm\/s\. If this is zero then WPNAV\_SPEED\_DN is used


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 500 | 10        | cm/s  |
+---------+-----------+-------+




.. _PILOT_VELZ_MAX:

PILOT\_VELZ\_MAX: Pilot maximum vertical speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The maximum vertical velocity the pilot may request in cm\/s


+----------+-----------+--------------------+
| Range    | Increment | Units              |
+==========+===========+====================+
| 50 - 500 | 10        | Centimeters/Second |
+----------+-----------+--------------------+




.. _PILOT_ACCEL_Z:

PILOT\_ACCEL\_Z: Pilot vertical acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The vertical acceleration used when pilot is controlling the altitude


+----------+-----------+--------+
| Range    | Increment | Units  |
+==========+===========+========+
| 50 - 500 | 10        | cm/s/s |
+----------+-----------+--------+




.. _THR_MIN:

THR\_MIN: Throttle Minimum
~~~~~~~~~~~~~~~~~~~~~~~~~~


The minimum throttle that will be sent to the motors to keep them spinning


+---------+-----------+------------+
| Range   | Increment | Units      |
+=========+===========+============+
| 0 - 300 | 1         | Percent*10 |
+---------+-----------+------------+




.. _FS_THR_ENABLE:

FS\_THR\_ENABLE: Throttle Failsafe Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel


+--------------------------------------------------------+
| Values                                                 |
+========================================================+
| +-------+--------------------------------------------+ |
| | Value | Meaning                                    | |
| +=======+============================================+ |
| | 0     | Disabled                                   | |
| +-------+--------------------------------------------+ |
| | 1     | Enabled always RTL                         | |
| +-------+--------------------------------------------+ |
| | 2     | Enabled Continue with Mission in Auto Mode | |
| +-------+--------------------------------------------+ |
| | 3     | Enabled always LAND                        | |
| +-------+--------------------------------------------+ |
|                                                        |
+--------------------------------------------------------+




.. _FS_THR_VALUE:

FS\_THR\_VALUE: Throttle Failsafe Value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The PWM level on channel 3 below which throttle sailsafe triggers


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 925 - 1100 | 1         | pwm   |
+------------+-----------+-------+




.. _THR_MID:

THR\_MID: Throttle Mid Position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The throttle output \(0 \~ 1000\) when throttle stick is in mid position\.  Used to scale the manual throttle so that the mid throttle stick position is close to the throttle required to hover


+-----------+-----------+------------+
| Range     | Increment | Units      |
+===========+===========+============+
| 300 - 700 | 10        | Percent*10 |
+-----------+-----------+------------+




.. _THR_DZ:

THR\_DZ: Throttle deadzone
~~~~~~~~~~~~~~~~~~~~~~~~~~


The deadzone above and below mid throttle\.  Used in AltHold\, Loiter\, PosHold flight modes


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 300 | 1         | pwm   |
+---------+-----------+-------+




.. _FLTMODE1:

FLTMODE1: Flight Mode 1
~~~~~~~~~~~~~~~~~~~~~~~


Flight mode when Channel 5 pwm is \<\= 1230


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Stabilize | |
| +-------+-----------+ |
| | 1     | Acro      | |
| +-------+-----------+ |
| | 2     | AltHold   | |
| +-------+-----------+ |
| | 3     | Auto      | |
| +-------+-----------+ |
| | 4     | Guided    | |
| +-------+-----------+ |
| | 5     | Loiter    | |
| +-------+-----------+ |
| | 6     | RTL       | |
| +-------+-----------+ |
| | 7     | Circle    | |
| +-------+-----------+ |
| | 9     | Land      | |
| +-------+-----------+ |
| | 11    | Drift     | |
| +-------+-----------+ |
| | 13    | Sport     | |
| +-------+-----------+ |
| | 14    | Flip      | |
| +-------+-----------+ |
| | 15    | AutoTune  | |
| +-------+-----------+ |
| | 16    | PosHold   | |
| +-------+-----------+ |
| | 17    | Brake     | |
| +-------+-----------+ |
| | 18    | Throw     | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE2:

FLTMODE2: Flight Mode 2
~~~~~~~~~~~~~~~~~~~~~~~


Flight mode when Channel 5 pwm is \>1230\, \<\= 1360


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Stabilize | |
| +-------+-----------+ |
| | 1     | Acro      | |
| +-------+-----------+ |
| | 2     | AltHold   | |
| +-------+-----------+ |
| | 3     | Auto      | |
| +-------+-----------+ |
| | 4     | Guided    | |
| +-------+-----------+ |
| | 5     | Loiter    | |
| +-------+-----------+ |
| | 6     | RTL       | |
| +-------+-----------+ |
| | 7     | Circle    | |
| +-------+-----------+ |
| | 9     | Land      | |
| +-------+-----------+ |
| | 11    | Drift     | |
| +-------+-----------+ |
| | 13    | Sport     | |
| +-------+-----------+ |
| | 14    | Flip      | |
| +-------+-----------+ |
| | 15    | AutoTune  | |
| +-------+-----------+ |
| | 16    | PosHold   | |
| +-------+-----------+ |
| | 17    | Brake     | |
| +-------+-----------+ |
| | 18    | Throw     | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE3:

FLTMODE3: Flight Mode 3
~~~~~~~~~~~~~~~~~~~~~~~


Flight mode when Channel 5 pwm is \>1360\, \<\= 1490


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Stabilize | |
| +-------+-----------+ |
| | 1     | Acro      | |
| +-------+-----------+ |
| | 2     | AltHold   | |
| +-------+-----------+ |
| | 3     | Auto      | |
| +-------+-----------+ |
| | 4     | Guided    | |
| +-------+-----------+ |
| | 5     | Loiter    | |
| +-------+-----------+ |
| | 6     | RTL       | |
| +-------+-----------+ |
| | 7     | Circle    | |
| +-------+-----------+ |
| | 9     | Land      | |
| +-------+-----------+ |
| | 11    | Drift     | |
| +-------+-----------+ |
| | 13    | Sport     | |
| +-------+-----------+ |
| | 14    | Flip      | |
| +-------+-----------+ |
| | 15    | AutoTune  | |
| +-------+-----------+ |
| | 16    | PosHold   | |
| +-------+-----------+ |
| | 17    | Brake     | |
| +-------+-----------+ |
| | 18    | Throw     | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE4:

FLTMODE4: Flight Mode 4
~~~~~~~~~~~~~~~~~~~~~~~


Flight mode when Channel 5 pwm is \>1490\, \<\= 1620


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Stabilize | |
| +-------+-----------+ |
| | 1     | Acro      | |
| +-------+-----------+ |
| | 2     | AltHold   | |
| +-------+-----------+ |
| | 3     | Auto      | |
| +-------+-----------+ |
| | 4     | Guided    | |
| +-------+-----------+ |
| | 5     | Loiter    | |
| +-------+-----------+ |
| | 6     | RTL       | |
| +-------+-----------+ |
| | 7     | Circle    | |
| +-------+-----------+ |
| | 9     | Land      | |
| +-------+-----------+ |
| | 11    | Drift     | |
| +-------+-----------+ |
| | 13    | Sport     | |
| +-------+-----------+ |
| | 14    | Flip      | |
| +-------+-----------+ |
| | 15    | AutoTune  | |
| +-------+-----------+ |
| | 16    | PosHold   | |
| +-------+-----------+ |
| | 17    | Brake     | |
| +-------+-----------+ |
| | 18    | Throw     | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE5:

FLTMODE5: Flight Mode 5
~~~~~~~~~~~~~~~~~~~~~~~


Flight mode when Channel 5 pwm is \>1620\, \<\= 1749


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Stabilize | |
| +-------+-----------+ |
| | 1     | Acro      | |
| +-------+-----------+ |
| | 2     | AltHold   | |
| +-------+-----------+ |
| | 3     | Auto      | |
| +-------+-----------+ |
| | 4     | Guided    | |
| +-------+-----------+ |
| | 5     | Loiter    | |
| +-------+-----------+ |
| | 6     | RTL       | |
| +-------+-----------+ |
| | 7     | Circle    | |
| +-------+-----------+ |
| | 9     | Land      | |
| +-------+-----------+ |
| | 11    | Drift     | |
| +-------+-----------+ |
| | 13    | Sport     | |
| +-------+-----------+ |
| | 14    | Flip      | |
| +-------+-----------+ |
| | 15    | AutoTune  | |
| +-------+-----------+ |
| | 16    | PosHold   | |
| +-------+-----------+ |
| | 17    | Brake     | |
| +-------+-----------+ |
| | 18    | Throw     | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _FLTMODE6:

FLTMODE6: Flight Mode 6
~~~~~~~~~~~~~~~~~~~~~~~


Flight mode when Channel 5 pwm is \>\=1750


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Stabilize | |
| +-------+-----------+ |
| | 1     | Acro      | |
| +-------+-----------+ |
| | 2     | AltHold   | |
| +-------+-----------+ |
| | 3     | Auto      | |
| +-------+-----------+ |
| | 4     | Guided    | |
| +-------+-----------+ |
| | 5     | Loiter    | |
| +-------+-----------+ |
| | 6     | RTL       | |
| +-------+-----------+ |
| | 7     | Circle    | |
| +-------+-----------+ |
| | 9     | Land      | |
| +-------+-----------+ |
| | 11    | Drift     | |
| +-------+-----------+ |
| | 13    | Sport     | |
| +-------+-----------+ |
| | 14    | Flip      | |
| +-------+-----------+ |
| | 15    | AutoTune  | |
| +-------+-----------+ |
| | 16    | PosHold   | |
| +-------+-----------+ |
| | 17    | Brake     | |
| +-------+-----------+ |
| | 18    | Throw     | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _SIMPLE:

SIMPLE: Simple mode bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask which holds which flight modes use simple heading mode \(eg bit 0 \= 1 means Flight Mode 0 uses simple mode\)


.. _LOG_BITMASK:

LOG\_BITMASK: Log bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~


4 byte bitmap of log types to enable


+-------------------------+------------------------------+
| Bitmask                 | Values                       |
+=========================+==============================+
| +-----+---------------+ | +--------+-----------------+ |
| | Bit | Meaning       | | | Value  | Meaning         | |
| +=====+===============+ | +========+=================+ |
| | 0   | ATTITUDE_FAST | | | 830    | Default         | |
| +-----+---------------+ | +--------+-----------------+ |
| | 1   | ATTITUDE_MED  | | | 894    | Default+RCIN    | |
| +-----+---------------+ | +--------+-----------------+ |
| | 2   | GPS           | | | 958    | Default+IMU     | |
| +-----+---------------+ | +--------+-----------------+ |
| | 3   | PM            | | | 1854   | Default+Motors  | |
| +-----+---------------+ | +--------+-----------------+ |
| | 4   | CTUN          | | | -6146  | NearlyAll-AC315 | |
| +-----+---------------+ | +--------+-----------------+ |
| | 5   | NTUN          | | | 45054  | NearlyAll       | |
| +-----+---------------+ | +--------+-----------------+ |
| | 6   | RCIN          | | | 131071 | All+FastATT     | |
| +-----+---------------+ | +--------+-----------------+ |
| | 7   | IMU           | | | 262142 | All+MotBatt     | |
| +-----+---------------+ | +--------+-----------------+ |
| | 8   | CMD           | | | 393214 | All+FastIMU     | |
| +-----+---------------+ | +--------+-----------------+ |
| | 9   | CURRENT       | | | 397310 | All+FastIMU+PID | |
| +-----+---------------+ | +--------+-----------------+ |
| | 10  | RCOUT         | | | 655358 | All+FullIMU     | |
| +-----+---------------+ | +--------+-----------------+ |
| | 11  | OPTFLOW       | | | 0      | Disabled        | |
| +-----+---------------+ | +--------+-----------------+ |
| | 12  | PID           | |                              |
| +-----+---------------+ |                              |
| | 13  | COMPASS       | |                              |
| +-----+---------------+ |                              |
| | 14  | INAV          | |                              |
| +-----+---------------+ |                              |
| | 15  | CAMERA        | |                              |
| +-----+---------------+ |                              |
| | 17  | MOTBATT       | |                              |
| +-----+---------------+ |                              |
| | 18  | IMU_FAST      | |                              |
| +-----+---------------+ |                              |
| | 19  | IMU_RAW       | |                              |
| +-----+---------------+ |                              |
|                         |                              |
+-------------------------+------------------------------+




.. _ESC_CALIBRATION:

ESC\_CALIBRATION: ESC Calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls whether ArduCopter will enter ESC calibration on the next restart\.  Do not adjust this parameter manually\.


+---------------------------------------------------------------------+
| Values                                                              |
+=====================================================================+
| +-------+---------------------------------------------------------+ |
| | Value | Meaning                                                 | |
| +=======+=========================================================+ |
| | 0     | Normal Start-up                                         | |
| +-------+---------------------------------------------------------+ |
| | 1     | Start-up in ESC Calibration mode if throttle high       | |
| +-------+---------------------------------------------------------+ |
| | 2     | Start-up in ESC Calibration mode regardless of throttle | |
| +-------+---------------------------------------------------------+ |
| | 9     | Disabled                                                | |
| +-------+---------------------------------------------------------+ |
|                                                                     |
+---------------------------------------------------------------------+




.. _TUNE:

TUNE: Channel 6 Tuning
~~~~~~~~~~~~~~~~~~~~~~


Controls which parameters \(normally PID gains\) are being tuned with transmitter\'s channel 6 knob


+--------------------------------+
| Values                         |
+================================+
| +-------+--------------------+ |
| | Value | Meaning            | |
| +=======+====================+ |
| | 0     | None               | |
| +-------+--------------------+ |
| | 1     | Stab Roll/Pitch kP | |
| +-------+--------------------+ |
| | 4     | Rate Roll/Pitch kP | |
| +-------+--------------------+ |
| | 5     | Rate Roll/Pitch kI | |
| +-------+--------------------+ |
| | 21    | Rate Roll/Pitch kD | |
| +-------+--------------------+ |
| | 3     | Stab Yaw kP        | |
| +-------+--------------------+ |
| | 6     | Rate Yaw kP        | |
| +-------+--------------------+ |
| | 26    | Rate Yaw kD        | |
| +-------+--------------------+ |
| | 14    | Altitude Hold kP   | |
| +-------+--------------------+ |
| | 7     | Throttle Rate kP   | |
| +-------+--------------------+ |
| | 34    | Throttle Accel kP  | |
| +-------+--------------------+ |
| | 35    | Throttle Accel kI  | |
| +-------+--------------------+ |
| | 36    | Throttle Accel kD  | |
| +-------+--------------------+ |
| | 42    | Loiter Speed       | |
| +-------+--------------------+ |
| | 12    | Loiter Pos kP      | |
| +-------+--------------------+ |
| | 22    | Velocity XY kP     | |
| +-------+--------------------+ |
| | 28    | Velocity XY kI     | |
| +-------+--------------------+ |
| | 10    | WP Speed           | |
| +-------+--------------------+ |
| | 25    | Acro RollPitch kP  | |
| +-------+--------------------+ |
| | 40    | Acro Yaw kP        | |
| +-------+--------------------+ |
| | 13    | Heli Ext Gyro      | |
| +-------+--------------------+ |
| | 17    | OF Loiter kP       | |
| +-------+--------------------+ |
| | 18    | OF Loiter kI       | |
| +-------+--------------------+ |
| | 19    | OF Loiter kD       | |
| +-------+--------------------+ |
| | 38    | Declination        | |
| +-------+--------------------+ |
| | 39    | Circle Rate        | |
| +-------+--------------------+ |
| | 41    | RangeFinder Gain   | |
| +-------+--------------------+ |
| | 46    | Rate Pitch kP      | |
| +-------+--------------------+ |
| | 47    | Rate Pitch kI      | |
| +-------+--------------------+ |
| | 48    | Rate Pitch kD      | |
| +-------+--------------------+ |
| | 49    | Rate Roll kP       | |
| +-------+--------------------+ |
| | 50    | Rate Roll kI       | |
| +-------+--------------------+ |
| | 51    | Rate Roll kD       | |
| +-------+--------------------+ |
| | 52    | Rate Pitch FF      | |
| +-------+--------------------+ |
| | 53    | Rate Roll FF       | |
| +-------+--------------------+ |
| | 54    | Rate Yaw FF        | |
| +-------+--------------------+ |
|                                |
+--------------------------------+




.. _TUNE_LOW:

TUNE\_LOW: Tuning minimum
~~~~~~~~~~~~~~~~~~~~~~~~~


The minimum value that will be applied to the parameter currently being tuned with the transmitter\'s channel 6 knob


+-----------+
| Range     |
+===========+
| 0 - 32767 |
+-----------+




.. _TUNE_HIGH:

TUNE\_HIGH: Tuning maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~


The maximum value that will be applied to the parameter currently being tuned with the transmitter\'s channel 6 knob


+-----------+
| Range     |
+===========+
| 0 - 32767 |
+-----------+




.. _FRAME:

FRAME: Frame Orientation \(\+\, X or V\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls motor mixing for multicopters\.  Not used for Tri or Traditional Helicopters\.


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Plus      | |
| +-------+-----------+ |
| | 1     | X         | |
| +-------+-----------+ |
| | 2     | V         | |
| +-------+-----------+ |
| | 3     | H         | |
| +-------+-----------+ |
| | 4     | V-Tail    | |
| +-------+-----------+ |
| | 5     | A-Tail    | |
| +-------+-----------+ |
| | 10    | Y6B (New) | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _CH7_OPT:

CH7\_OPT: Channel 7 option
~~~~~~~~~~~~~~~~~~~~~~~~~~


Select which function if performed when CH7 is above 1800 pwm


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Do Nothing           | |
| +-------+----------------------+ |
| | 2     | Flip                 | |
| +-------+----------------------+ |
| | 3     | Simple Mode          | |
| +-------+----------------------+ |
| | 4     | RTL                  | |
| +-------+----------------------+ |
| | 5     | Save Trim            | |
| +-------+----------------------+ |
| | 7     | Save WP              | |
| +-------+----------------------+ |
| | 9     | Camera Trigger       | |
| +-------+----------------------+ |
| | 10    | RangeFinder          | |
| +-------+----------------------+ |
| | 11    | Fence                | |
| +-------+----------------------+ |
| | 12    | ResetToArmedYaw      | |
| +-------+----------------------+ |
| | 13    | Super Simple Mode    | |
| +-------+----------------------+ |
| | 14    | Acro Trainer         | |
| +-------+----------------------+ |
| | 16    | Auto                 | |
| +-------+----------------------+ |
| | 17    | AutoTune             | |
| +-------+----------------------+ |
| | 18    | Land                 | |
| +-------+----------------------+ |
| | 19    | EPM                  | |
| +-------+----------------------+ |
| | 21    | Parachute Enable     | |
| +-------+----------------------+ |
| | 22    | Parachute Release    | |
| +-------+----------------------+ |
| | 23    | Parachute 3pos       | |
| +-------+----------------------+ |
| | 24    | Auto Mission Reset   | |
| +-------+----------------------+ |
| | 25    | AttCon Feed Forward  | |
| +-------+----------------------+ |
| | 26    | AttCon Accel Limits  | |
| +-------+----------------------+ |
| | 27    | Retract Mount        | |
| +-------+----------------------+ |
| | 28    | Relay On/Off         | |
| +-------+----------------------+ |
| | 34    | Relay2 On/Off        | |
| +-------+----------------------+ |
| | 35    | Relay3 On/Off        | |
| +-------+----------------------+ |
| | 36    | Relay4 On/Off        | |
| +-------+----------------------+ |
| | 29    | Landing Gear         | |
| +-------+----------------------+ |
| | 30    | Lost Copter Sound    | |
| +-------+----------------------+ |
| | 31    | Motor Emergency Stop | |
| +-------+----------------------+ |
| | 32    | Motor Interlock      | |
| +-------+----------------------+ |
| | 33    | Brake                | |
| +-------+----------------------+ |
| | 37    | Throw                | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _CH8_OPT:

CH8\_OPT: Channel 8 option
~~~~~~~~~~~~~~~~~~~~~~~~~~


Select which function if performed when CH8 is above 1800 pwm


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Do Nothing           | |
| +-------+----------------------+ |
| | 2     | Flip                 | |
| +-------+----------------------+ |
| | 3     | Simple Mode          | |
| +-------+----------------------+ |
| | 4     | RTL                  | |
| +-------+----------------------+ |
| | 5     | Save Trim            | |
| +-------+----------------------+ |
| | 7     | Save WP              | |
| +-------+----------------------+ |
| | 9     | Camera Trigger       | |
| +-------+----------------------+ |
| | 10    | RangeFinder          | |
| +-------+----------------------+ |
| | 11    | Fence                | |
| +-------+----------------------+ |
| | 12    | ResetToArmedYaw      | |
| +-------+----------------------+ |
| | 13    | Super Simple Mode    | |
| +-------+----------------------+ |
| | 14    | Acro Trainer         | |
| +-------+----------------------+ |
| | 16    | Auto                 | |
| +-------+----------------------+ |
| | 17    | AutoTune             | |
| +-------+----------------------+ |
| | 18    | Land                 | |
| +-------+----------------------+ |
| | 19    | EPM                  | |
| +-------+----------------------+ |
| | 21    | Parachute Enable     | |
| +-------+----------------------+ |
| | 22    | Parachute Release    | |
| +-------+----------------------+ |
| | 23    | Parachute 3pos       | |
| +-------+----------------------+ |
| | 24    | Auto Mission Reset   | |
| +-------+----------------------+ |
| | 25    | AttCon Feed Forward  | |
| +-------+----------------------+ |
| | 26    | AttCon Accel Limits  | |
| +-------+----------------------+ |
| | 27    | Retract Mount        | |
| +-------+----------------------+ |
| | 28    | Relay On/Off         | |
| +-------+----------------------+ |
| | 34    | Relay2 On/Off        | |
| +-------+----------------------+ |
| | 35    | Relay3 On/Off        | |
| +-------+----------------------+ |
| | 36    | Relay4 On/Off        | |
| +-------+----------------------+ |
| | 29    | Landing Gear         | |
| +-------+----------------------+ |
| | 30    | Lost Copter Sound    | |
| +-------+----------------------+ |
| | 31    | Motor Emergency Stop | |
| +-------+----------------------+ |
| | 32    | Motor Interlock      | |
| +-------+----------------------+ |
| | 33    | Brake                | |
| +-------+----------------------+ |
| | 37    | Throw                | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _CH9_OPT:

CH9\_OPT: Channel 9 option
~~~~~~~~~~~~~~~~~~~~~~~~~~


Select which function if performed when CH9 is above 1800 pwm


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Do Nothing           | |
| +-------+----------------------+ |
| | 2     | Flip                 | |
| +-------+----------------------+ |
| | 3     | Simple Mode          | |
| +-------+----------------------+ |
| | 4     | RTL                  | |
| +-------+----------------------+ |
| | 5     | Save Trim            | |
| +-------+----------------------+ |
| | 7     | Save WP              | |
| +-------+----------------------+ |
| | 9     | Camera Trigger       | |
| +-------+----------------------+ |
| | 10    | RangeFinder          | |
| +-------+----------------------+ |
| | 11    | Fence                | |
| +-------+----------------------+ |
| | 12    | ResetToArmedYaw      | |
| +-------+----------------------+ |
| | 13    | Super Simple Mode    | |
| +-------+----------------------+ |
| | 14    | Acro Trainer         | |
| +-------+----------------------+ |
| | 16    | Auto                 | |
| +-------+----------------------+ |
| | 17    | AutoTune             | |
| +-------+----------------------+ |
| | 18    | Land                 | |
| +-------+----------------------+ |
| | 19    | EPM                  | |
| +-------+----------------------+ |
| | 21    | Parachute Enable     | |
| +-------+----------------------+ |
| | 22    | Parachute Release    | |
| +-------+----------------------+ |
| | 23    | Parachute 3pos       | |
| +-------+----------------------+ |
| | 24    | Auto Mission Reset   | |
| +-------+----------------------+ |
| | 25    | AttCon Feed Forward  | |
| +-------+----------------------+ |
| | 26    | AttCon Accel Limits  | |
| +-------+----------------------+ |
| | 27    | Retract Mount        | |
| +-------+----------------------+ |
| | 28    | Relay On/Off         | |
| +-------+----------------------+ |
| | 34    | Relay2 On/Off        | |
| +-------+----------------------+ |
| | 35    | Relay3 On/Off        | |
| +-------+----------------------+ |
| | 36    | Relay4 On/Off        | |
| +-------+----------------------+ |
| | 29    | Landing Gear         | |
| +-------+----------------------+ |
| | 30    | Lost Copter Sound    | |
| +-------+----------------------+ |
| | 31    | Motor Emergency Stop | |
| +-------+----------------------+ |
| | 32    | Motor Interlock      | |
| +-------+----------------------+ |
| | 33    | Brake                | |
| +-------+----------------------+ |
| | 37    | Throw                | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _CH10_OPT:

CH10\_OPT: Channel 10 option
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Select which function if performed when CH10 is above 1800 pwm


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Do Nothing           | |
| +-------+----------------------+ |
| | 2     | Flip                 | |
| +-------+----------------------+ |
| | 3     | Simple Mode          | |
| +-------+----------------------+ |
| | 4     | RTL                  | |
| +-------+----------------------+ |
| | 5     | Save Trim            | |
| +-------+----------------------+ |
| | 7     | Save WP              | |
| +-------+----------------------+ |
| | 9     | Camera Trigger       | |
| +-------+----------------------+ |
| | 10    | RangeFinder          | |
| +-------+----------------------+ |
| | 11    | Fence                | |
| +-------+----------------------+ |
| | 12    | ResetToArmedYaw      | |
| +-------+----------------------+ |
| | 13    | Super Simple Mode    | |
| +-------+----------------------+ |
| | 14    | Acro Trainer         | |
| +-------+----------------------+ |
| | 16    | Auto                 | |
| +-------+----------------------+ |
| | 17    | AutoTune             | |
| +-------+----------------------+ |
| | 18    | Land                 | |
| +-------+----------------------+ |
| | 19    | EPM                  | |
| +-------+----------------------+ |
| | 21    | Parachute Enable     | |
| +-------+----------------------+ |
| | 22    | Parachute Release    | |
| +-------+----------------------+ |
| | 23    | Parachute 3pos       | |
| +-------+----------------------+ |
| | 24    | Auto Mission Reset   | |
| +-------+----------------------+ |
| | 25    | AttCon Feed Forward  | |
| +-------+----------------------+ |
| | 26    | AttCon Accel Limits  | |
| +-------+----------------------+ |
| | 27    | Retract Mount        | |
| +-------+----------------------+ |
| | 28    | Relay On/Off         | |
| +-------+----------------------+ |
| | 34    | Relay2 On/Off        | |
| +-------+----------------------+ |
| | 35    | Relay3 On/Off        | |
| +-------+----------------------+ |
| | 36    | Relay4 On/Off        | |
| +-------+----------------------+ |
| | 29    | Landing Gear         | |
| +-------+----------------------+ |
| | 30    | Lost Copter Sound    | |
| +-------+----------------------+ |
| | 31    | Motor Emergency Stop | |
| +-------+----------------------+ |
| | 32    | Motor Interlock      | |
| +-------+----------------------+ |
| | 33    | Brake                | |
| +-------+----------------------+ |
| | 37    | Throw                | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _CH11_OPT:

CH11\_OPT: Channel 11 option
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Select which function if performed when CH11 is above 1800 pwm


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Do Nothing           | |
| +-------+----------------------+ |
| | 2     | Flip                 | |
| +-------+----------------------+ |
| | 3     | Simple Mode          | |
| +-------+----------------------+ |
| | 4     | RTL                  | |
| +-------+----------------------+ |
| | 5     | Save Trim            | |
| +-------+----------------------+ |
| | 7     | Save WP              | |
| +-------+----------------------+ |
| | 9     | Camera Trigger       | |
| +-------+----------------------+ |
| | 10    | RangeFinder          | |
| +-------+----------------------+ |
| | 11    | Fence                | |
| +-------+----------------------+ |
| | 12    | ResetToArmedYaw      | |
| +-------+----------------------+ |
| | 13    | Super Simple Mode    | |
| +-------+----------------------+ |
| | 14    | Acro Trainer         | |
| +-------+----------------------+ |
| | 16    | Auto                 | |
| +-------+----------------------+ |
| | 17    | AutoTune             | |
| +-------+----------------------+ |
| | 18    | Land                 | |
| +-------+----------------------+ |
| | 19    | EPM                  | |
| +-------+----------------------+ |
| | 21    | Parachute Enable     | |
| +-------+----------------------+ |
| | 22    | Parachute Release    | |
| +-------+----------------------+ |
| | 23    | Parachute 3pos       | |
| +-------+----------------------+ |
| | 24    | Auto Mission Reset   | |
| +-------+----------------------+ |
| | 25    | AttCon Feed Forward  | |
| +-------+----------------------+ |
| | 26    | AttCon Accel Limits  | |
| +-------+----------------------+ |
| | 27    | Retract Mount        | |
| +-------+----------------------+ |
| | 28    | Relay On/Off         | |
| +-------+----------------------+ |
| | 34    | Relay2 On/Off        | |
| +-------+----------------------+ |
| | 35    | Relay3 On/Off        | |
| +-------+----------------------+ |
| | 36    | Relay4 On/Off        | |
| +-------+----------------------+ |
| | 29    | Landing Gear         | |
| +-------+----------------------+ |
| | 30    | Lost Copter Sound    | |
| +-------+----------------------+ |
| | 31    | Motor Emergency Stop | |
| +-------+----------------------+ |
| | 32    | Motor Interlock      | |
| +-------+----------------------+ |
| | 33    | Brake                | |
| +-------+----------------------+ |
| | 37    | Throw                | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _CH12_OPT:

CH12\_OPT: Channel 12 option
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Select which function if performed when CH12 is above 1800 pwm


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Do Nothing           | |
| +-------+----------------------+ |
| | 2     | Flip                 | |
| +-------+----------------------+ |
| | 3     | Simple Mode          | |
| +-------+----------------------+ |
| | 4     | RTL                  | |
| +-------+----------------------+ |
| | 5     | Save Trim            | |
| +-------+----------------------+ |
| | 7     | Save WP              | |
| +-------+----------------------+ |
| | 9     | Camera Trigger       | |
| +-------+----------------------+ |
| | 10    | RangeFinder          | |
| +-------+----------------------+ |
| | 11    | Fence                | |
| +-------+----------------------+ |
| | 12    | ResetToArmedYaw      | |
| +-------+----------------------+ |
| | 13    | Super Simple Mode    | |
| +-------+----------------------+ |
| | 14    | Acro Trainer         | |
| +-------+----------------------+ |
| | 16    | Auto                 | |
| +-------+----------------------+ |
| | 17    | AutoTune             | |
| +-------+----------------------+ |
| | 18    | Land                 | |
| +-------+----------------------+ |
| | 19    | EPM                  | |
| +-------+----------------------+ |
| | 21    | Parachute Enable     | |
| +-------+----------------------+ |
| | 22    | Parachute Release    | |
| +-------+----------------------+ |
| | 23    | Parachute 3pos       | |
| +-------+----------------------+ |
| | 24    | Auto Mission Reset   | |
| +-------+----------------------+ |
| | 25    | AttCon Feed Forward  | |
| +-------+----------------------+ |
| | 26    | AttCon Accel Limits  | |
| +-------+----------------------+ |
| | 27    | Retract Mount        | |
| +-------+----------------------+ |
| | 28    | Relay On/Off         | |
| +-------+----------------------+ |
| | 34    | Relay2 On/Off        | |
| +-------+----------------------+ |
| | 35    | Relay3 On/Off        | |
| +-------+----------------------+ |
| | 36    | Relay4 On/Off        | |
| +-------+----------------------+ |
| | 29    | Landing Gear         | |
| +-------+----------------------+ |
| | 30    | Lost Copter Sound    | |
| +-------+----------------------+ |
| | 31    | Motor Emergency Stop | |
| +-------+----------------------+ |
| | 32    | Motor Interlock      | |
| +-------+----------------------+ |
| | 33    | Brake                | |
| +-------+----------------------+ |
| | 37    | Throw                | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _ARMING_CHECK:

ARMING\_CHECK: Arming check
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows enabling or disabling of pre\-arming checks of receiver\, accelerometer\, barometer\, compass and GPS


+----------------------------------+-------------------------------------+
| Bitmask                          | Values                              |
+==================================+=====================================+
| +-----+------------------------+ | +-------+-------------------------+ |
| | Bit | Meaning                | | | Value | Meaning                 | |
| +=====+========================+ | +=======+=========================+ |
| | 0   | All                    | | | 0     | Disabled                | |
| +-----+------------------------+ | +-------+-------------------------+ |
| | 1   | Baro                   | | | 1     | Enabled                 | |
| +-----+------------------------+ | +-------+-------------------------+ |
| | 2   | Compass                | | | -3    | Skip Baro               | |
| +-----+------------------------+ | +-------+-------------------------+ |
| | 3   | GPS                    | | | -5    | Skip Compass            | |
| +-----+------------------------+ | +-------+-------------------------+ |
| | 4   | INS                    | | | -9    | Skip GPS                | |
| +-----+------------------------+ | +-------+-------------------------+ |
| | 5   | Parameters+Rangefinder | | | -17   | Skip INS                | |
| +-----+------------------------+ | +-------+-------------------------+ |
| | 6   | RC                     | | | -33   | Skip Params/Rangefinder | |
| +-----+------------------------+ | +-------+-------------------------+ |
| | 7   | Voltage                | | | -65   | Skip RC                 | |
| +-----+------------------------+ | +-------+-------------------------+ |
|                                  | | 127   | Skip Voltage            | |
|                                  | +-------+-------------------------+ |
|                                  |                                     |
+----------------------------------+-------------------------------------+




.. _DISARM_DELAY:

DISARM\_DELAY: Disarm delay
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Delay before automatic disarm in seconds\. A value of zero disables auto disarm\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 - 127 | Seconds |
+---------+---------+




.. _ANGLE_MAX:

ANGLE\_MAX: Angle Max
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Maximum lean angle in all flight modes


+-------------+---------------+
| Range       | Units         |
+=============+===============+
| 1000 - 8000 | Centi-degrees |
+-------------+---------------+




.. _RC_FEEL_RP:

RC\_FEEL\_RP: RC Feel Roll\/Pitch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RC feel for roll\/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp


+---------+------------------------+-----------+
| Range   | Values                 | Increment |
+=========+========================+===========+
| 0 - 100 | +-------+------------+ | 10        |
|         | | Value | Meaning    | |           |
|         | +=======+============+ |           |
|         | | 0     | Very Soft  | |           |
|         | +-------+------------+ |           |
|         | | 25    | Soft       | |           |
|         | +-------+------------+ |           |
|         | | 50    | Medium     | |           |
|         | +-------+------------+ |           |
|         | | 75    | Crisp      | |           |
|         | +-------+------------+ |           |
|         | | 100   | Very Crisp | |           |
|         | +-------+------------+ |           |
|         |                        |           |
+---------+------------------------+-----------+




.. _PHLD_BRAKE_RATE:

PHLD\_BRAKE\_RATE: PosHold braking rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

PosHold flight mode\'s rotation rate during braking in deg\/sec


+--------+---------+
| Range  | Units   |
+========+=========+
| 4 - 12 | deg/sec |
+--------+---------+




.. _PHLD_BRAKE_ANGLE:

PHLD\_BRAKE\_ANGLE: PosHold braking angle max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

PosHold flight mode\'s max lean angle during braking in centi\-degrees


+-------------+---------------+
| Range       | Units         |
+=============+===============+
| 2000 - 4500 | Centi-degrees |
+-------------+---------------+




.. _LAND_REPOSITION:

LAND\_REPOSITION: Land repositioning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enables user input during LAND mode\, the landing phase of RTL\, and auto mode landings\.


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | No repositioning | |
| +-------+------------------+ |
| | 1     | Repositioning    | |
| +-------+------------------+ |
|                              |
+------------------------------+




.. _FS_EKF_ACTION:

FS\_EKF\_ACTION: EKF Failsafe Action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Controls the action that will be taken when an EKF failsafe is invoked


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 1     | Land                   | |
| +-------+------------------------+ |
| | 2     | AltHold                | |
| +-------+------------------------+ |
| | 3     | Land even in Stabilize | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _FS_EKF_THRESH:

FS\_EKF\_THRESH: EKF failsafe variance threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Allows setting the maximum acceptable compass and velocity variance


+--------------------------------------+
| Values                               |
+======================================+
| 0.6:Strict, 0.8:Default, 1.0:Relaxed |
+--------------------------------------+




.. _FS_CRASH_CHECK:

FS\_CRASH\_CHECK: Crash check enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This enables automatic crash checking\. When enabled the motors will disarm if a crash is detected\.


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




.. _RC_SPEED:

RC\_SPEED: ESC Update Speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This is the speed in Hertz that your ESCs will receive updates


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 50 - 490 | 1         | Hz    |
+----------+-----------+-------+




.. _ACRO_RP_P:

ACRO\_RP\_P: Acro Roll and Pitch P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode\.  Higher values mean faster rate of rotation\.


+--------+
| Range  |
+========+
| 1 - 10 |
+--------+




.. _ACRO_YAW_P:

ACRO\_YAW\_P: Acro Yaw P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Converts pilot yaw input into a desired rate of rotation in ACRO\, Stabilize and SPORT modes\.  Higher values mean faster rate of rotation\.


+--------+
| Range  |
+========+
| 1 - 10 |
+--------+




.. _ACRO_BAL_ROLL:

ACRO\_BAL\_ROLL: Acro Balance Roll
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

rate at which roll angle returns to level in acro mode\.  A higher value causes the vehicle to return to level faster\.


+-------+-----------+
| Range | Increment |
+=======+===========+
| 0 - 3 | 0.1       |
+-------+-----------+




.. _ACRO_BAL_PITCH:

ACRO\_BAL\_PITCH: Acro Balance Pitch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

rate at which pitch angle returns to level in acro mode\.  A higher value causes the vehicle to return to level faster\.


+-------+-----------+
| Range | Increment |
+=======+===========+
| 0 - 3 | 0.1       |
+-------+-----------+




.. _ACRO_TRAINER:

ACRO\_TRAINER: Acro Trainer
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Type of trainer used in acro mode


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | Disabled             | |
| +-------+----------------------+ |
| | 1     | Leveling             | |
| +-------+----------------------+ |
| | 2     | Leveling and Limited | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _ACRO_EXPO:

ACRO\_EXPO: Acro Expo
~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Acro roll\/pitch Expo to allow faster rotation when stick at edges


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Disabled  | |
| +-------+-----------+ |
| | 0.1   | Very Low  | |
| +-------+-----------+ |
| | 0.2   | Low       | |
| +-------+-----------+ |
| | 0.3   | Medium    | |
| +-------+-----------+ |
| | 0.4   | High      | |
| +-------+-----------+ |
| | 0.5   | Very High | |
| +-------+-----------+ |
|                       |
+-----------------------+




.. _VEL_XY_P:

VEL\_XY\_P: Velocity \(horizontal\) P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Velocity \(horizontal\) P gain\.  Converts the difference between desired velocity to a target acceleration


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.1 - 6.0 | 0.1       |
+-----------+-----------+




.. _VEL_XY_I:

VEL\_XY\_I: Velocity \(horizontal\) I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Velocity \(horizontal\) I gain\.  Corrects long\-term difference in desired velocity to a target acceleration


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.02 - 1.00 | 0.01      |
+-------------+-----------+




.. _VEL_XY_IMAX:

VEL\_XY\_IMAX: Velocity \(horizontal\) integrator maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Velocity \(horizontal\) integrator maximum\.  Constrains the target acceleration that the I gain will output


+----------+-----------+--------+
| Range    | Increment | Units  |
+==========+===========+========+
| 0 - 4500 | 10        | cm/s/s |
+----------+-----------+--------+




.. _VEL_Z_P:

VEL\_Z\_P: Velocity \(vertical\) P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Velocity \(vertical\) P gain\.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller


+---------------+
| Range         |
+===============+
| 1.000 - 8.000 |
+---------------+




.. _ACCEL_Z_P:

ACCEL\_Z\_P: Throttle acceleration controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Throttle acceleration controller P gain\.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output


+---------------+-----------+
| Range         | Increment |
+===============+===========+
| 0.500 - 1.500 | 0.05      |
+---------------+-----------+




.. _ACCEL_Z_I:

ACCEL\_Z\_I: Throttle acceleration controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Throttle acceleration controller I gain\.  Corrects long\-term difference in desired vertical acceleration and actual acceleration


+---------------+
| Range         |
+===============+
| 0.000 - 3.000 |
+---------------+




.. _ACCEL_Z_IMAX:

ACCEL\_Z\_IMAX: Throttle acceleration controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Throttle acceleration controller I gain maximum\.  Constrains the maximum pwm that the I term will generate


+----------+------------+
| Range    | Units      |
+==========+============+
| 0 - 1000 | Percent*10 |
+----------+------------+




.. _ACCEL_Z_D:

ACCEL\_Z\_D: Throttle acceleration controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Throttle acceleration controller D gain\.  Compensates for short\-term change in desired vertical acceleration vs actual acceleration


+---------------+
| Range         |
+===============+
| 0.000 - 0.400 |
+---------------+




.. _ACCEL_Z_FILT_HZ:

ACCEL\_Z\_FILT\_HZ: Throttle acceleration filter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Filter applied to acceleration to reduce noise\.  Lower values reduce noise but add delay\.


+-----------------+-------+
| Range           | Units |
+=================+=======+
| 1.000 - 100.000 | Hz    |
+-----------------+-------+




.. _POS_Z_P:

POS\_Z\_P: Position \(vertical\) controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position \(vertical\) controller P gain\.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller


+---------------+
| Range         |
+===============+
| 1.000 - 3.000 |
+---------------+




.. _POS_XY_P:

POS\_XY\_P: Position \(horizonal\) controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Loiter position controller P gain\.  Converts the distance \(in the latitude direction\) to the target location into a desired speed which is then passed to the loiter latitude rate controller


+---------------+
| Range         |
+===============+
| 0.500 - 2.000 |
+---------------+




.. _AUTOTUNE_AXES:

AUTOTUNE\_AXES: Autotune axis bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


1\-byte bitmap of axes to autotune


+-------------------+----------------------------+
| Bitmask           | Values                     |
+===================+============================+
| +-----+---------+ | +-------+----------------+ |
| | Bit | Meaning | | | Value | Meaning        | |
| +=====+=========+ | +=======+================+ |
| | 0   | Roll    | | | 7     | All            | |
| +-----+---------+ | +-------+----------------+ |
| | 1   | Pitch   | | | 1     | Roll Only      | |
| +-----+---------+ | +-------+----------------+ |
| | 2   | Yaw     | | | 2     | Pitch Only     | |
| +-----+---------+ | +-------+----------------+ |
|                   | | 4     | Yaw Only       | |
|                   | +-------+----------------+ |
|                   | | 3     | Roll and Pitch | |
|                   | +-------+----------------+ |
|                   | | 5     | Roll and Yaw   | |
|                   | +-------+----------------+ |
|                   | | 6     | Pitch and Yaw  | |
|                   | +-------+----------------+ |
|                   |                            |
+-------------------+----------------------------+




.. _AUTOTUNE_AGGR:

AUTOTUNE\_AGGR: Autotune aggressiveness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Autotune aggressiveness\. Defines the bounce back used to detect size of the D term\.


+-------------+
| Range       |
+=============+
| 0.05 - 0.10 |
+-------------+




.. _AUTOTUNE_MIN_D:

AUTOTUNE\_MIN\_D: AutoTune minimum D
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the minimum D gain


+---------------+
| Range         |
+===============+
| 0.001 - 0.006 |
+---------------+




.. _THROW_MOT_START:

THROW\_MOT\_START: Start motors before throwing is detected
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Used by THROW mode\. Controls whether motors will run at the speed set by THR\_MIN or will be stopped when armed and waiting for the throw\.


+---------------------+
| Values              |
+=====================+
| +-------+---------+ |
| | Value | Meaning | |
| +=======+=========+ |
| | 0     | Stopped | |
| +-------+---------+ |
| | 1     | Running | |
| +-------+---------+ |
|                     |
+---------------------+




.. _TERRAIN_FOLLOW:

TERRAIN\_FOLLOW: Terrain Following use control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This enables terrain following for RTL and LAND flight modes\. To use this option TERRAIN\_ENABLE must be 1 and the GCS must  support sending terrain data to the aircraft\.  In RTL the RTL\_ALT will be considered a height above the terrain\.  In LAND mode the vehicle will slow to LAND\_SPEED 10m above terrain \(instead of 10m above home\)\.  This parameter does not affect AUTO and Guided which use a per\-command flag to determine if the height is above\-home\, absolute or above\-terrain\.


+----------------------------------------------------------------+
| Values                                                         |
+================================================================+
| +-------+------------------------------+---------------------+ |
| | Value | Meaning                      |                       |
| +=======+==============================+=====================+ |
| | 0     | Do Not Use in RTL and Land 1 | Use in RTL and Land | |
| +-------+------------------------------+---------------------+ |
|                                                                |
+----------------------------------------------------------------+





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





.. _parameters_EPM_:

EPM\_ Parameters
----------------


.. _EPM_ENABLE:

EPM\_ENABLE: EPM Enable\/Disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


EPM enable\/disable


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




.. _EPM_GRAB:

EPM\_GRAB: EPM Grab PWM
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

PWM value sent to EPM to initiate grabbing the cargo


+-------------+
| Range       |
+=============+
| 1000 - 2000 |
+-------------+




.. _EPM_RELEASE:

EPM\_RELEASE: EPM Release PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

PWM value sent to EPM to release the cargo


+-------------+
| Range       |
+=============+
| 1000 - 2000 |
+-------------+




.. _EPM_NEUTRAL:

EPM\_NEUTRAL: EPM Neutral PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

PWM value sent to EPM when not grabbing or releasing


+-------------+
| Range       |
+=============+
| 1000 - 2000 |
+-------------+




.. _EPM_REGRAB:

EPM\_REGRAB: EPM Regrab interval
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Time in seconds that gripper will regrab the cargo to ensure grip has not weakend


+------------------------------+
| Values                       |
+==============================+
| +-------+------------------+ |
| | Value | Meaning          | |
| +=======+==================+ |
| | 0     | Never            | |
| +-------+------------------+ |
| | 15    | every 15 seconds | |
| +-------+------------------+ |
| | 30    | every 30 seconds | |
| +-------+------------------+ |
| | 60    | once per minute  | |
| +-------+------------------+ |
|                              |
+------------------------------+





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





.. _parameters_LGR_:

LGR\_ Parameters
----------------


.. _LGR_SERVO_RTRACT:

LGR\_SERVO\_RTRACT: Landing Gear Servo Retracted PWM Value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Servo PWM value when landing gear is retracted


+-------------+-----------+-------+
| Range       | Increment | Units |
+=============+===========+=======+
| 1000 - 2000 | 1         | pwm   |
+-------------+-----------+-------+




.. _LGR_SERVO_DEPLOY:

LGR\_SERVO\_DEPLOY: Landing Gear Servo Deployed PWM Value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Servo PWM value when landing gear is deployed


+-------------+-----------+-------+
| Range       | Increment | Units |
+=============+===========+=======+
| 1000 - 2000 | 1         | pwm   |
+-------------+-----------+-------+





.. _parameters_IM_:

IM\_ Parameters
---------------


.. _IM_STAB_COL_1:

IM\_STAB\_COL\_1: Stabilize Mode Collective Point 1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Helicopter\'s minimum collective pitch setting at zero throttle input in Stabilize mode


+---------+-----------+------------+
| Range   | Increment | Units      |
+=========+===========+============+
| 0 - 500 | 1         | Percent*10 |
+---------+-----------+------------+




.. _IM_STAB_COL_2:

IM\_STAB\_COL\_2: Stabilize Mode Collective Point 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Helicopter\'s collective pitch setting at mid\-low throttle input in Stabilize mode


+---------+-----------+------------+
| Range   | Increment | Units      |
+=========+===========+============+
| 0 - 500 | 1         | Percent*10 |
+---------+-----------+------------+




.. _IM_STAB_COL_3:

IM\_STAB\_COL\_3: Stabilize Mode Collective Point 3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Helicopter\'s collective pitch setting at mid\-high throttle input in Stabilize mode


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| 500 - 1000 | 1         | Percent*10 |
+------------+-----------+------------+




.. _IM_STAB_COL_4:

IM\_STAB\_COL\_4: Stabilize Mode Collective Point 4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Helicopter\'s maximum collective pitch setting at full throttle input in Stabilize mode


+------------+-----------+------------+
| Range      | Increment | Units      |
+============+===========+============+
| 500 - 1000 | 1         | Percent*10 |
+------------+-----------+------------+




.. _IM_ACRO_COL_EXP:

IM\_ACRO\_COL\_EXP: Acro Mode Collective Expo
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to soften collective pitch inputs near center point in Acro mode\.


+-----------------------+
| Values                |
+=======================+
| +-------+-----------+ |
| | Value | Meaning   | |
| +=======+===========+ |
| | 0     | Disabled  | |
| +-------+-----------+ |
| | 0.1   | Very Low  | |
| +-------+-----------+ |
| | 0.2   | Low       | |
| +-------+-----------+ |
| | 0.3   | Medium    | |
| +-------+-----------+ |
| | 0.4   | High      | |
| +-------+-----------+ |
| | 0.5   | Very High | |
| +-------+-----------+ |
|                       |
+-----------------------+





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





.. _parameters_WPNAV_:

WPNAV\_ Parameters
------------------


.. _WPNAV_SPEED:

WPNAV\_SPEED: Waypoint Horizontal Speed Target
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the speed in cm\/s which the aircraft will attempt to maintain horizontally during a WP mission


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 0 - 2000 | 50        | cm/s  |
+----------+-----------+-------+




.. _WPNAV_RADIUS:

WPNAV\_RADIUS: Waypoint Radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the distance from a waypoint\, that when crossed indicates the wp has been hit\.


+------------+-----------+-------+
| Range      | Increment | Units |
+============+===========+=======+
| 100 - 1000 | 1         | cm    |
+------------+-----------+-------+




.. _WPNAV_SPEED_UP:

WPNAV\_SPEED\_UP: Waypoint Climb Speed Target
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the speed in cm\/s which the aircraft will attempt to maintain while climbing during a WP mission


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 0 - 1000 | 50        | cm/s  |
+----------+-----------+-------+




.. _WPNAV_SPEED_DN:

WPNAV\_SPEED\_DN: Waypoint Descent Speed Target
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the speed in cm\/s which the aircraft will attempt to maintain while descending during a WP mission


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0 - 500 | 10        | cm/s  |
+---------+-----------+-------+




.. _WPNAV_LOIT_SPEED:

WPNAV\_LOIT\_SPEED: Loiter Horizontal Maximum Speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the maximum speed in cm\/s which the aircraft will travel horizontally while in loiter mode


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 0 - 2000 | 50        | cm/s  |
+----------+-----------+-------+




.. _WPNAV_ACCEL:

WPNAV\_ACCEL: Waypoint Acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the horizontal acceleration in cm\/s\/s used during missions


+----------+-----------+--------+
| Range    | Increment | Units  |
+==========+===========+========+
| 50 - 500 | 10        | cm/s/s |
+----------+-----------+--------+




.. _WPNAV_ACCEL_Z:

WPNAV\_ACCEL\_Z: Waypoint Vertical Acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the vertical acceleration in cm\/s\/s used during missions


+----------+-----------+--------+
| Range    | Increment | Units  |
+==========+===========+========+
| 50 - 500 | 10        | cm/s/s |
+----------+-----------+--------+




.. _WPNAV_LOIT_JERK:

WPNAV\_LOIT\_JERK: Loiter maximum jerk
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Loiter maximum jerk in cm\/s\/s\/s


+------------+-----------+----------+
| Range      | Increment | Units    |
+============+===========+==========+
| 500 - 5000 | 1         | cm/s/s/s |
+------------+-----------+----------+




.. _WPNAV_LOIT_MAXA:

WPNAV\_LOIT\_MAXA: Loiter maximum acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Loiter maximum acceleration in cm\/s\/s\.  Higher values cause the copter to accelerate and stop more quickly\.


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 100 - 981 | 1         | cm/s/s |
+-----------+-----------+--------+




.. _WPNAV_LOIT_MINA:

WPNAV\_LOIT\_MINA: Loiter minimum acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Loiter minimum acceleration in cm\/s\/s\. Higher values stop the copter more quickly when the stick is centered\, but cause a larger jerk when the copter stops\.


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 100 - 981 | 1         | cm/s/s |
+-----------+-----------+--------+





.. _parameters_CIRCLE_:

CIRCLE\_ Parameters
-------------------


.. _CIRCLE_RADIUS:

CIRCLE\_RADIUS: Circle Radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Defines the radius of the circle the vehicle will fly when in Circle flight mode


+-----------+-----------+-------+
| Range     | Increment | Units |
+===========+===========+=======+
| 0 - 10000 | 100       | cm    |
+-----------+-----------+-------+




.. _CIRCLE_RATE:

CIRCLE\_RATE: Circle rate
~~~~~~~~~~~~~~~~~~~~~~~~~


Circle mode\'s turn rate in deg\/sec\.  Positive to turn clockwise\, negative for counter clockwise


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| -90 - 90 | 1         | deg/s |
+----------+-----------+-------+





.. _parameters_ATC_:

ATC\_ Parameters
----------------


.. _ATC_RAT_RLL_P:

ATC\_RAT\_RLL\_P: Roll axis rate controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate controller P gain\.  Converts the difference between desired roll rate and actual roll rate into a motor speed output


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.08 - 0.30 | 0.005     |
+-------------+-----------+




.. _ATC_RAT_RLL_I:

ATC\_RAT\_RLL\_I: Roll axis rate controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate controller I gain\.  Corrects long\-term difference in desired roll rate vs actual roll rate


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.01 - 0.5 | 0.01      |
+------------+-----------+




.. _ATC_RAT_RLL_IMAX:

ATC\_RAT\_RLL\_IMAX: Roll axis rate controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate controller I gain maximum\.  Constrains the maximum motor output that the I gain will output


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 1 | 0.01      | Percent |
+-------+-----------+---------+




.. _ATC_RAT_RLL_D:

ATC\_RAT\_RLL\_D: Roll axis rate controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate controller D gain\.  Compensates for short\-term change in desired roll rate vs actual roll rate


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.0 - 0.02 | 0.001     |
+------------+-----------+




.. _ATC_RAT_RLL_FILT:

ATC\_RAT\_RLL\_FILT: Roll axis rate conroller input frequency in Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Roll axis rate conroller input frequency in Hz


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 1 - 100 | 1         | Hz    |
+---------+-----------+-------+




.. _ATC_RAT_PIT_P:

ATC\_RAT\_PIT\_P: Pitch axis rate controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate controller P gain\.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.08 - 0.30 | 0.005     |
+-------------+-----------+




.. _ATC_RAT_PIT_I:

ATC\_RAT\_PIT\_I: Pitch axis rate controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate controller I gain\.  Corrects long\-term difference in desired pitch rate vs actual pitch rate


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.01 - 0.5 | 0.01      |
+------------+-----------+




.. _ATC_RAT_PIT_IMAX:

ATC\_RAT\_PIT\_IMAX: Pitch axis rate controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate controller I gain maximum\.  Constrains the maximum motor output that the I gain will output


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 1 | 0.01      | Percent |
+-------+-----------+---------+




.. _ATC_RAT_PIT_D:

ATC\_RAT\_PIT\_D: Pitch axis rate controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate controller D gain\.  Compensates for short\-term change in desired pitch rate vs actual pitch rate


+------------+-----------+
| Range      | Increment |
+============+===========+
| 0.0 - 0.02 | 0.001     |
+------------+-----------+




.. _ATC_RAT_PIT_FILT:

ATC\_RAT\_PIT\_FILT: Pitch axis rate conroller input frequency in Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis rate conroller input frequency in Hz


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 1 - 100 | 1         | Hz    |
+---------+-----------+-------+




.. _ATC_RAT_YAW_P:

ATC\_RAT\_YAW\_P: Yaw axis rate controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate controller P gain\.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.10 - 0.50 | 0.005     |
+-------------+-----------+




.. _ATC_RAT_YAW_I:

ATC\_RAT\_YAW\_I: Yaw axis rate controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate controller I gain\.  Corrects long\-term difference in desired yaw rate vs actual yaw rate


+--------------+-----------+
| Range        | Increment |
+==============+===========+
| 0.010 - 0.05 | 0.01      |
+--------------+-----------+




.. _ATC_RAT_YAW_IMAX:

ATC\_RAT\_YAW\_IMAX: Yaw axis rate controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate controller I gain maximum\.  Constrains the maximum motor output that the I gain will output


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 1 | 0.01      | Percent |
+-------+-----------+---------+




.. _ATC_RAT_YAW_D:

ATC\_RAT\_YAW\_D: Yaw axis rate controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate controller D gain\.  Compensates for short\-term change in desired yaw rate vs actual yaw rate


+--------------+-----------+
| Range        | Increment |
+==============+===========+
| 0.000 - 0.02 | 0.001     |
+--------------+-----------+




.. _ATC_RAT_YAW_FILT:

ATC\_RAT\_YAW\_FILT: Yaw axis rate conroller input frequency in Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis rate conroller input frequency in Hz


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 1 - 100 | 1         | Hz    |
+---------+-----------+-------+





.. _parameters_POSCON_:

POSCON\_ Parameters
-------------------


.. _POSCON__ACC_XY_FILT:

POSCON\_\_ACC\_XY\_FILT: XY Acceleration filter cutoff frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Lower values will slow the response of the navigation controller and reduce twitchiness


+---------+-----------+-------+
| Range   | Increment | Units |
+=========+===========+=======+
| 0.5 - 5 | 0.1       | Hz    |
+---------+-----------+-------+





.. _parameters_SR0_:

SR0\_ Parameters
----------------


.. _SR0_RAW_SENS:

SR0\_RAW\_SENS: Raw sensor stream rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of RAW\_IMU\, SCALED\_IMU2\, SCALED\_PRESSURE\, and SENSOR\_OFFSETS to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_EXT_STAT:

SR0\_EXT\_STAT: Extended status stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of SYS\_STATUS\, MEMINFO\, MISSION\_CURRENT\, GPS\_RAW\_INT\, NAV\_CONTROLLER\_OUTPUT\, and LIMITS\_STATUS to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_RC_CHAN:

SR0\_RC\_CHAN: RC Channel stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of SERVO\_OUTPUT\_RAW and RC\_CHANNELS\_RAW to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_RAW_CTRL:

SR0\_RAW\_CTRL: Raw Control stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of RC\_CHANNELS\_SCALED \(HIL only\) to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_POSITION:

SR0\_POSITION: Position stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of GLOBAL\_POSITION\_INT to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_EXTRA1:

SR0\_EXTRA1: Extra data type 1 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of ATTITUDE and SIMSTATE \(SITL only\) to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_EXTRA2:

SR0\_EXTRA2: Extra data type 2 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of VFR\_HUD to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_EXTRA3:

SR0\_EXTRA3: Extra data type 3 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of AHRS\, HWSTATUS\, and SYSTEM\_TIME to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR0_PARAMS:

SR0\_PARAMS: Parameter stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of PARAM\_VALUE to ground station


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

Stream rate of RAW\_IMU\, SCALED\_IMU2\, SCALED\_PRESSURE\, and SENSOR\_OFFSETS to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_EXT_STAT:

SR1\_EXT\_STAT: Extended status stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of SYS\_STATUS\, MEMINFO\, MISSION\_CURRENT\, GPS\_RAW\_INT\, NAV\_CONTROLLER\_OUTPUT\, and LIMITS\_STATUS to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_RC_CHAN:

SR1\_RC\_CHAN: RC Channel stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of SERVO\_OUTPUT\_RAW and RC\_CHANNELS\_RAW to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_RAW_CTRL:

SR1\_RAW\_CTRL: Raw Control stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of RC\_CHANNELS\_SCALED \(HIL only\) to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_POSITION:

SR1\_POSITION: Position stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of GLOBAL\_POSITION\_INT to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_EXTRA1:

SR1\_EXTRA1: Extra data type 1 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of ATTITUDE and SIMSTATE \(SITL only\) to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_EXTRA2:

SR1\_EXTRA2: Extra data type 2 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of VFR\_HUD to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_EXTRA3:

SR1\_EXTRA3: Extra data type 3 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of AHRS\, HWSTATUS\, and SYSTEM\_TIME to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR1_PARAMS:

SR1\_PARAMS: Parameter stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of PARAM\_VALUE to ground station


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

Stream rate of RAW\_IMU\, SCALED\_IMU2\, SCALED\_PRESSURE\, and SENSOR\_OFFSETS to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_EXT_STAT:

SR2\_EXT\_STAT: Extended status stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of SYS\_STATUS\, MEMINFO\, MISSION\_CURRENT\, GPS\_RAW\_INT\, NAV\_CONTROLLER\_OUTPUT\, and LIMITS\_STATUS to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_RC_CHAN:

SR2\_RC\_CHAN: RC Channel stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of SERVO\_OUTPUT\_RAW and RC\_CHANNELS\_RAW to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_RAW_CTRL:

SR2\_RAW\_CTRL: Raw Control stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of RC\_CHANNELS\_SCALED \(HIL only\) to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_POSITION:

SR2\_POSITION: Position stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of GLOBAL\_POSITION\_INT to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_EXTRA1:

SR2\_EXTRA1: Extra data type 1 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of ATTITUDE and SIMSTATE \(SITL only\) to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_EXTRA2:

SR2\_EXTRA2: Extra data type 2 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of VFR\_HUD to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_EXTRA3:

SR2\_EXTRA3: Extra data type 3 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of AHRS\, HWSTATUS\, and SYSTEM\_TIME to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR2_PARAMS:

SR2\_PARAMS: Parameter stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of PARAM\_VALUE to ground station


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

Stream rate of RAW\_IMU\, SCALED\_IMU2\, SCALED\_PRESSURE\, and SENSOR\_OFFSETS to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_EXT_STAT:

SR3\_EXT\_STAT: Extended status stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of SYS\_STATUS\, MEMINFO\, MISSION\_CURRENT\, GPS\_RAW\_INT\, NAV\_CONTROLLER\_OUTPUT\, and LIMITS\_STATUS to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_RC_CHAN:

SR3\_RC\_CHAN: RC Channel stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of SERVO\_OUTPUT\_RAW and RC\_CHANNELS\_RAW to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_RAW_CTRL:

SR3\_RAW\_CTRL: Raw Control stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of RC\_CHANNELS\_SCALED \(HIL only\) to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_POSITION:

SR3\_POSITION: Position stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of GLOBAL\_POSITION\_INT to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_EXTRA1:

SR3\_EXTRA1: Extra data type 1 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of ATTITUDE and SIMSTATE \(SITL only\) to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_EXTRA2:

SR3\_EXTRA2: Extra data type 2 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of VFR\_HUD to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_EXTRA3:

SR3\_EXTRA3: Extra data type 3 stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of AHRS\, HWSTATUS\, and SYSTEM\_TIME to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+




.. _SR3_PARAMS:

SR3\_PARAMS: Parameter stream rate to ground station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Stream rate of PARAM\_VALUE to ground station


+--------+-----------+-------+
| Range  | Increment | Units |
+========+===========+=======+
| 0 - 10 | 1         | Hz    |
+--------+-----------+-------+





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





.. _parameters_SPRAY_:

SPRAY\_ Parameters
------------------


.. _SPRAY_ENABLE:

SPRAY\_ENABLE: Sprayer enable\/disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows you to enable \(1\) or disable \(0\) the sprayer


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




.. _SPRAY_PUMP_RATE:

SPRAY\_PUMP\_RATE: Pump speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Desired pump speed when travelling 1m\/s expressed as a percentage


+---------+------------+
| Range   | Units      |
+=========+============+
| 0 - 100 | percentage |
+---------+------------+




.. _SPRAY_SPINNER:

SPRAY\_SPINNER: Spinner rotation speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Spinner\'s rotation speed in PWM \(a higher rate will disperse the spray over a wider area horizontally\)


+-------------+-------+
| Range       | Units |
+=============+=======+
| 1000 - 2000 | ms    |
+-------------+-------+




.. _SPRAY_SPEED_MIN:

SPRAY\_SPEED\_MIN: Speed minimum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Speed minimum at which we will begin spraying


+----------+-------+
| Range    | Units |
+==========+=======+
| 0 - 1000 | cm/s  |
+----------+-------+




.. _SPRAY_PUMP_MIN:

SPRAY\_PUMP\_MIN: Pump speed minimum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum pump speed expressed as a percentage


+---------+------------+
| Range   | Units      |
+=========+============+
| 0 - 100 | percentage |
+---------+------------+





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





.. _parameters_FENCE_:

FENCE\_ Parameters
------------------


.. _FENCE_ENABLE:

FENCE\_ENABLE: Fence enable\/disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows you to enable \(1\) or disable \(0\) the fence functionality


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




.. _FENCE_TYPE:

FENCE\_TYPE: Fence Type
~~~~~~~~~~~~~~~~~~~~~~~


Enabled fence types held as bitmask


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | None                | |
| +-------+---------------------+ |
| | 1     | Altitude            | |
| +-------+---------------------+ |
| | 2     | Circle              | |
| +-------+---------------------+ |
| | 3     | Altitude and Circle | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _FENCE_ACTION:

FENCE\_ACTION: Fence Action
~~~~~~~~~~~~~~~~~~~~~~~~~~~


What action should be taken when fence is breached


+-------------------------+
| Values                  |
+=========================+
| +-------+-------------+ |
| | Value | Meaning     | |
| +=======+=============+ |
| | 0     | Report Only | |
| +-------+-------------+ |
| | 1     | RTL or Land | |
| +-------+-------------+ |
|                         |
+-------------------------+




.. _FENCE_ALT_MAX:

FENCE\_ALT\_MAX: Fence Maximum Altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum altitude allowed before geofence triggers


+-----------+-----------+--------+
| Range     | Increment | Units  |
+===========+===========+========+
| 10 - 1000 | 1         | Meters |
+-----------+-----------+--------+




.. _FENCE_RADIUS:

FENCE\_RADIUS: Circular Fence Radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Circle fence radius which when breached will cause an RTL


+------------+--------+
| Range      | Units  |
+============+========+
| 30 - 10000 | Meters |
+------------+--------+




.. _FENCE_MARGIN:

FENCE\_MARGIN: Fence Margin
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Distance that autopilot\'s should maintain from the fence to avoid a breach


+--------+--------+
| Range  | Units  |
+========+========+
| 1 - 10 | Meters |
+--------+--------+





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





.. _parameters_H_:

H\_ Parameters
--------------


.. _H_SV1_POS:

H\_SV1\_POS: Servo 1 Position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Angular location of swash servo \#1


+------------+-----------+---------+
| Range      | Increment | Units   |
+============+===========+=========+
| -180 - 180 | 1         | Degrees |
+------------+-----------+---------+




.. _H_SV2_POS:

H\_SV2\_POS: Servo 2 Position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Angular location of swash servo \#2


+------------+-----------+---------+
| Range      | Increment | Units   |
+============+===========+=========+
| -180 - 180 | 1         | Degrees |
+------------+-----------+---------+




.. _H_SV3_POS:

H\_SV3\_POS: Servo 3 Position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Angular location of swash servo \#3


+------------+-----------+---------+
| Range      | Increment | Units   |
+============+===========+=========+
| -180 - 180 | 1         | Degrees |
+------------+-----------+---------+




.. _H_TAIL_TYPE:

H\_TAIL\_TYPE: Tail Type
~~~~~~~~~~~~~~~~~~~~~~~~


Tail type selection\.  Simpler yaw controller used if external gyro is selected


+------------------------------------+
| Values                             |
+====================================+
| +-------+------------------------+ |
| | Value | Meaning                | |
| +=======+========================+ |
| | 0     | Servo only             | |
| +-------+------------------------+ |
| | 1     | Servo with ExtGyro     | |
| +-------+------------------------+ |
| | 2     | DirectDrive VarPitch   | |
| +-------+------------------------+ |
| | 3     | DirectDrive FixedPitch | |
| +-------+------------------------+ |
|                                    |
+------------------------------------+




.. _H_SWASH_TYPE:

H\_SWASH\_TYPE: Swash Type
~~~~~~~~~~~~~~~~~~~~~~~~~~


Swash Type Setting \- either 3\-servo CCPM or H1 Mechanical Mixing


+----------------------------------+
| Values                           |
+==================================+
| +-------+----------------------+ |
| | Value | Meaning              | |
| +=======+======================+ |
| | 0     | 3-Servo CCPM         | |
| +-------+----------------------+ |
| | 1     | H1 Mechanical Mixing | |
| +-------+----------------------+ |
|                                  |
+----------------------------------+




.. _H_GYR_GAIN:

H\_GYR\_GAIN: External Gyro Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


PWM sent to external gyro on ch7 when tail type is Servo w\/ ExtGyro


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 0 - 1000 | 1         | PWM   |
+----------+-----------+-------+




.. _H_PHANG:

H\_PHANG: Swashplate Phase Angle Compensation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Phase angle correction for rotor head\.  If pitching the swash forward induces a roll\, this can be correct the problem


+----------+-----------+---------+
| Range    | Increment | Units   |
+==========+===========+=========+
| -90 - 90 | 1         | Degrees |
+----------+-----------+---------+




.. _H_COLYAW:

H\_COLYAW: Collective\-Yaw Mixing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Feed\-forward compensation to automatically add rudder input when collective pitch is increased\. Can be positive or negative depending on mechanics\.


+----------+-----------+
| Range    | Increment |
+==========+===========+
| -10 - 10 | 0.1       |
+----------+-----------+




.. _H_FLYBAR_MODE:

H\_FLYBAR\_MODE: Flybar Mode Selector
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Flybar present or not\.  Affects attitude controller used during ACRO flight mode


+----------------------+
| Values               |
+======================+
| +-------+----------+ |
| | Value | Meaning  | |
| +=======+==========+ |
| | 0     | NoFlybar | |
| +-------+----------+ |
| | 1     | Flybar   | |
| +-------+----------+ |
|                      |
+----------------------+




.. _H_TAIL_SPEED:

H\_TAIL\_SPEED: Direct Drive VarPitch Tail ESC speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Direct Drive VarPitch Tail ESC speed\.  Only used when TailType is DirectDrive VarPitch


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 0 - 1000 | 1         | PWM   |
+----------+-----------+-------+




.. _H_GYR_GAIN_ACRO:

H\_GYR\_GAIN\_ACRO: External Gyro Gain for ACRO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


PWM sent to external gyro on ch7 when tail type is Servo w\/ ExtGyro\. A value of zero means to use H\_GYR\_GAIN


+----------+-----------+-------+
| Range    | Increment | Units |
+==========+===========+=======+
| 0 - 1000 | 1         | PWM   |
+----------+-----------+-------+





.. _parameters_MOT_:

MOT\_ Parameters
----------------


.. _MOT_SV_SPEED:

MOT\_SV\_SPEED: Servo speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Servo update speed in hz


+--------------+
| Values       |
+==============+
| 50, 125, 250 |
+--------------+





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





.. _parameters_PLND_:

PLND\_ Parameters
-----------------


.. _PLND_ENABLED:

PLND\_ENABLED: Precision Land enabled\/disabled and behaviour
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Precision Land enabled\/disabled and behaviour


+---------------------------------+
| Values                          |
+=================================+
| +-------+---------------------+ |
| | Value | Meaning             | |
| +=======+=====================+ |
| | 0     | Disabled            | |
| +-------+---------------------+ |
| | 1     | Enabled Always Land | |
| +-------+---------------------+ |
| | 2     | Enabled Strict      | |
| +-------+---------------------+ |
|                                 |
+---------------------------------+




.. _PLND_TYPE:

PLND\_TYPE: Precision Land Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Precision Land Type


+-------------------------------+
| Values                        |
+===============================+
| +-------+-------------------+ |
| | Value | Meaning           | |
| +=======+===================+ |
| | 0     | None              | |
| +-------+-------------------+ |
| | 1     | CompanionComputer | |
| +-------+-------------------+ |
| | 2     | IRLock            | |
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



