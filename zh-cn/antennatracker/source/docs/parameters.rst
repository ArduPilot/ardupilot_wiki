.. Dynamically generated list of documented parameters
.. This page was generated using Tools\/autotest\/param\_metadata\/param\_parse\.py

.. DO NOT EDIT


.. _parameters:

Complete Parameter List
=======================

This is a complete list of the parameters which can be set \(e\.g\. via the MAVLink protocol\) to control vehicle behaviour\. They are stored in persistent storage on the vehicle\.

This list is automatically generated from the latest ardupilot source code\, and so may contain parameters which are not yet in the stable released versions of the code\.




.. _parameters_AntennaTracker:

AntennaTracker Parameters
-------------------------


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

Allows setting an individual system id for this vehicle to distinguish it from others on the same network


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




.. _SYSID_TARGET:

SYSID\_TARGET: Target vehicle\'s MAVLink system ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The identifier of the vehicle being tracked\. This should be zero \(to auto detect\) or be the same as the SYSID\_THISMAV parameter of the vehicle being tracked\.


+---------+
| Range   |
+=========+
| 1 - 255 |
+---------+




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




.. _YAW_SLEW_TIME:

YAW\_SLEW\_TIME: Time for yaw to slew through its full range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly the tracker will change the servo output for yaw\. It is set as the number of seconds to do a full rotation\. You can use this parameter to slow the trackers movements\, which may help with some types of trackers\. A value of zero will allow for unlimited servo movement per update\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 20 | 0.1       | seconds |
+--------+-----------+---------+




.. _PITCH_SLEW_TIME:

PITCH\_SLEW\_TIME: Time for pitch to slew through its full range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly the tracker will change the servo output for pitch\. It is set as the number of seconds to do a full range of pitch movement\. You can use this parameter to slow the trackers movements\, which may help with some types of trackers\. A value of zero will allow for unlimited servo movement per update\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 20 | 0.1       | seconds |
+--------+-----------+---------+




.. _SCAN_SPEED:

SCAN\_SPEED: Speed at which to rotate in scan mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly the tracker will move the servos in SCAN mode


+---------+-----------+----------------+
| Range   | Increment | Units          |
+=========+===========+================+
| 0 - 100 | 1         | degrees/second |
+---------+-----------+----------------+




.. _MIN_REVERSE_TIME:

MIN\_REVERSE\_TIME: Minimum time to apply a yaw reversal
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When the tracker detects it has reached the limit of servo movement in yaw it will reverse and try moving to the other extreme of yaw\. This parameter controls the minimum time it should reverse for\. It is used to cope with trackers that have a significant lag in movement to ensure they do move all the way around\.


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 20 | 1         | seconds |
+--------+-----------+---------+




.. _START_LATITUDE:

START\_LATITUDE: Initial Latitude before GPS lock
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Combined with START\_LONGITUDE this parameter allows for an initial position of the tracker to be set\. This position will be used until the GPS gets lock\. It can also be used to run a stationary tracker with no GPS attached\.


+----------+-----------+---------+
| Range    | Increment | Units   |
+==========+===========+=========+
| -90 - 90 | 0.000001  | degrees |
+----------+-----------+---------+




.. _START_LONGITUDE:

START\_LONGITUDE: Initial Longitude before GPS lock
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Combined with START\_LATITUDE this parameter allows for an initial position of the tracker to be set\. This position will be used until the GPS gets lock\. It can also be used to run a stationary tracker with no GPS attached\.


+------------+-----------+---------+
| Range      | Increment | Units   |
+============+===========+=========+
| -180 - 180 | 0.000001  | degrees |
+------------+-----------+---------+




.. _STARTUP_DELAY:

STARTUP\_DELAY: Delay before first servo movement from trim
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This parameter can be used to force the servos to their trim value for a time on startup\. This can help with some servo types


+--------+-----------+---------+
| Range  | Increment | Units   |
+========+===========+=========+
| 0 - 10 | 0.1       | seconds |
+--------+-----------+---------+




.. _SERVO_TYPE:

SERVO\_TYPE: Type of servo system being used
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This allows selection of position servos or on\/off servos


+--------------------------------+
| Values                         |
+================================+
| +-------+--------------------+ |
| | Value | Meaning            | |
| +=======+====================+ |
| | 0     | Position           | |
| +-------+--------------------+ |
| | 1     | OnOff              | |
| +-------+--------------------+ |
| | 2     | ContinuousRotation | |
| +-------+--------------------+ |
|                                |
+--------------------------------+




.. _ONOFF_YAW_RATE:

ONOFF\_YAW\_RATE: Yaw rate for on\/off servos
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Rate of change of yaw in degrees\/second for on\/off servos


+--------+-----------+----------------+
| Range  | Increment | Units          |
+========+===========+================+
| 0 - 50 | 0.1       | degrees/second |
+--------+-----------+----------------+




.. _ONOFF_PITCH_RATE:

ONOFF\_PITCH\_RATE: Pitch rate for on\/off servos
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Rate of change of pitch in degrees\/second for on\/off servos


+--------+-----------+----------------+
| Range  | Increment | Units          |
+========+===========+================+
| 0 - 50 | 0.1       | degrees/second |
+--------+-----------+----------------+




.. _ONOFF_YAW_MINT:

ONOFF\_YAW\_MINT: Yaw minimum movement time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum amount of time in seconds to move in yaw


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 2 | 0.01      | seconds |
+-------+-----------+---------+




.. _ONOFF_PITCH_MINT:

ONOFF\_PITCH\_MINT: Pitch minimum movement time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimim amount of time in seconds to move in pitch


+-------+-----------+---------+
| Range | Increment | Units   |
+=======+===========+=========+
| 0 - 2 | 0.01      | seconds |
+-------+-----------+---------+




.. _YAW_TRIM:

YAW\_TRIM: Yaw trim
~~~~~~~~~~~~~~~~~~~


Amount of extra yaw to add when tracking\. This allows for small adjustments for an out of trim compass\.


+----------+-----------+---------+
| Range    | Increment | Units   |
+==========+===========+=========+
| -10 - 10 | 0.1       | degrees |
+----------+-----------+---------+




.. _PITCH_TRIM:

PITCH\_TRIM: Pitch trim
~~~~~~~~~~~~~~~~~~~~~~~


Amount of extra pitch to add when tracking\. This allows for small adjustments for a badly calibrated barometer\.


+----------+-----------+---------+
| Range    | Increment | Units   |
+==========+===========+=========+
| -10 - 10 | 0.1       | degrees |
+----------+-----------+---------+




.. _YAW_RANGE:

YAW\_RANGE: Yaw Angle Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis total range of motion in degrees


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 360 | 0.1       | degrees |
+---------+-----------+---------+




.. _PITCH_RANGE:

PITCH\_RANGE: Pitch Range
~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis total range of motion in degrees


+---------+-----------+---------+
| Range   | Increment | Units   |
+=========+===========+=========+
| 0 - 180 | 0.1       | degrees |
+---------+-----------+---------+




.. _DISTANCE_MIN:

DISTANCE\_MIN: Distance minimum to target
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Tracker will track targets at least this distance away


+---------+-----------+--------+
| Range   | Increment | Units  |
+=========+===========+========+
| 0 - 100 | 1         | meters |
+---------+-----------+--------+




.. _LOG_BITMASK:

LOG\_BITMASK: Log bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~


4 byte bitmap of log types to enable


+--------------------+----------------------+
| Bitmask            | Values               |
+====================+======================+
| +-----+----------+ | +-------+----------+ |
| | Bit | Meaning  | | | Value | Meaning  | |
| +=====+==========+ | +=======+==========+ |
| | 0   | ATTITUDE | | | 63    | Default  | |
| +-----+----------+ | +-------+----------+ |
| | 1   | GPS      | | | 0     | Disabled | |
| +-----+----------+ | +-------+----------+ |
| | 2   | RCIN     | |                      |
| +-----+----------+ |                      |
| | 3   | IMU      | |                      |
| +-----+----------+ |                      |
| | 4   | RCOUT    | |                      |
| +-----+----------+ |                      |
| | 5   | COMPASS  | |                      |
| +-----+----------+ |                      |
|                    |                      |
+--------------------+----------------------+




.. _PITCH2SRV_P:

PITCH2SRV\_P: Pitch axis controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis controller P gain\.  Converts the difference between desired pitch angle and actual pitch angle into a pitch servo pwm change


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 3.0 | 0.01      |
+-----------+-----------+




.. _PITCH2SRV_I:

PITCH2SRV\_I: Pitch axis controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis controller I gain\.  Corrects long\-term difference in desired pitch angle vs actual pitch angle


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 3.0 | 0.01      |
+-----------+-----------+




.. _PITCH2SRV_IMAX:

PITCH2SRV\_IMAX: Pitch axis controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis controller I gain maximum\.  Constrains the maximum pwm change that the I gain will output


+----------+-----------+------------+
| Range    | Increment | Units      |
+==========+===========+============+
| 0 - 4000 | 10        | Percent*10 |
+----------+-----------+------------+




.. _PITCH2SRV_D:

PITCH2SRV\_D: Pitch axis controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch axis controller D gain\.  Compensates for short\-term change in desired pitch angle vs actual pitch angle


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.001 - 0.1 | 0.001     |
+-------------+-----------+




.. _YAW2SRV_P:

YAW2SRV\_P: Yaw axis controller P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis controller P gain\.  Converts the difference between desired yaw angle \(heading\) and actual yaw angle into a yaw servo pwm change


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 3.0 | 0.01      |
+-----------+-----------+




.. _YAW2SRV_I:

YAW2SRV\_I: Yaw axis controller I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis controller I gain\.  Corrects long\-term difference in desired yaw angle \(heading\) vs actual yaw angle


+-----------+-----------+
| Range     | Increment |
+===========+===========+
| 0.0 - 3.0 | 0.01      |
+-----------+-----------+




.. _YAW2SRV_IMAX:

YAW2SRV\_IMAX: Yaw axis controller I gain maximum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis controller I gain maximum\.  Constrains the maximum pwm change that the I gain will output


+----------+-----------+------------+
| Range    | Increment | Units      |
+==========+===========+============+
| 0 - 4000 | 10        | Percent*10 |
+----------+-----------+------------+




.. _YAW2SRV_D:

YAW2SRV\_D: Yaw axis controller D gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Yaw axis controller D gain\.  Compensates for short\-term change in desired yaw angle \(heading\) vs actual yaw angle


+-------------+-----------+
| Range       | Increment |
+=============+===========+
| 0.001 - 0.1 | 0.001     |
+-------------+-----------+




.. _CMD_TOTAL:

CMD\_TOTAL: Number of loaded mission items
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Set to 1 if HOME location has been loaded by the ground station\. Do not change this manually\.


+---------+
| Range   |
+=========+
| 1 - 255 |
+---------+





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



