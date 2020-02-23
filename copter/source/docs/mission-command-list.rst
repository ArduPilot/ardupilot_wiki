.. _mission-command-list:

===========================
Copter Mission Command List
===========================

This page provides details of all the mission commands 
(i.e. `MAVLink commands <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT>`__) supported by Copter that can be run as part of a mission (i.e. :ref:`AUTO flight mode <auto-mode>`).

Each of the commands below is either a "Navigation" command or a "Do"
command.  Navigation commands (i.e. "TakeOff" and "Waypoint") affect the
location of the vehicle while "Do" commands (i.e. "Do-Set-Servo" and
"Do-Cam-Trigg-Dist") are for auxiliary functions and do not affect the
vehicle's position.  During a mission at most one "Navigation" command
and one "Do" command can be running at one time.  The "Do" commands will
be run in order as soon as the preceding navigation command before them
completes.  For more information on setting up missions please refer to the
:ref:`Planning a Mission with Waypoints and Events <common-planning-a-mission-with-waypoints-and-events>`
page.

Takeoff
~~~~~~~

.. image:: ../../../images/TakeOff.jpg
    :target: ../_images/TakeOff.jpg

The vehicle will climb straight up from it's current location to the
altitude specified (in meters). This should be the first command of
nearly all missions. If the mission is begun while the copter is already
flying, the vehicle will climb straight up to the specified altitude, if
the vehicle is already above the specified altitude the takeoff command
will be ignored and the mission will move onto the next command
immediately.

Waypoint
~~~~~~~~

.. image:: ../../../images/WayPoint.jpg
    :target: ../_images/WayPoint.jpg

The vehicle will fly a straight line to the location specified as a lat,
lon and altitude (in meters).

**Delay** - the amount of time to wait (in seconds) at the waypoint
before proceeding to the next command

**Hit Rad** - not supported - meant to hold the distance (in meters)
from the target point that will qualify the waypoint as complete. This
command is not supported. Instead the :ref:`WPNAV_RADIUS<WPNAV_RADIUS>` parameter should be
used (see "WP Radius" field in above screen shot or adjust through the
Standard Parameters List).  Even the :ref:`WPNAV_RADIUS<WPNAV_RADIUS>` is only used when the
waypoint has a Delay. With no delay specified the waypoint will be
considered complete when the virtual point that the vehicle is chasing
reaches the waypoint. This can be 10m (or more) ahead of the vehicle
meaning that the vehicle will turn towards the following waypoint long
before it actually reaches the current waypoint

**Yaw Ang** - not supported - meant to hold the resulting yaw angle in
degrees (0=north, 90 = east). Instead use a
:ref:`CONDITION_YAW <mission-command-list_condition-yaw>` command.

**Lat, Lon** - the latitude and longitude targets.  If left as zero it
will hold the current location.

**Alt** - the target altitude above home in meters.  If left as zero it
will hold the current altitude.

Spline Waypoint
~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_SplineWaypoint.jpg
    :target: ../_images/MissionList_SplineWaypoint.jpg

`Spline <https://en.wikipedia.org/wiki/Spline_%28mathematics%29>`__ waypoints take all the same arguments as regular waypoints (lat, lon, alt, delay) but when executed the vehicle will fly smooth paths (both vertically and horizontally) instead of straight lines.  Spline waypoints can be mixed with regular straight line waypoints as shown in the image above.

The Mission Planner shows the path the vehicle will take.  The image
above shows some characteristics of the spline

-  The 1 second delay at the end of Waypoint #1 causes the vehicle to
   stop so Spline command #2 begins by taking a sharp 90degree turn
-  The direction of travel as the vehicle passes through Spline Waypoint
   #3 is parallel to an imaginary line drawn between waypoints #2 and #4
-  Waypoint #5 is a straight line so the vehicle lines itself up to
   point towards waypoint #5 even before reaching waypoint #4.

Loiter_Time
~~~~~~~~~~~

.. image:: ../../../images/MissionList_LoiterTime.png
    :target: ../_images/MissionList_LoiterTime.png

The vehicle will fly to and then wait at the specified location for the
specified number of seconds.  This is the same as a Waypoint command.

**Times** - the number of seconds to remain at the location.

**Lat, Lon** - the latitude and longitude targets. If left as zero it
will hold at the current location.

**Alt** - the target altitude above home in meters. If left as zero it
will hold at the current altitude.

Loiter_Turns
~~~~~~~~~~~~

.. image:: ../../../images/MissionList_LoiterTurns.png
    :target: ../_images/MissionList_LoiterTurns.png

The mission equivalent of the :ref:`Circle flight mode <circle-mode>`.  The vehicle will fly a circle around the
specified lat, lon and altitude (in meters).  The radius of the circle
is controlled by the :ref:`CIRCLE_RADIUS<CIRCLE_RADIUS>` parameter (i.e. cannot be set as
part of the mission).

**Turn** - the number of full rotations to complete around the point.

**Dir 1=CW** - the direction to turn around the point. -1 = counter
clockwise, +1 = clockwise.

**Lat, Lon** - the latitude and longitude targets.  If left as zero it
will circle around the current location.

**Alt** - the target altitude above home in meters.  If left as zero it
will circle at the current altitude.

Loiter_Unlimited
~~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_LoiterUnlimited.png
    :target: ../_images/MissionList_LoiterUnlimited.png

The vehicle will fly to and then wait at the specified location
indefinitely.  The mission will not proceed past this command.

**Lat, Lon** - the latitude and longitude targets. If left as zero it
will wait at the current location.

**Alt** - the target altitude above home in meters. If left as zero it
will wait at the current altitude.

Return-To-Launch
~~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_RTL.png
    :target: ../_images/MissionList_RTL.png

Mission equivalent of the :ref:`RTL flight mode <rtl-mode>`.  As
described on the RTL flight mode wiki page, the vehicle will first climb
to the :ref:`RTL_ALT<RTL_ALT>` parameter's specified altitude (default is 15m) before
returning home.  The home location is where the vehicle was last armed.

This command takes no parameters and generally should be the last
command in the mission.

Land
~~~~

.. image:: ../../../images/MissionList_Land.png
    :target: ../_images/MissionList_Land.png

Vehicle will land at it's current location or at the lat/lon coordinates
provided.  This is the mission equivalent of the :ref:`LAND flight mode <land-mode>`.

**Lat, Lon** - the latitude and longitude targets. If left as zero it
will land at the current location.

Delay
~~~~~

.. image:: ../../../images/MissionList_NavDelay.png
    :target: ../_images/MissionList_NavDelay.png

Support for the Delay command was added in Copter-3.4.

Vehicle will remain at it's current location until the specified number of seconds has passed or the absolute time is reached.
The first column ("Seconds") holds the number of seconds to delay.  Set to -1 if this field should be ignored.
The "hour UTC", "minute UTC" and "second UTC" fields can be used to specify an absolute time (`UTC <https://en.wikipedia.org/wiki/Coordinated_Universal_Time>`__).  The example above will cause the vehicle to take-off at 1:30pm UTC.  Note that the hour and/or minute field can also be ignored by setting them to -1.

This is a video made during the development of this feature.  Note in the video CONDITION_DELAY command is used but in it's final version this DELAY command should be used.

..  youtube:: 9VK3yjIyCSo
    :width: 100%

Package Place
~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_packageplace.png
    :target: ../_images/MissionList_packageplace.png

Support for Package Place was added in Copter-3.5.

Vehicle flies to the specified location and descends until it senses (using motor output) that the package has reached the ground.  The gripper is then released to unload the package.
The first column ("Max Desc") hold the maximum descent (in meters) that the vehicle should descend.  If the package has still not reached the ground despite this descent, the package will not be released and the vehicle will advance to the next mission command.

..  youtube:: m4GK4ALqluc
    :width: 100%

.. _mission-command-list_do-set-roi:

Do-Set-ROI
~~~~~~~~~~

.. image:: ../../../images/MissionList_DoSetRoi.jpg
    :target: ../_images/MissionList_DoSetRoi.jpg

Points the nose of the vehicle or :ref:`camera gimbal <common-cameras-and-gimbals>` at the "region of interest".  In the example above the nose or camera would be pointed at the red marker.

The nose will continue to point at the red marker until the end of the mission OR "clear" the do-set-roi and cause the vehicle to return to its default behaviour (i.e. pointing at the next waypoint) by adding a second DO_SET_ROI command later in the mission with all zero for Lat, Lon and Alt.

**Lat, Lon** - the latitude and longitude targets.

**Alt** - the target altitude above home in meters.

..  youtube:: W8NCFHrEjfU
    :width: 100%
    
Condition-Delay
~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_ConditionDelay.png
    :target: ../_images/MissionList_ConditionDelay.png

Delays the start of the next "Do" command for the specified number of
seconds. In the example above, Command #4 (Do-Set-Roi) is delayed so
that it only starts 5 seconds after the vehicle has passed Waypoint #2.
Note: this command does not stop the vehicle. Also note that the delay
timer is only active until the next waypoint is reached, so if the "Do"
command hasn't been triggered by then it never will be.

**Time (sec)** - the time to delay the next "Do" command in seconds.

Condition-Distance
~~~~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_ConditionDistance.png
    :target: ../_images/MissionList_ConditionDistance.png

Delays the start of the next "Do" command until the vehicle is within
the specified number of meters of the next waypoint. In the example
above, Command #4 (Do-Set-Roi) is delayed so that it only starts once
the vehicle is within 50m of waypoint #5. Note: this command does not
cause the vehicle to stop or pause during the mission, it only affects
"Do" commands.

**Dist (m)** - how close the vehicle should be to the next waypoint (in
meters) before beginning the next "Do" command.


.. _mission-command-list_condition-yaw:

Condition-Yaw
~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_ConditionYaw.png
    :target: ../_images/MissionList_ConditionYaw.png

Point the nose of the vehicle to the specified heading by a specified
amount.

**Deg** - the desired heading in degrees (0 = North, 90 = East, etc)
when "rel/abs" field is "0" (meaning absolute)

OR the change in heading (in degrees) when "rel/abs" field is "1"
(meaning relative)

**Sec** - not supported.  meant to limit the rotation speed (in deg/sec)
as the vehicle turns to the desired heading

**Dir (1=CW, -1=CCW)** - Used only when relative heading is specified
(i.e. "rel/abs" = "1") denotes whether the autopilot should add
(CW) or subtract (CCW) the degrees (Deg) from the current heading to
calculate the target heading. The vehicle will always turn in direction
that most quickly gets it to the new target heading regardless if CW or
CCW are used.

**rel/abs** - allows specifying the heading (i.e "Deg" field) as an
absolute heading (if "0") or relative to the current heading (if "1")

Do-Jump
~~~~~~~

.. image:: ../../../images/MissionList_DoJump.png
    :target: ../_images/MissionList_DoJump.png

Jump to the specified mission command the specified number of times
before continuing with the mission.  In the example above the vehicle
would fly back-and-forth between waypoints #1 and #2 a total of 3 times
before flying on to waypoint #4.

**WP #** - the mission command to jump to.

**Repeat** - the maximum number of times we should perform the jump. -1
means always jump

Despite the name, Do-Jump is really a "Nav" command rather than a "Do"
command, so conditional commands like CONDITION_DELAY won't affect
Do-Jump, it will always perform the jump as soon as it hits that
command.

.. warning::

   No more that fifteen (15) Do-Jump commands can be used in a mission.  Subsequent Do-Jumps will be ignored.

Do-Change-Speed
~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoChangeSpeed.png
    :target: ../_images/MissionList_DoChangeSpeed.png

Change the target horizontal speed (in meters/sec) of the vehicle.

**speed m/s** - the desired maximum speed in meters/second.

.. warning::

   The speed parameter should be in the SECOND COLUMN, not the first column as shown in the screenshot above

Do-Set-Home
~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoSetHome.png
    :target: ../_images/MissionList_DoSetHome.png

This command should be avoided if possible and instead use :ref:`Rally Points <common-rally-points>` which can also control the position used for Return-To-Launch.

**Current(1)/Spec(0)** - if "1" use the current location as the new home, if "0" use the specified Lat and Lon coordinate (not labelled yet!)

**Lat, Lon** - the latitude and longitude targets.

Do-Set-Cam-Trigg-Dist
~~~~~~~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoSetCamTriggDist.png
    :target: ../_images/MissionList_DoSetCamTriggDist.png

Trigger the :ref:`camera shutter <common-camera-shutter-with-servo>`
at regular distance intervals. For example the command above will cause
the camera shutter to trigger after every 5m that the vehicle travels.

**Dist (m)** - distance interval in meters.

To stop the camera from triggering, add a DO_SET_CAM_TRIGG_DIST later in the mission with Dist = 0

.. _mission-command-list_do-set-relay:

Do-Set-Relay
~~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoSetRelay.png
    :target: ../_images/MissionList_DoSetRelay.png

Set a :ref:`Relay <common-relay>` pin's voltage high or low.  The
columns are mislabeled in the Mission Planner (issue here).

**First column is Relay number** : 0 = First Relay, 1 = Second Relay

**Second column is On/Off** : 0 = Off (i.e. 0V), 1 = On (i.e. 3.3V on
Pixhawk, 5V on APM)

Do-Repeat-Relay
~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoRepeatRelay.png
    :target: ../_images/MissionList_DoRepeatRelay.png

Toggle the :ref:`Relay <common-relay>` pin's voltage a specified number of
times. In the example above, assuming the relay was off to begin with,
it would be set high and then after 3 seconds it would be toggled low
again.

**First column is Relay number** : 0 = First Relay, 1 = Second Relay

**Repeat #** : how many times the relay should be toggled

**Delay(s)** - Number of seconds between each toggle.

Do-Set-Servo
~~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoSetServo.png
    :target: ../_images/MissionList_DoSetServo.png

Move a :ref:`servo <common-servo>` to a particular PWM value. In the
example above, the servo attached to output channel 8 would be moved to
PWM 1700 (servo's generally accept PWM values between 1000 and 2000).

**Ser No** : the output channel the servo is attached to

**PWM** : PWM value to output to the servo

Do-Repeat-Servo
~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoRepeatServo.png
    :target: ../_images/MissionList_DoRepeatServo.png

Repeatedly move a :ref:`servo <common-servo>` back and forth between
it's mid position and a specified PWM value. In the example above, the
servo attached to output channel 8 would be moved to PWM 1700, then
after 4 second, back to mid (default is 1500 which is held in the
:ref:`RC8_TRIM<RC8_TRIM>` parameter), after another 4 seconds it would be moved to 1700
again, then finally after 4 more seconds it would be moved back to mid.

**Ser No** : the output channel the servo is attached to

**PWM** : PWM value to output to the servo

**Repeat #** : Number of times to move the servo to the specified PWM
value

**Delay (s)** : the delay in seconds between each servo movement

Do-Digicam-Control
~~~~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoDigicamControl.png
    :target: ../_images/MissionList_DoDigicamControl.png

Trigger the :ref:`camera shutter <common-camera-shutter-with-servo>`
once. This command takes no additional arguments.

Do-Mount-Control
~~~~~~~~~~~~~~~~

.. image:: ../../../images/MissionList_DoMountControl.png
    :target: ../_images/MissionList_DoMountControl.png

Controls the roll, pitch and yaw angle of the :ref:`camera gimbal <common-cameras-and-gimbals>`. This can be used to point the camera in specific directions at various times in the mission.

Do-Gripper
~~~~~~~~~~

.. image:: ../../../images/MissionList_DoGripper.png
    :target: ../_images/MissionList_DoGripper.png

This command allows opening and closing a :ref:`servo gripper <common-gripper-servo>` or :ref:`EPM gripper <common-cameras-and-gimbals>`.
Set the "drop/grab" column to 0 to close the gripper, 1 to open the gripper.  The first column, "Gripper No" is ignored because we currently only support a single gripper per vehicle.