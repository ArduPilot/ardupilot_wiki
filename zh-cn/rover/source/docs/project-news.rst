.. _project-news:

==================
Rover Project News
==================

This topic contains milestones and announcements related to the Rover
project.

**June 19, 2015: Rover 2.50 released:**
=======================================

The ArduPilot development team has released version 2.50 of APM:Rover.
This release is mostly a back-end improvement to ArduPilot but a few new
features and bug fixes are included.

**Re-do Accelerometer Calibration**

Due to a change in the maximum accelerometer range on the Pixhawk all
users must re-do their accelerometer calibration for this release.

**Only 3D accel calibration**

The old "1D" accelerometer calibration method has now been removed, so
you must use the 3D accelerometer calibration method. The old method was
removed because a significant number of users had poor experiences.

Changes in this release are:

-  CLI_ENABLED parameter added so the CLI can now be accessed in Rover
-  PID logging for the steering controller. It its now possible to graph
   what the P, I and D are doing as your driving the rover around to
   enable much better tuning of the vehicle.
-  Transition from .pde file to .cpp files for improved development.
-  GIT Submodules created for PX4Firmware, PX4Nuttx and uavcan git
   repositories for improved development.
-  Followme mode now works for Rover
-  GUIDED mode significantly improved. If you have a GCS which is in
   Followme mode if the user then changes mode with the RC transmitter
   to HOLD or anything else then the Rover will STOP listening to the
-  Followme updated guided mode waypoints.
-  When going into GUIDED mode the rover went into RTL - this is fixed.
-  Added EKF_STATUS_REPORT MAVLink message
-  64-bit timestamps in dataflash logs
-  Numerous EKF improvements
-  Added support for 4th Mavlink channel
-  Added support for raw IMU logging
-  Updated Piksi RTK GPS driver
-  Improved support for GPS data injection (for Piksi RTK GPS)
-  The SITL software in the loop simulation system has been completely
   rewritten for this release. A major change is to make it possible to
   run SITL on native windows without needing a Linux virtual machine.
   (thanks Tridge)

**March 4, 2015: Rover 2.49 released:**
=======================================

Hi everyone. The ardupilot team is delighted to announce the release of
version 2.49 of Rover.

In case anyone needs to manually download the beta release its here:

http://firmware.ardupilot.org/Rover/stable/

This release is a bug fix release with two important bugs found by Marco
Walther - Thanks Marco!

The bug fixes in this release are:

-  Fixed a sonar problem where objects to the left wouldn't be
   identified - thanks Marco Walther!
-  Fixed the ordering of the AP_Notify call so the main indicator light
   would be correct on startup - thanks Marco Walther!

**February 20, 2015: Rover 2.48 released:**
===========================================

Hi everyone. The ardupilot team is delighted to announce the release of
version 2.48 of Rover.

In case anyone needs to manually download the beta release its here:

http://firmware.ardupilot.org/Rover/stable/

The ardupilot development team has released version 2.48 of Rover. This
release is a bug fix release with some important bugs found by the users
of ardupilot.


The changes in this release are:


-  fixed a bug that could cause short term loss of RC control with some
   receiver systems and configurations
-  allowed for shorter sync pulse widths for PPM-SUM receivers on APM1
   and APM2
-  fix an issue where battery reporting could be intermittent (thanks
   Georgii Staroselskii!)
-  fixed a mission handling bug that could cause a crash if jump
   commands form an infinite loop (thanks to Dellarb for reporting this
   bug)
-  improved support for in-kernel SPI handling on Linux (thanks to John
   Williams)
-  support UAVCAN based ESCs and GPS modules on Pixhawk (thanks to
   Pavel, Holger and and PX4 dev team)
-  Multiple updates for the NavIO+ cape on RaspberryPi (thanks to Emlid)
-  Lots of EKF changes
-  added support for MAVLink packet routing
-  added detection and recovery from faulty gyro and accel sensors
-  added support for BBBMini Linux port
-  increased number of AVR input channels from 8 to 11
-  auto-set system clock based on GPS in Linux ports
-  added SBUS FrSky telemetry support (thanks to Mathias)
-  Added AK8963 MAG support (thanks Staroselskii Georgii)
-  Added support for second battery
-  Auto formatting of SDCard if it cannot be accessed on startup
-  A number of significant performance improvements for the PX4 platform

The most important bug fix is the one for short term loss of RC control.
This is a very long standing bug which didn't have a noticible impact
for most people, but could cause loss of RC control for around 1 or 2
seconds for some people in certain circumstances.

The bug was in the the AP_HAL RCInput API. Each HAL backend has a flag
that says whether there is a new RC input frame available. That flag was
cleared by the read() method (typically hal.rcin->read()). Callers would
check for new input by checking the boolean hal.rcin->new_input()
function.

The problem was that read() was called from multiple places. Normally
this is fine as reads from other than the main radio input loop happen
before the other reads, but if the timing of the new radio frame exactly
matched the loop frequency then a read from another place could clear
the new_input flag and we would not see the new RC input frame. If that
happened enough times we would go into a short term RC failsafe and
ignore RC inputs, even in manual mode.

The fix was very simple - it is the new_input() function itself that
should clear the flag, not read().

Many thanks to MarkM for helping us track down this bug by providing us
with sufficient detail on how to reproduce it. In Marks case his
OpenLRSng configuration happened to produce exactly the worst case
timing needed to reproduce the issue. Once Tridge copied his OpenLRS

settings to his TX/RX he was able to reproduce the problem and it was
easy to find and fix.

A number of users have reported occasional glitches in manual control
where servos pause for short periods in past releases. It is likely that
some of those issues were caused by this bug. The dev team would like to
apologise for taking so long to track down this bug!

The other main change was also related to RC input. Some receivers use a
PPM-SUM sync pulse width shorter than what the APM1/APM2 code was setup
to handle. The OpenLRSng default sync pulse width is 3000 microseconds,
but the APM1/APM2 code was written for a minimum sync pulse width of
4000 microseconds. For this release we have changed the APM1/APM2 driver
to accept a sync pulse width down to 2700 microseconds.

**Auto format of SD Card**

From time to time the SD cards in the PX4 autopilots get corrupted. This
isn't a surprise considering what we do to them. Your all familiar with
the windows "please unmount or eject your SDCard before removing"
process. Well we don't do that. In fact normal operation is to just pull
the power on the SDCard - whilst its being written too!! Not to mention
the horrible vibration rich environment the SDCard exists in. If the
autopilot is setup in the internal innards of your plane/copter/rover
this can be a nightmare to get to. To resolve that problem Tridge has
added code at startup so when ArduPilot tries to mount to SDCard to
access it - if that fails it will then try to format the SDCard and if
successful mount the card and proceed. If the format fails then you will
get the usual SOS Audio that makes most of us want to find the buzzer
and rip its heart out.

I mention this in case anyone has precious logs saved on the SDCard or
they are using the SDCard out of their phone with their wedding photo's
on it. Probably best not to do that and assume any data on the SDCard
can be deleted.

We are also looking to add a parameter to control whether the card is
auto formatted on startup or not but it isn't in there yet.

November 15, 2014: Rover 2.47 released:
=======================================

This is a minor bug fix release. The most important change in this
release is the fixing of the skid steering support but there have been a
number of fixes in other areas as well.

Full changes list for this release:

-  add support for controlling safety switch on Pixhawk from ground
   station
-  prevent reports of failed AHRS during initialisation
-  fixed skid steering that was broken in the last release
-  report gyro unhealthy if gyro calibration failed
-  fixed dual sonar support in CLI sonar test
-  fixed Nuttx crash on Pixhawk with bad I2C cables
-  added GPS_SBAS_MODE parameter - turns on/off satellite based
   augemtation system for GPS
-  added GPS_MIN_ELEV parameter - specifiy the elevation mask for GPS
   satellites
-  added RELAY_DEFAULT parameter to control default of relay on startup
-  fixed bug in FRAM storage on Pixhawk that could cause parameters
   changes not to be saved
-  better handling of compass errors in the EKF (Extended Kalman Filter)
-  improved support for linux based autopilots
-  added support for PulsedLight LIDAR as a range finder

Many thanks to everyone who contributed to this release, especially

Tom Coyle and Linus Penzlien for their excellent testing and feedback.

August 26, 2014: Rover 2.46 released:
=====================================

This release is based on a lot of development and testing that happened
prior to the \ `AVC competition <https://avc.sparkfun.com/2014>`__ where
APM based vehicles performed very well. Details
`here <http://diydrones.com/profiles/blogs/apm-rover-2-46-released>`__.

Full changes list for this release:

-  added support for higher baudrates on telemetry ports, to make it
   easier to use high rate telemetry to companion boards. Rates of up to
   1.5MBit are now supported to companion boards.
-  new Rangefinder code with support for a wider range of rangefinder
   types including a range of Lidars (thanks to Allyson Kreft)
-  added logging of power status on Pixhawk
-  added PIVOT_TURN_ANGLE parameter for pivot based turns on skid
   steering rovers
-  lots of improvements to the EKF support for Rover, thanks to Paul
   Riseborough and testing from Tom Coyle. Using the EKF can greatly
   improve navigation accuracy for fast rovers. Enable with
   AHRS_EKF_USE=1.
-  improved support for dual GPS on Pixhawk. Using a 2nd GPS can greatly
   improve performance when in an area with an obstructed view of the
   sky
-  support for up to 14 RC channels on Pihxawk
-  added BRAKING_PERCENT and BRAKING_SPEEDERR parameters for better
   breaking support when cornering
-  added support for FrSky telemetry via SERIAL2_PROTOCOL parameter
   (thanks to Matthias Badaire)
-  added support for Linux based autopilots, initially with the PXF
   BeagleBoneBlack cape and the Erle robotics board. Support for more
   boards is expected in future releases. Thanks to Victor, Sid and Anuj
   for their great work on the Linux port.
-  added StorageManager library, which expands available FRAM storage on
   Pixhawk to 16 kByte. This allows for 724 waypoints on Pixhawk.
-  improved reporting of compass (magnetometer) and barometer errors to
   the GCS
-  fixed a bug in automatic flow control detection for serial ports in
   Pixhawk
-  fixed use of FMU servo pins as digital inputs on Pixhawk
-  imported latest updates for VRBrain boards (thanks to Emile
   Castelnuovo and Luca Micheletti)
-  updates to the Piksi GPS support (thanks to Niels Joubert)
-  improved gyro estimate in DCM (thanks to Jon Challinger)
-  improved position projection in DCM in wind (thanks to Przemek
   Lekston)
-  several updates to AP_NavEKF for more robust handling of errors
   (thanks to Paul Riseborough)
-  lots of small code cleanups thanks to Daniel Frenzel
-  initial support for NavIO board from Mikhail Avkhimenia
-  fixed logging of RCOU for up to 12 channels (thanks to Emile
   Castelnuovo)
-  code cleanups from Silvia Nunezrivero
-  improved parameter download speed on radio links with no flow control

January 26, 2014 Rover 2.45 released.
=====================================

Details on
Forum \ `Here <http://ardupilot.com/forum/viewtopic.php?f=89&t=5834>`__
and the associated Blog
`Here <http://diydrones.com/profiles/blogs/major-new-update-for-apm-rover-code>`__.

.. image:: http://api.ning.com/files/eiC0hTlxr2LUqAYpNm3tBKRHFRG0Y-jvLGbt8BeFyd87MztgXoUhaHoVpIST0FMCKREnd-MJC0Sd2zX5AC1obbWlcP-sMGpU/tomroverauto11.png
    :target:  http://api.ning.com/files/eiC0hTlxr2LUqAYpNm3tBKRHFRG0Y-jvLGbt8BeFyd87MztgXoUhaHoVpIST0FMCKREnd-MJC0Sd2zX5AC1obbWlcP-sMGpU/tomroverauto11.png

Rover Developer Test Track and Typical Mission Test Course

Good news from Andrew Tridgell and the rest of the Rover (Rover)
Development Team: they've pushed out a major new release (2.45). Major
changes include: numerous updates for PX4 and Pixhawk, dual sensor
support on the Pixhawk and reduced GPS latency for all three navigation
controllers.

The following is from Tridge's \ `release announcement <http://ardupilot.com/forum/viewtopic.php?f=89&t=5834>`__.
Please post any questions or comments there.

Pixhawk updates
---------------

The biggest set of changes are for the PX4 and Pixhawk boards. The
Pixhawk production boards started shipping a few weeks ago and include
some nice hardware features that didn't make it into the 2.44 release.
The most important of these is dual sensor support. The Pixhawk has two
gyros and two accelerometers, and (if you have an external GPS/compass
combo) dual compass as well. In the 2.44 release only one of each of
these sensors could be used. With the 2.45 release the health of each
sensor is monitored and if one sensor fails the second sensor can take
over. In addition to failover support the code logs both sets of sensor
values both to MAVLink and to the logs on the microSD card, which is
very useful for diagnostics. We are also working on dual GPS support
which we hope to get into the next release.

GPS navigation improvements
---------------------------

A very simple change to our GPS configuration made a big difference for
this release. In previous rover releases we setup the uBlox as
ENGINE_AUTOMOTIVE as we thought that would be the best match for a
rover. What we discovered recently is that using the AUTOMOTIVE mode led
to much more time delay (lag) in the GPS velocity data, which led to
poor handling of sharp turns. We have now switched to
ENGINE_AIRBORNE_4G and found that we get much better navigation at
higher speed with sharp turns.

Parameter storage
-----------------

Another major change for this release is the way parameters are stored
on PX4 and Pixhawk. In previous releases we stored parameters on a file
on the microSD card. That usually worked fine, but recently there have
been a few too many issues with FAT filesystem corruption of microSD
cards, especially when powering off while writing to the SD card. For
this release we have moved all parameters to the EEPROM on the PX4 and
the FRAM chip on the Pixhawk. This makes parameter storage independent
of the microSD card, avoiding parameters becoming corrupt due to microSD
card problems. Parameters from a microSD card will be automatically
copied to EEPROM/FRAM when you upgrade to Rover 2.45.

Improved relay code
-------------------

The relay and servo set code has had a major overhaul, with up to 4
relays now supported for MAVLink control and much better support for the
DO_SET_SERVO, DO_SET_RELAY, DO_REPEAT_SERVO and DO_REPEAT_RELAY
MAVLink commands. Along with these changes is a new parameter
BRD_PWM_COUNT which allows you to specify how many auxillary PWM
outputs to enable, with the remaining outputs being available as digital
relays. This allows you to re-assign some of the aux servo outputs on
Pixhawk for use as relays, by setting the RELAY_PIN, RELAY_PIN2,
RELAY_PIN3 and RELAY_PIN4 parameters. The pin numbers for these pins
start at 50 for the first aux servo pin, and go to 55 on Pixhawk.

Improved logging
----------------

There have been logs of logging improvements in this release. Apart from
the dual sensor logging, we can now transfer on-board log files to the
ground station over MAVLink, which makes it much easier to get detailed
logs without having to pull a microSD card out, or boot to the CLI.
We're hoping to remove the need for the CLI completely in a future
release, doing everything over MAVLink.

The new logging code also includes the git version number of APM and (if
needed) the PX4Firmware and PX4NuttX repositories used to build the
firmware in the logs. This makes it easier to track down any issues to
the exact code used.

More telemetry ports
--------------------

On Pixhawk and PX4 you can now have a 3rd telemetry port, allowing USB
and two serial telemetry ports to be active at the same time, which is
very useful when you have an onboard computer.

Regards,

TCIII Rover2 Developer

Rover 2.44 Released
===================

-  December 29 2013 Rover 2.44 released.
   Details \ `here <http://ardupilot.com/forum/viewtopic.php?f=87&t=5326>`__
-  The Rover development team are delighted to announce the 2.44 release
   of the rover firmware.
-  The main change in this release is better support for the new Pixhawk
   autopilot, but there are also a lot of smaller changes that improve
   the Rover code.
-  The Pixhawk changes include support for dual sensors with failover,
   support for auto-update of the IO firmware and lots of small
   performance improvements.
-  Other changes include:

   -  Added support for the MAV_CMD_DO_SET_CAM_TRIGG_DIST MAVLink
      command
   -  Added SKIP_GYRO_CAL option
   -  Added support for MAVLink system time from GPS time
   -  Improved GPS timing for AHRS
   -  Lots of improvements to dataflash logging
   -  Support for MAVLink log download
   -  Support for reverse in STEERING mode

-  Added support for 2nd telemetry port on Pixhawk
-  This release is parameter compatible with previous releases, so it
   should be an easy update for all users.

Rover 2.43 Released
===================

-  October 10 2013 Rover 2.43 released. Details
   `here <http://diydrones.com/group/ardurover-user-group/forum/topics/apmrover2-2-43-released>`__
-  Tom and Tridge have just released APMrover2 2.43 for your driving
   pleasure!
-  This is a major new release with a lot of improvements, especially in
   the area of more precise steering and navigation. Highlights include:
-  adoption of L1 navigation code from Plane

   -  new more precise steering controller
   -  support for the new Pixhawk autopilot
   -  conversion to centimeter level precision throughout the code

-  I'll only be able to give a rough summary of the changes here as we
   have had more than 1500 changes in total go into the git tree since
   the last release.
-  Only some of those were related exclusively to the rover code, but a
   lot of them do affect rover.
-  The most significant change is in the navigation controller.

   -  Earlier this year the Plane code adopted a new L1 navigation
      system.
   -  That was highly successful, and led to the fixed-wing plane code
      improving its navigation performance enormously, with much
      smoother and more accurate turns, and much better handling of
      wind, along with simpler tuning.

-  Since then Tom and I have been working to bring the same benefits to
   the Rover code.

   -  It was a bit of a rocky start, with initial attempts not working
      well on real rovers.
   -  With a single set of tuning values we were able to get good
      results either on straight driving or in tight turns, but not in
      both with the same settings.

-  We finally made progress when we wrote a new STEER2SRV steering
   controller that takes the output of the L1 navigation code (as a
   demanded lateral acceleration) and produces the steering controls.

   -  The new controller was based initially on the fixed-wing plane
      roll controller, but adapted to the physics of wheeled vehicles.
   -  That controller replaces the old classic PID based steering
      controller in the rover code, and provides much smoother and more
      adaptable steering.
   -  After we got the tuning for it right, steering on both of our
      rovers is now much better than it has been previously.

-  The new steering code has been so successful in fact that I have now
   ported it back for use in Plane to control the ground steering on
   planes when taxiing and in automatic takeoff and landing.

   -  It has really helped with one of my planes that tends to be
      difficult to control during takeoff.
   -  So Rover and Plane now use the same ground steering code, just
      with different tuning parameters.

-  The next big change that you may notice in this release is the
   internal move to 32 bit integer positions instead of 32 bit floating
   point positions.

   -  The reason is that 32 bit floats that represent a global
      latitude/longitude only have a precision of a bit over 1 meter in
      some parts of the world.
   -  On a plane flying at 60 knots this isn't a problem, as 1m accuracy
      is great, but for a rover it is not good.
   -  Switching to 32 bit integers gives us a precision of about 1cm,
      which is a good level for a rover.
   -  This lowers the amount of weaving of the rover as it no longer
      tends to jump across 1m boundaries.

-  That change to 1cm accuracy had impacts in a lot of different parts
   of the code.

   -  For example, the code which takes account of GPS lag to predict
      the actual position of the rover needs to do its math carefully to
      avoid ending up with floating point rounding errors.
   -  The NMEA GPS driver needed to be modified quite a lot to read the
      floating point numbers in the NMEA protocol in a way that
      preserves centimetre level precision.

-  The centimeter precision change doesn't mean that every rover will
   now navigate with cm precision.

   -  The rover will still be limited by the precision of its GPS and
      how good its steering linkages are.
   -  But if you have a good (expensive!) GPS and good hardware then
      Rover should be able to get much better results with this release
      than it could previously.

-  Support for the new Pixhawk autopilot is another major feature of
   this release.

   -  Most of my testing for this release has been done with a Pixhawk
      and it has worked very well.
   -  We still support the APM1, APM2 and PX4 of course, it is just nice
      to also work with the latest hardware.
   -  We also are working on supporting Rover on embedded Linux boards
      like the BeagleBone black, and Mike McCauley is working on a port
      to the Flymaple board.

-  For this release you will need to read the new navigation tuning
   guide for Rover as quite a few parameters have changed.

   -  The full parameter list is also useful as a reference.

-  There have been a lot of other smaller changes in this release. Here
   is a partial list of the more important ones:

   -  added new AP_BattMonitor library for common battery handling with
      copter and plane
   -  use GPS projected position based on estimated GPS lag for more
      precise waypoint turns
   -  changed STEERING more to allow for throttle nudging
   -  added RCMAP support for mapping RC channels in any order
   -  switched to using new AP_Scheduler scheduling
   -  enabled RELAY support
   -  added LEARN_CH parameter for setting which channel triggers
      waypoint learning
   -  support camera stabilization and camera mount controls
   -  adopted new AP_Notify system for controlling LEDs on different
      boards
   -  lowered default accel/gyro filter frequency to cope with rough
      surfaces (eg. grass)

-  Tom and I wish you a lot of happy rovering with this release!
-  Cheers, Tridge

Rover 2.42 Released
===================

-  July 1, 2013: Rover version 2.42 released.
   Details \ `here <http://diydrones.com/group/ardurover-user-group/forum/topics/apmrover2-2-42-released>`__
-  This release contains numerous small improvements that have
   accumulated over the last couple of months.
-  Many of the improvements came out of the period of intensive
   development and testing that Tom and I put into the code before the
   ACV2013 competition (which we were delighted to win!)
-  The main improvements are:

   -  fixed voltage calculation for RSSI receiver signal logging
   -  use projected GPS position for waypoint calculations. This helps a
      lot for tight missions, such as around the barrels at AVC
   -  disable cross-tracking over last 3 meters of each mission leg, to
      prevent fishtailing as a waypoint is approached
   -  fixed change of speed command in missions (it was off by a factor
      of 100x, leading to some crazy speeds!)
   -  fixed MAVLink logging of sonar distances
   -  made it possible to change :ref:`AHRS_ORIENTATION <AHRS_ORIENTATION>` at runtime without a reboot
   -  added new RCMAP\_ parameters to allow for having RC input channels in any order
   -  added new internal task scheduler for more accurate timing
   -  fixed HIL simulation code
   -  fixed handling of PX4 servo arming
   -  changed
      the :ref:`AUTO_TRIGGER_PIN <AUTO_TRIGGER_PIN>` to
      be push-on/push-off

-  Meanwhile Tom and I are working towards adding the L1 navigation
   controller for the next release.

   -  We didn't put that in this release as we aren't yet happy enough
      with the performance. We hope to get that fixed soon!

-  The other thing that has changed since the last release is that we
   have switched to the new rover documentation site.

   -  The docs have improved a lot, although there is still more to do!

Happy rovering!

Previous Releases
=================

-  May 15, 2013: Project retitled Rover to conform with the rest of the
   APM projects, which are now no longer just Arduino-based.
-  May 6, 2013: Rover 2.41 released. Bug fixes and performance
   improvements.
   Details \ `here <http://diydrones.com/group/ardurover-user-group/forum/topics/apmrover2-2-41-released>`__
-  April 6, 2013: Rover 2.40 released. Major enhancement of the Rover
   code, adding many performance enhancements and bringing it in sync
   with the latest Plane code and current APM and Mission Planner
   features.
   Details \ `here <http://diydrones.com/group/ardurover-user-group/forum/topics/apmrover2-2-40-released>`__
-  June 16, 2012: Rover successfully finishes the \ `Sparkfun Autonomous Vehicle Competition <https://www.sparkfun.com/news/767>`__
-  May 2, 2012: Jean-Louis Naudin ports APM code to rovers
