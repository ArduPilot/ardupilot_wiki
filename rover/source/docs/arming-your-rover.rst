.. _arming-your-rover:

==================
Arming / Disarming
==================

Before you can driver your rover you need to arm it. Arming the rover
before flight has two purposes:

-  prevent the motor from turning when the user is not ready to driver (a
   safety feature)
-  prevent movement before the autopilot is fully configured and ready to
   go

In past releases of APM:Rover arming was optional, and the requirement
to arm (controlled by the ARMING_REQUIRED parameter) was disabled by
default. This was changed for the 3.0.0 release to require arming by
default.

The key thing that arming does is to enable the motor. You will not be
able to start the motor (ie. control the throttle) until the rover is
armed.

**Note**: If you have AHRS_EKF_USE enabled (you are using the EKF)
then it is particularly important that you have arming enabled with
arming checks enabled. Using EKF without arming checks may cause a
crash.

Configuring Arming
==================

There are two parameters which control how arming works:

-  **ARMING_REQUIRE**: this controls whether an arming step is
   required. The default is 1, meaning that arming is required before
   moving. If set to 0 then arming is not required (the rover starts
   off armed).
-  **ARMING_CHECK**: this controls what checks the autopilot does
   before arming is allowed. The default is 1, meaning all checks are
   done. Most users should leave it at 1, as the arming checks are
   important to ensure the autopilot is ready.

How to Arm
==========

When you are ready to go you can ask Rover to arm. This can be done in
two ways:

-  **Steering Arming**. Hold the steering stick fully to the right and
   make sure the throttle stick is centred in the dead zone (DZ) for 2 seconds.
-  **GCS Arming**. Press the arming button on your ground station

How to Disarm
=============

This is whilst the throttle is in the DZ (normally centred with hands
off) and steering to the left for 2 seconds. In ArduRover this
condition could be accidentally triggered while driving so there are
additional requirements prior to disarm:

-  The ArduPilot controller needs to make sure that you are not actually 
   driving. You need to be in a manual mode.  This can be MANUAL,
   HOLD, LEARNING or STEERING.  If your in AUTO, RTL or GUIDED you won't be able to disarm.

You can also disarm without using the transmitter with one of the 
following methods:

-  use a ground station to issue a disarm command
-  use the safety switch on your rover (on Pixhawk)

.. note::

   If you have a skid steering rover you will not be able to disarm
   via the transmitter stick as the rover would just turn around and
   around in circles on the spot whilst you were trying to disarm

Visual and Audible signals
==========================

APM:Rover will provide visual and audio clues to the arming state if
your autopilot has LEDs and a buzzer. The clues are:

-  if the autopilot is disarmed, but is ready to arm then the large
   3-colour LED will be flashing green
-  if the autopilot is armed and ready to fly the large 3-colour LED is
   solid green
-  when the autopilot is ready to arm it will play a "ready to arm"
   sound on the buzzer
-  when the autopilot is armed or disarmed it will play the
   corresponding sound

See the :ref:`sounds page <common-sounds-pixhawkpx4>` to listen to what the
buzzer sounds like for each state.

Arming Checks
=============

Before allowing arming the autopilot checks a set of conditions. All
conditions must pass for arming to be allowed. If any condition fails
then a message explaining what failed is set to the GCS.

The checks performed are:

-  Safety switch. The PX4 safety switch must be set to the safety-off
   state before arming is allowed. This is either done by pressing the
   safety switch for 2 seconds until it stops flashing, or you can
   disable the use of the safety switch by setting BRD_SAFETY_ENABLE=0
-  Inertial Sensor Checks. The accelerometers and gyroscopes must all be
   healthy and all be calibrated. If you have more than one accel or
   gyro then they need to be consistent with each other.
-  AHRS checks. The AHRS (attitude heading reference system) needs to be
   initialized and ready. Note that if you have the EKF enabled this may
   take up to 30 seconds after boot.
-  Compass checks. All compasses must be configured and calibrated, and
   need to be consistent with each other (if you have more than one
   compass)
-  GPS Checks. You need to have a 3D GPS fix.
-  Battery checks. The battery voltage must be above the failsafe
   voltage (if configured)
-  Logging checks. The logging subsystem needs to be working (ie. a
   microSD must be fitted and working on PX4)
-  RC Control checks. You need to not be in RC failsafe

Throttle output when disarmed
=============================

When the rover is disarmed the throttle channel will not respond to
input. There are two possible behaviours you can configure:

-  ARMING_REQUIRE=1. When disarmed the trim value for the throttle
   channel (normally RC3_TRIM) will be sent to the throttle channel
-  ARMING_REQUIRE=2. When disarmed no pulses are sent to the throttle
   channel. Note that some ESCs will beep to complain that they are
   powered on without a control signal

Diagnosing failure to arm
=========================

It can be frustrating if your rover refuses to arm. To diagnose arming
issues follow this guide

Check it is ready to arm
------------------------

If your board has a "ready to arm" LED (the large LED in the middle of
the board on a Pixhawk) then that LED should be flashing green when the
board is ready to arm. If it is flashing yellow then that indicates that
one of the arming checks is not passing.

Try arming
----------

Try sending an arm command with your GCS. If arming is refused then a
message will be sent from the autopilot to the GCS indicating why it is
refusing to arm.

Steering arming
---------------

If you are using right-steer + trim-throttle to arm and you don't get a
message on your GCS giving a arming failure reason then it may be that
your RC calibration is a bit off and the autopilot is not quite seeing
trim throttle or isn't quite seeing full right steering.

Reasons for refusing to arm
---------------------------

When the autopilot refuses to arm it sends a STATUSTEXT MAVLink message
to the GCS explaining why it is refusing. The possible reasons why the
autopilot can refuse to arm are:

-  **logging not available**. If your microSD card has failed or is
   corrupt then logging won't be available and you cannot arm.
-  **gyros not healthy**. If the gyros have failed the autopilot will
   refuse to arm. This is rare, and if it happens repeatedly then you
   may have a hardware failure.
-  **gyros not calibrated**. This happens when the automatic gyro
   calibration at startup didn't converge. Try rebooting the autopilot
   with the rover still.
-  **accels not healthy**. If the accelerometers have failed the
   autopilot will refuse to arm. Try recalibrating your accelerometers.
-  **GPS accuracy errors**. There are 4 types of GPS arming errors that
   can be reported. They are "GPS vert vel error", "GPS speed error",
   "GPS horiz error", "GPS numsats". Try moving your rover for better
   GPS reception or switching off any RF sources (such as a FPV
   transmitter) that may be interfering with your GPS.
-  **Mag yaw error**. This happens when your compass is badly out of
   alignment. Check your compass orientation and re-do your compass
   calibration or move your rover further away from any magnetic
   materials.
-  **EKF warmup**. This happens when the EKF is still warming up. Wait
   another 10 seconds and try again.
-  **AHRS not healthy**. This means the EKF is not healthy. If the error
   persists then try rebooting your board.
-  **3D accel cal needed**. This happens when you have not done a 3D
   accelerometer calibration.
-  **Inconsistent accelerometers**. This happens when you have multiple
   IMUs (such as the Pixhawk which has two) and they are not consistent.
   This can be caused by temperature changes. If the error doesn't clear
   itself after a minute you will need to redo your accelerometer
   calibration.
-  **Inconsistent gyros**. This happens when you have multiple gyros and
   they are not reporting consistent values. If the error does not clear
   itself after 30 seconds then you will need to reboot.
-  **GPS n has not been fully configured**. This happens with a uBlox
   GPS where the GPS driver is unable to fully configure the GPS for
   the settings that are being requested. This can be caused by a bad
   wire between the autopilot and GPS, or by a bad response from the
   GPS. If the message is about "GPS 0" then it is the first GPS. If
   it is "GPS 1" then it is the 2nd GPS. If you get a failure for the
   2nd GPS and don't have two GPS modules installed then set GPS_TYPE2
   to zero to disable the 2nd GPS

