.. _arming-your-plane:

============
Arming Plane
============

Before you can fly your plane you need to arm it. Arming the aircraft
before flight has two purposes:

-  prevent the motor from turning when the pilot is not ready to fly (a
   safety feature)
-  prevent takeoff before the autopilot is fully configured and ready to
   fly

In past releases of APM:Plane arming was optional, and the requirement
to arm (controlled by the ARMING_REQUIRED parameter) was disabled by
default. This was changed for the 3.3.0 release to require arming by
default.

The key thing that arming does is to enable the motor. You will not be
able to start the motor (ie. control the throttle) until the aircraft is
armed.

**Note**: If you have AHRS_EKF_USE enabled (you are using the EKF)
then it is particularly important that you have arming enabled with
arming checks enabled. Flying EKF without arming checks may cause a
crash.

Configuring Arming
==================

There are three parameters which control how arming works:

-  **ARMING_REQUIRE**: this controls whether an arming step is
   required. The default is 1, meaning that arming is required before
   takeoff. If set to 0 then arming is not required (the plane starts
   off armed).
-  **ARMING_CHECK**: this controls what checks the autopilot does
   before arming is allowed. The default is 1, meaning all checks are
   done. Most users should leave it at 1, as the arming checks are
   important to ensure the autopilot is ready.
-  **ARMING_RUDDER**: This parameter allows you to configure rudder 
   based arming/disarming. The default is 1, meaning you are able to 
   arm with right rudder. If you set this to 2 you can also disarm 
   with left rudder. If you set this to 0 then you will only be able 
   to arm/disarm via a ground station.

How to Arm
==========

When you are ready to fly you can ask Plane to arm. This can be done in
two ways:

-  **Rudder Arming**. Hold the rudder stick fully to the right and the
   throttle stick fully down for 2 seconds.
-  **GCS Arming**. Press the arming button on your ground station

How to Disarm
=============

Since APM:Plane 3.4.0 it is possible to disarm using the transmitter. 
This is done holding throttle at minimum and rudder to the left for 2 
seconds. In ArduPlane this condition could be accidentally triggered by 
pilots while flying so there are additional requirements prior to disarm:

-  You need to allow rudder disarming by changing **ARMING_RUDDER** 
   parameter to 2 (ArmOrDisarm).
-  The flight controller needs to make sure that you are not actually 
   flying. There is an algorithm for this that uses the **airspeed sensor** 
   readings. So you need this source available and giving values lower 
   enough (in a windy day you might not be able to disarm even landed 
   if the plane thinks you are still flying)

You can also disarm without using the transmitter with one of the 
following methods:

-  use a ground station to issue a disarm command
-  use the safety switch on your aircraft (on Pixhawk)
-  after an auto-landing the plane will automatically disarm after 20
   seconds if still on the ground (controlled by LAND_DISARMDELAY
   parameter)

Visual and Audible signals
==========================

APM:Plane will provide visual and audio clues to the arming state if
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

-  Safety switch. The safety switch must be set to the safety-off
   state before arming is allowed. This is either done by pressing the
   safety switch for 2 seconds until it stops flashing, or you can
   disable the use of the safety switch by setting BRD_SAFETY_ENABLE=0
-  Barometer check. The barometer must be healthy (getting good data)
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
-  Airspeed checks. If you have configured an airspeed sensor then the
   sensor needs to be working.
-  Logging checks. The logging subsystem needs to be working (ie. a
   microSD must be fitted and working)
-  RC Control checks. You need to not be in RC failsafe

Throttle output when disarmed
=============================

When the plane is disarmed the throttle channel will not respond to
pilot input. There are two possible behaviours you can configure:

-  ARMING_REQUIRE=1. When disarmed the minimum value for the throttle
   channel (normally RC3_MIN) will be sent to the throttle channel
-  ARMING_REQUIRE=2. When disarmed no pulses are sent to the throttle
   channel. Note that some ESCs will beep to complain that they are
   powered on without a control signal

Diagnosing failure to arm
=========================

It can be frustrating if your plane refuses to arm. To diagnose arming
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

Rudder arming
-------------

If you are using right-rudder + zero-throttle to arm and you don't get a
message on your GCS giving a arming failure reason then it may be that
your RC calibration is a bit off and the autopilot is not quite seeing
zero throttle or isn't quite seeing full right rudder.

Reasons for refusing to arm
---------------------------

When the autopilot refuses to arm it sends a STATUSTEXT MAVLink message
to the GCS explaining why it is refusing. The possible reasons why the
autopilot can refuse to arm are:

-  **barometer not healthy**. This is very rare. If it happens
   repeatedly then you may have a barometer hardware fault.
-  **airspeed not healthy**. If you have a airspeed sensor fitted and
   the autopilot is not getting an airspeed reading it will refuse to
   arm.
-  **logging not available**. If your microSD card has failed or is
   corrupt then logging won't be available and you cannot arm.
-  **gyros not healthy**. If the gyros have failed the autopilot will
   refuse to arm. This is rare, and if it happens repeatedly then you
   may have a hardware failure.
-  **gyros not calibrated**. This happens when the automatic gyro
   calibration at startup didn't converge. Try rebooting the autopilot
   with the plane held still.
-  **accels not healthy**. If the accelerometers have failed the
   autopilot will refuse to arm. Try recalibrating your accelerometers.
-  **GPS accuracy errors**. There are 4 types of GPS arming errors that
   can be reported. They are "GPS vert vel error", "GPS speed error",
   "GPS horiz error", "GPS numsats". Try moving your plane for better
   GPS reception or switching off any RF sources (such as a FPV
   transmitter) that may be interfering with your GPS.
-  **Mag yaw error**. This happens when your compass is badly out of
   alignment. Check your compass orientation and re-do your compass
   calibration or move your plane further away from any magnetic
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
-  **Limit errors**. The arming checks some of your parameter settings
   to make sure they are in a reasonable range. The checks are
   "LIM_ROLL_CD too small", "LIM_PITCH_MAX too small",
   "LIM_PITCH_MIN too large", "invalid THR_FS_VALUE".
-  **GPS n has not been fully configured**. This happens with a uBlox
   GPS where the GPS driver is unable to fully configure the GPS for
   the settings that are being requested. This can be caused by a bad
   wire between the autopilot and GPS, or by a bad response from the
   GPS. If the message is about "GPS 0" then it is the first GPS. If
   it is "GPS 1" then it is the 2nd GPS. If you get a failure for the
   2nd GPS and don't have two GPS modules installed then set GPS_TYPE2
   to zero to disable the 2nd GPS

