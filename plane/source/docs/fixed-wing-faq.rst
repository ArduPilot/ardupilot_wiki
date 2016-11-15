.. _fixed-wing-faq:

==============
Fixed Wing FAQ
==============

This is a set of frequently asked questions with answers. It is created
when we see questions in `the forums <http://ardupilot.com/forum/viewforum.php?f=1>`__ or on
`discuss.ardupilot.org <http://discuss.ardupilot.org/c/arduplane>`__ that are not sufficiently
answered in the rest of the docs.

When reading this FAQ please refer to the :doc:`full parameter list <parameters>` for an explanation of each
parameter which is mentioned in the answers.

How do you stop a Nitro plane from cutting the engine in flight?
----------------------------------------------------------------

For internal combustion motor planes (which can be prone to cutting the
engine at low throttle), you should use the following settings:

-  THR_MIN=10
-  THR_PASS_STAB=1
-  THR_SUPP_MAN=1

That will prevent the throttle dropping below 10%, but will give you
manual throttle control for idling while on the ground, and manual
throttle control in stabilisation modes (such as FBWA and STABILIZE) for
shutting down the motor when you need to.

THR_SLEWRATE can also aid in prevention of a nitro engine stalling in
flight by slowing the throttle advance from going wide open to quickly.

How do you prevent the servo demo and have faster startups?
-----------------------------------------------------------

Set the parameter SKIP_GYRO_CAL=1. That will disable the servo demo
and will also skip the gyro calibration done on each startup. The last
gyro calibration from when you calibrated the accelerometers will be
used instead. This also means you don't need to hold the plane still
when booting.

The downside of using SKIP_GYRO_CAL is that your gyros may have
drifted since you last calibrated them, perhaps due to temperature
changes. If you have ARMING_REQUIRE enabled then the arming checks will
catch that problem if it is very large, but in general we recommend not
using SKIP_GYRO_CAL unless you have a good reason to use it. This is
especially the case if you have the EKF enabled, as it is particularly
sensitive to gyro error.

Tuning is too difficult, how do I make it easier?
-------------------------------------------------

Please see the :ref:`documentation for autotune <automatic-tuning-with-autotune>`

How do I setup reverse throttle on a IC plane?
----------------------------------------------

Some planes (mostly nitro or petrol planes) have a reversed throttle
servo, so lower PWM values on the throttle channel gives more throttle
not less. To setup Plane to handle this you need to change 3 settings:

-  set RC3_REV to -1
-  setup your transmitter for reverse throttle
-  change THR_FS_VALUE to a high value instead of a low value (eg.
   2100 instead of 900). It needs to be a higher PWM value than you will
   use in normal flight

After you setup reverse throttle make sure you test correct failsafe by
turning off your transmitter while on the ground.

What happens if an airspeed sensor fails in flight?
---------------------------------------------------

What happens when airspeed fails depends on the type of failure. The
most likely scenarios are:

-  the I2C cable fails with a digital airspeed sensor
-  the pitot tube gets partially blocked, leading to incorrect scaling
   of the pressure (either too low or high)
-  the pitot tube gets completely blocked, leading to fixed readings
   (either too low or high)

In the first case where an I2C failure is detected the code will detect
the failure and stop using the airspeed sensor. The aircraft will
continue to fly with the algorithms it uses for an aircraft with no
airspeed sensor. The ground station will be notified of the failure, and
should display a warning to the user.

For the other two cases the code does not currently have a reliable way
to detect an airspeed failure. If the failure leads to a low airspeed
reading then if the plane is in an auto-throttle mode (such as AUTO,
GUIDED, LOITER or RTL) then the plane will tend to lose altitude as it
tries to gain speed. The amount of altitude it will lose depends on how
low the airspeed reading is. If the airspeed reading it low enough then
it may trigger a fast enough descent to crash the aircraft.

If the failure leads to a too high airspeed reading then the plane will
slow down to try to keep its airspeed at the target airspeed. If the
reading is high enough then the plane may slow down enough to cause a
stall and crash the aircraft.

We are looking at ways of detecting partial airspeed sensor failures and
hope to add some protection into the code in the future.

Why don't my surfaces move enough with FLAPERON_OUTPUT, VTAIL_OUTPUT or ELEVON_OUTPUT?
-----------------------------------------------------------------------------------------

You are probably using the default MIXING_GAIN of 0.5. The default is
setup to prevent channel saturation. If you instead want to be able to
have full deflection then try setting MIXING_GAIN=1.0 or something in
between.

How do I get a good flare in automatic landing?
-----------------------------------------------

Please see :ref:`this page <automatic-landing>`

How do I reset all parameters to defaults?
------------------------------------------

To reset all parameters set the parameter FORMAT_VERSION to 0 and
reboot. When ArduPilot starts up it checks if FORMAT_VERSION has the
correct value, and if it doesn't it wipes the parameters, which resets
them to the default values.

What does "Bad AHRS" mean on a ground station?
----------------------------------------------

It means the "Attitude Heading Reference System" is unhealthy. That is
the software that determines the attitude of the aircraft. The most
common cause is bad 3D accelerometer calibration when AHRS_EKF_USE is
set to 1. Check your accelerometer calibration.

How do I reduce throttle oscillation in auto flight?
----------------------------------------------------

There are 3 parameters that affect the amount the throttle changes in
automatic flight.

-  THR_SLEWRATE is the percentage of throttle change allowed per
   second. A value of 100 means the throttle cannot change over its full
   range in less than 1 second.
-  TECS_THR_DAMP is a damping factor for throttle control. The default
   is 0.5. A higher value will dampen throttle changes.
-  TECS_TIME_CONST is the overall time constant for both throttle and
   pitch changes in TECS. It controls how rapidly TECS tries to correct
   for any error in speed or height. It is in seconds, and defaults to
   5. A higher value makes the pitch and throttle corrections happen
   more slowly.

Why do I get small surface movements in ground tests?
-----------------------------------------------------

Before takeoff it is common to look at the amount of movement of
ailerons and elevator when the plane is rolled and pitched on the
ground. Some users have wondered why the amount of movement they see in
this test is less in recent releases of the firmware.

The reason is the new :ref:`stall prevention code <stall-prevention>`. When the plane is on
the ground the airspeed is very low, so is always under the minimum
airspeed set in ARSPD_FBW_MIN. That means the maximum roll demand is
limited to 25 degrees, which means the amount of demanded aileron
surface movement is less than it would be without stall prevention.

If you want to see what the movement would be without stall prevention
then just set STALL_PREVENTION=0. Remember to turn it back on before
you fly.

.. _fixed-wing-faq_how_would_i_setup_crow_flaps:

How would I setup crow flaps?
-----------------------------

Crow flaps combined flaperons with normal flaps, but the flaperons move
upward when the flaps are engaged. Crow flaps can add a lot of drag to
slow an aircraft for landing without inducing a lot of pitching moment.

To setup crow flaps you will need to combine two features. First you
will need to :ref:`setup flaperons <flaperons-on-plane>` on two
output channels using the :ref:`flaperon output channels functions <channel-output-functions_flaperon1_and_flaperon2>`.
You will need to choose the FLAPERON_OUTPUT parameter value so that the
flaps go up instead of down when flaps are engaged, while being careful
that aileron input goes in the right direction for roll.

Then you should separately setup 1 or 2 flap channels (depending on
whether your flap servos are setup to use a Y lead) using the
:ref:`flap_auto output channel function <channel-output-functions_flap_auto>`.

It is strongly suggested that you also setup a FLAP_INPUT_CHANNEL on
an RC input channel to allow easy testing of flaps on the ground, and to
give manual flap control for testing in FBWA mode. That will allow you
to test what degree of flap movement produces the desired increase in
drag without inducing a stall.

Why do my servos jitter when on the ground?
-------------------------------------------

When the aircraft is on the ground in a mode where it is doing attitude
stabilization (such as FBWA mode) the servos often move about a small
amount, even though the aircraft is not moving.

The reason this happens is the attitude estimation code is doing it's
best to estimate the attitude of the aircraft, and it is getting a small
amount of false input. The sources of the false input are:

-  if you have GPS lock then the GPS may be reporting a small amount of
   velocity change (GPS noise). This gets used to correct the
   accelerometers and comes out as a small amount of attitude noise,
   resulting in small attitude corrections.
-  if you don't have GPS lock but you have an airspeed sensor then the
   DCM code will try to use cross-product of the airspeed with the gyros
   to estimate inertial force corrections to the accelerometers. The
   airspeed is quite noisy at low speed, so this effect can be quite
   large

Both of these effects are smaller if you enable the EKF (with
AHRS_EKF_USE=1) as it has smarter logic for handling attitude
estimation when on the ground.

How is airspeed used with no airspeed sensor?
---------------------------------------------

When you have an aircraft with no airspeed sensor Plane uses a range of
techniques to fly as reliably as possible despite the lack of airspeed
sensor data. The techniques are:

-  a synthetic airspeed estimate is calculated by the AHRS system by
   combining a wind estimate, the GPS ground speed and the response of
   the aircraft when turning. This airspeed estimate is usually quite
   good, although it is not as accurate as a real airspeed sensor.
-  for speed and height control, a different algorithm in TECS is used
   that does not rely on an airspeed measurement. The algorithm
   primarily relies on using throttle to maintain the desired height,
   relying on the fact that an aircraft will start to sink if its
   airspeed is too low. See the TECS code for full details.
-  For surface speed scaling (the change in control surface movement
   needed with different airspeed) the synthetic airspeed estimate is
   used.
-  For stall prevention (if enabled) the synthetic airspeed is used

When no airspeed sensor is available some parameters are not used for
some purposes:

-  the TRIM_ARSPD_CM parameter is not used as an airspeed target in
   auto flight. Instead the TRIM_THROTTLE parameter is used as base
   throttle, with extra throttle added/removed to retain the target
   altitude
-  the ARSPD_FBW_MIN and ARSPD_FBW_MAX parameters are not used for
   airspeed limiting in TECS, but they are still used for the stall
   prevention code, using the synthetic airspeed value

Why does my trim change when I change modes?
--------------------------------------------

Some people experience a problem where their roll or pitch trim changes
when they change flight modes. So for example they are trimmed with
level aileron in manual with the aircraft level and when they change to
FBWA mode the ailerons move significantly off centre trim.

One likely cause of this is that you have a transmitter that has per
flight mode trims. The Taranis is a good example of this if you use its
builtin flight mode controls. What happens is you setup the plane with
correct trims in MANUAL by adjusting using the trip tabs in flight, but
those trims don't get used when you change flight modes. You need to
change your transmitter settings so that the stick inputs are the same
in all flight modes.
