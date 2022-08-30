.. _fixed-wing-faq:

==============
Fixed Wing FAQ
==============

This is a set of frequently asked questions with answers. 
It is updated when we see questions on `discuss.ardupilot.org <https://discuss.ardupilot.org/c/arduplane>`__ that are not sufficiently answered in the rest of the docs.

When reading this FAQ please refer to the :doc:`full parameter list <parameters>` for an explanation of each parameter which is mentioned in the answers.

How do you stop a Nitro plane from cutting the engine in flight?
----------------------------------------------------------------

For internal combustion motor planes (which can be prone to cutting the engine at low throttle), you should use the following settings:

-  :ref:`THR_MIN<THR_MIN>` =10
-  :ref:`THR_PASS_STAB<THR_PASS_STAB>` =1
-  :ref:`THR_SUPP_MAN<THR_SUPP_MAN>` =1

That will prevent the throttle dropping below 10%, but will give you
manual throttle control for idling while on the ground, and manual
throttle control in stabilisation modes (such as FBWA and STABILIZE) for
shutting down the motor when you need to.

:ref:`THR_SLEWRATE<THR_SLEWRATE>` can also aid in prevention of a nitro engine stalling in
flight by slowing the throttle advance from going wide open to quickly.

How do you inhibit gyro calibration?
------------------------------------

Set the parameter :ref:`INS_GYR_CAL<INS_GYR_CAL>` =0. That will skip the gyro calibration 
done on each startup. The last
gyro calibration from when you calibrated the accelerometers will be
used instead. This also means you don't need to hold the plane still
when booting.

The downside of skipping gyro calibration is that your gyros may have
drifted since you last calibrated them, perhaps due to temperature
changes. If you have :ref:`ARMING_REQUIRE<ARMING_REQUIRE>` enabled then the arming checks will
catch that problem if it is very large, but in general we recommend not
skipping gyro calibration unless you have a good reason to use it. This is
especially the case if you have the EKF enabled, as it is particularly
sensitive to gyro error.

Tuning is too difficult, how do I make it easier?
-------------------------------------------------

Please see the :ref:`documentation for autotune <automatic-tuning-with-autotune>`

How do I setup reverse throttle on a IC plane?
----------------------------------------------

Some planes (mostly nitro or petrol planes) have a reversed throttle
servo, so lower PWM values on the throttle channel gives more throttle
not less.

After you setup reverse throttle make sure you test correct failsafe by
turning off your transmitter while on the ground.

The servo library allows you to reverse the 
throttle output channel without affecting your RC inputs or failsafe configuration.
to do this, set :ref:`SERVO3_REVERSED<SERVO3_REVERSED>` to 1.

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
low the airspeed reading is. If the airspeed reading is low enough then
it may trigger a fast enough descent to crash the aircraft.

If the failure leads to a too high airspeed reading then the plane will
slow down to try to keep its airspeed at the target airspeed. If the
reading is high enough then the plane may slow down enough to cause a
stall and crash the aircraft.

We are looking at ways of detecting partial airspeed sensor failures and
hope to add some protection into the code in the future.

Why don't my surfaces move enough when using flaperons, elevons, or v-tail?
-----------------------------------------------------------------------------------------

You are probably using the default :ref:`MIXING_GAIN<MIXING_GAIN>` of 0.5. The default is
setup to prevent channel saturation. If you instead want to be able to
have full deflection then try setting :ref:`MIXING_GAIN<MIXING_GAIN>` =1.0 or something in
between.

How do I get a good flare in automatic landing?
-----------------------------------------------

Please see :ref:`this page <automatic-landing>`

How do I reset all parameters to defaults?
------------------------------------------

To reset all parameters set the parameter :ref:`FORMAT_VERSION<FORMAT_VERSION>` to 0 and
reboot. When ArduPilot starts up it checks if :ref:`FORMAT_VERSION<FORMAT_VERSION>` has the
correct value, and if it doesn't it wipes the parameters, which resets
them to the default values.

What does "Bad AHRS" mean on a ground station?
----------------------------------------------

It means the "Attitude Heading Reference System" is unhealthy. That is
the software that determines the attitude of the aircraft. Possible causes:

- Accelerometer calibration
- GPS has not acquired a good enough lock (#sats, HDOP, etc.)
- EKF has not settled (tilt/yaw initialization, origin not set yet, variances, etc.). 
  **"EKF IMUx using GPS"** message will be displayed on ground control station when EKF is ready.
  
.. note:: if no compass is enabled, the **"EKF IMUx using GPS"** message will be not be displayed until after flight begins, since EKF yaw alignment will not occur until sufficient ground speed is acquired for the GPS to provide a heading. This is normal in this situation.

How do I reduce throttle oscillation in auto flight?
----------------------------------------------------

There are 3 parameters that affect the amount the throttle changes in
automatic flight.

-  :ref:`THR_SLEWRATE<THR_SLEWRATE>` is the percentage of throttle change allowed per
   second. A value of 100 means the throttle cannot change over its full
   range in less than 1 second.
-  :ref:`TECS_THR_DAMP<TECS_THR_DAMP>` is a damping factor for throttle control. The default
   is 0.5. A higher value will dampen throttle changes.
-  :ref:`TECS_TIME_CONST<TECS_TIME_CONST>` is the overall time constant for both throttle and
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
airspeed set in :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` . That means the maximum roll demand is
limited to 25 degrees, which means the amount of demanded aileron
surface movement is less than it would be without stall prevention.

If you want to see what the movement would be without stall prevention
then just set :ref:`STALL_PREVENTION<STALL_PREVENTION>` = 0. Remember to turn it back on before
you fly.

.. _fixed-wing-faq_how_would_i_setup_crow_flaps:

How would I setup crow flaps?
-----------------------------

Crow flaps combine flaperons with normal flaps, but the flaperons move
upward when the flaps are engaged. Crow flaps can add a lot of drag to
slow an aircraft for landing without inducing a lot of pitching moment.

To setup crow flaps you :ref:`setup flaperons <flaperons-on-plane>` on two
output channels just as you would for normal flaperons. However, follow the crow instructions to make sure that the ailerons move upward when the flap channel is activated, instead of downward.


Why do my servos jitter when on the ground?
-------------------------------------------

When the aircraft is on the ground in a mode where it is doing attitude
stabilization (such as FBWA mode) the servos often move about a small
amount, even though the aircraft is not moving.

The reason this happens is the attitude estimation code is doing its
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

-  the :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` parameter is not used as an airspeed target in
   auto flight. Instead the :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` parameter is used as base
   throttle, with extra throttle added/removed to retain the target
   altitude
-  the :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` and :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` parameters are not used for
   airspeed limiting in TECS, but they are still used for the stall
   prevention code, using the synthetic airspeed value

Why does my trim change when I change modes?
--------------------------------------------

Some people experience a problem where their roll or pitch trim changes
when they change flight modes. So for example, in FBWA the plane is flying level, but
when changed to MANUAL mode the plane is no longer in level trim.

One cause of this is that you have a transmitter that has per
flight mode trims. The Taranis is a good example of this if you use its
built in flight mode controls. You need to be sure  your transmitter trim settings 
are the same, so that the stick inputs are the same in all flight modes and  match those
you used for the RC calibration setup.

Another, is that you are not using :ref:`SERVO_AUTO_TRIM<SERVO_AUTO_TRIM>` to automatically adjust the trim
of the pitch and roll controlling flying surfaces. Enabling this will assure that MANUAL mode trim matches that of the auto-leveled modes, like STABLIZE and FBWA.
