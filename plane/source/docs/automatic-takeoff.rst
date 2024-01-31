.. _automatic-takeoff:

=================
Automatic Takeoff
=================

Plane can automatically launch a wide range of aircraft types. The
instructions below will teach you how to setup your mission to support
automatic takeoff. Automatic takeoff can also be accomplished by the :ref:`takeoff-mode` supported in 
ArduPlane 4.0 and later versions.

Basic Instructions
==================

The basic idea of automatic takeoff is for the autopilot to set the
throttle to maximum and climb until a designated altitude is reached. To
cause the plane to execute a takeoff, add a NAV_TAKEOFF command to your
mission, usually as the first command. There are two parameters to this
command - the minimum pitch, and the takeoff altitude. The minimum pitch
controls how steeply the aircraft will climb during the takeoff. A value
of between 10 and 15 degrees is recommended for most aircraft. The
takeoff altitude controls the altitude above home at which the takeoff
is considered complete. Make sure that this is high enough that the
aircraft can safely turn after takeoff. An altitude of 40 meters is good
for a wide range of aircraft.

During takeoff the wings will be held level to within
:ref:`LEVEL_ROLL_LIMIT <LEVEL_ROLL_LIMIT>`
degrees until 5 meters of altitude is reached, gradually allowing more roll until
the normal :ref:`ROLL_LIMIT_DEG<ROLL_LIMIT_DEG>` limit is allowed once 15 meters of altitude is
reached. This prevents a sharp roll from causing the wings to hit the
runway for ground takeoffs.

Note that the takeoff direction is set from the direction the plane is
pointing when the automatic takeoff command is started. So you need to
point the plane in the right direction, then switch to AUTO mode. During
the first stage of the takeoff the autopilot will use the gyroscope as
the principal mechanism for keeping the aircraft flying straight, or the compass if
equipped. After sufficient speed for a good GPS heading is reached the aircraft will
switch to using the GPS ground track (and compass if so equipped) which allows it to account for a
cross-wind.

You should try to launch into the wind whenever possible.

Hand Launching
==============

A hand launch is a common method to launch smaller aircraft, such as
foam gliders. Plane has a number of parameters that control how a hand
launch is done. If you are planning to hand launch your aircraft in AUTO
mode then please look through these options carefully.

The keys to a good hand launch are:

-  if the propeller is behind your hand during launch then ensuring that
   the motor does not start until it is past your hand
-  ensuring that the aircraft does not try to climb out too steeply
-  assuring that the parameters are set to actually start the motor after the vehicle leaves your hand

.. note:: default values for the following parameters will usually start the motor as soon at takeoff is selected (mode or mission item). This may be fine for certain hand launch situations/vehicles, but adjusting the following parameters to non-default values may be needed for safety in other situations.

The main parameters that control the hand launch motor start are:

-  :ref:`TKOFF_THR_MINACC <TKOFF_THR_MINACC>`
-  :ref:`TKOFF_THR_DELAY <TKOFF_THR_DELAY>`
-  :ref:`TKOFF_THR_MINSPD <TKOFF_THR_MINSPD>`
-  :ref:`TECS_PITCH_MAX <TECS_PITCH_MAX>`

When the auto takeoff mission command starts (usually by switching to
AUTO mode) or you switch to TAKEOFF mode when armed, the autopilot starts in "throttle suppressed" mode. The
throttle will not start until the conditions set by the ``TKOFF_THR_x``
parameters are met.

The :ref:`TKOFF_THR_MINACC <TKOFF_THR_MINACC>` parameter controls the minimum forward
acceleration of the aircraft before the throttle will engage. The
forward acceleration comes from the throwing action of your arm as you
launch the aircraft. You need to set this value high enough that the
motor won't start automatically when you are carrying the aircraft
normally, but low enough that you can reliably trigger the acceleration
with a normal throwing action. A value of around 15 m/s/s is good for
most aircraft. The default value is 0, meaning this safeguard is not active.

Sometimes it is desirable to require multiple "shakes" of the vehicle above the :ref:`TKOFF_THR_MINACC <TKOFF_THR_MINACC>` before allowing a motor start. The :ref:`TKOFF_ACCEL_CNT<TKOFF_ACCEL_CNT>` parameter can be used to require multiple events exceeding this acceleration before proceeding in the motor start.

The :ref:`TKOFF_THR_DELAY <TKOFF_THR_DELAY>` parameter is a delay in 1/10 of a second units to
hold off starting the motor after the minimum acceleration or acceleration count is reached.
This is meant to ensure that the propeller is past your hand before the
motor starts. A value of at least 2 (which is 0.2 seconds and the default for the parameter) is
recommended for a hand launch. More may be needed.

The :ref:`TKOFF_THR_MINSPD <TKOFF_THR_MINSPD>` parameter is a minimum ground speed (as measured
by the GPS) before the motor starts after the above dealy. This is an additional safety
measure to ensure the aircraft is out of your hand before the motor
starts. A value of 4m/s is a good starting point if you wish this safety feature to be active. The default is 0, which would allow the motor to start immediately after the above delay with most GPSes since their velocity estimate is usually a bit  noisey.

Note that if your aircraft is a "tractor" type with the motor at the
front then you may want to use the default values of :ref:`TKOFF_THR_DELAY <TKOFF_THR_DELAY>` and :ref:`TKOFF_THR_MINSPD <TKOFF_THR_MINSPD>`, or use low values.

The final parameter you should think about is the :ref:`TECS_PITCH_MAX <TECS_PITCH_MAX>`
parameter. That controls the maximum pitch which the autopilot will
demand in auto flight. When set to a non-zero value this replaces the
:ref:`PTCH_LIM_MAX_DEG <PTCH_LIM_MAX_DEG>` parameter for all auto-throttle flight modes. Setting
this parameter to a value which is small enough to ensure the aircraft
can climb reliably at full throttle will make takeoff much more
reliable. A value of 20 is good for most aircraft.

.. note::  Hand launching a plane usually requires that the vehicle leaves your hand at zero degrees to 5 degs pitch. Higher pitches could lead to a stall. ArduPilot will not start the motors if the pitch is greater than 40 degrees or roll greater than 30 degrees from level. These limits should never be approached when throwing the plane, but are provided as a safety gate. 

Catapult Launching
==================

The main differences between catapult launching and hand launching is
that a catapult will usually give the aircraft a greater level of
acceleration, and the risk involved is primarily that the propeller will
strike the catapult frame instead of hitting your hand.

In most other ways a catapult launch is like a hand launch, and the same
4 key parameters apply. If your catapult is setup so that the motor
cannot run until the aircraft is out of the frame of the catapult then
you will need to choose the parameters to ensure there is sufficient
delay. Often this means a higher value for :ref:`TKOFF_THR_MINACC <TKOFF_THR_MINACC>` (say
20m/s/s) and a longer delay before the GPS ground speed is measured.
Some experimentation may be needed, but a value of :ref:`TKOFF_THR_DELAY <TKOFF_THR_DELAY>`
of 5 is likely to be good for many catapults.

Bungee Launching
================

A bungee launch uses a long piece of stretched elastic to launch the
aircraft. This can be a cheaper alternative to a catapult and gives good
results for a lot of small to medium sized models.

The same 4 parameters that apply to hand launch and catapult launch also
apply to a bungee launch, but the values you will need are different.
The main risk with a bungee launch (especially with a pusher propeller)
is that the propeller will strike the bungee cord, damaging either the
propeller or the bungee or both. To prevent this from happening you
should have a much higher value of :ref:`TKOFF_THR_DELAY <TKOFF_THR_DELAY>`, making it high
enough that the aircraft will have released the bungee before the motor
starts. A value of around 50 (giving a 5 second delay) may be a good
starting point.

Runway Takeoffs (CTOL)
======================

The final class of takeoff is runway takeoff, also known as wheeled
takeoff or CTOL (Conventional Takeoff and Landing). Setting up for a
good automatic takeoff from a runway is a bit more complex than the
other types of launches with more parameters to set and more tuning
required. This type of launch greatly benefits from the use of a compass
onboard since initial heading is critical.

One key consideration with runway takeoffs is whether you have a tail
dragger (tail wheel steering) or tricycle undercarriage (nose wheel
steering). Automatic takeoff is easier with a tricycle undercarriage
aircraft, with a tail dragger needing additional parameters.

The key parameters for runway takeoff are:

-  :ref:`TKOFF_TDRAG_ELEV <TKOFF_TDRAG_ELEV>`
-  :ref:`TKOFF_TDRAG_SPD1 <TKOFF_TDRAG_SPD1>`
-  :ref:`TKOFF_THR_SLEW <TKOFF_THR_SLEW>`
-  :ref:`TKOFF_ROTATE_SPD <TKOFF_ROTATE_SPD>`
-  :ref:`TECS_PITCH_MAX <TECS_PITCH_MAX>`
-  :ref:`GROUND_STEER_ALT <GROUND_STEER_ALT>`
-  :ref:`TKOFF_GND_PITCH<TKOFF_GND_PITCH>`

In addition to those parameters you also need to tune ground steering,
so that the ground steering controller is able to reliably steer the
aircraft. See the separate page on :ref:`setting up ground steering <tuning-ground-steering-for-a-plane>`. As part of this tuning
you will need to setup the :ref:`GROUND_STEER_ALT <GROUND_STEER_ALT>` parameter.

The first two parameters are primarily for tail dragger aircraft,
although they can also be used to hold the nose of a tricycle aircraft
down on takeoff.

The :ref:`TKOFF_TDRAG_ELEV <TKOFF_TDRAG_ELEV>` parameter is used to hold the tail of a tail
dragger hard on the runway during the initial stages of takeoff, to give
it enough grip on the runway to steer. For a tail dragger this is
normally set to 100, meaning that 100% up elevator is applied during the
initial stages of takeoff. For a tricycle undercarriage plane that needs
a bit of extra weight on the nose for good steering you may find that a
value of -20 (meaning 20% down elevator) may help.

When the takeoff starts, the autopilot will apply :ref:`TKOFF_TDRAG_ELEV <TKOFF_TDRAG_ELEV>`
elevator (as a percentage) until the aircraft reaches a speed of
:ref:`TKOFF_TDRAG_SPD1 <TKOFF_TDRAG_SPD1>` meters per second. You need to set
:ref:`TKOFF_TDRAG_SPD1 <TKOFF_TDRAG_SPD1>` to a speed below the takeoff speed, but above the
speed where the aircraft is able to steer using its rudder. When the
aircraft reaches :ref:`TKOFF_TDRAG_SPD1 <TKOFF_TDRAG_SPD1>` it will release the elevator and
instead use the normal flight pitch controller to try to hold the pitch
level. That will have the effect of raising the tail on a tail dragger
aircraft.

The :ref:`TKOFF_ROTATE_SPD <TKOFF_ROTATE_SPD>` parameter controls when the autopilot will try to
raise the nose (pitch up) to leave the ground. This needs to be a speed
at which the aircraft can sustain a climb, so it should be at least 2
meters per second above the stall speed of the aircraft, preferably
more. A higher value will mean a longer takeoff (and thus need more
runway). During the run, use :ref:`TKOFF_GND_PITCH<TKOFF_GND_PITCH>` to keep the aircraft on the runway while below this airspeed.

The :ref:`TKOFF_THR_SLEW <TKOFF_THR_SLEW>` parameter controls the throttle slew rate (as a
percentage per second) during takeoff. This is used to allow the
throttle to ramp up at a rate appropriate for your aircraft. How high
this should be depends on the type of aircraft. It is usually a good
idea for a ground takeoff to limit how fast the throttle ramps up to
prevent torque from the motor causing large steering changes. A value of
20 (meaning 20% throttle change per second) is good for many tail
draggers. A tricycle undercarriage aircraft may be able to handle a
larger throttle slew rate.

As with other types of takeoff the :ref:`TECS_PITCH_MAX <TECS_PITCH_MAX>` parameter controls
the maximum pitch used when climbing on takeoff. Make sure that this is
limited to a value that the aircraft can use to climb quickly at full
throttle. A value of around 20 degrees is good for a wide range of
aircraft.

.. _takeoff-fbwa:

Testing Ground Takeoff in FBWA mode
===================================

It is sometimes useful to test the takeoff code using the FBWA flight
mode. The way you do this is to set the ``FBWA_TDRAG_CHAN`` parameter (versions prior to 4.1) or  to an RC input channel on your transmitter for a switch (usually a
momentary switch, such as the trainer switch), or the channel's ``RCx_OPTION`` to 95 for versions 4.1 and later. When this RC channel goes high while you are on the runway waiting for takeoff in FBWA mode the
autopilot will check if you have configured the :ref:`TKOFF_TDRAG_ELEV <TKOFF_TDRAG_ELEV>` and
:ref:`TKOFF_TDRAG_SPD1 <TKOFF_TDRAG_SPD1>` parameters. If they have been set to non-zero
values then the elevator will be controlled in FBWA in an identical
manner to how it is controller for an AUTO takeoff. The elevator will go
to the :ref:`TKOFF_TDRAG_ELEV <TKOFF_TDRAG_ELEV>` value (usually 100% for a tail dragger) as
soon as that RC channel goes high, and will stay there until the
aircraft reaches an airspeed (if airspeed sensor is used) or groundspeed (if airspeed sensor is not used) of :ref:`TKOFF_TDRAG_SPD1 <TKOFF_TDRAG_SPD1>` meters per second.

This provides a convenient way to test auto takeoff in FBWA mode, and
also is a nice way to get better ground steering in FBWA mode in
general.

Speed Scaling Issues with no Airspeed Sensor
============================================

Since control effectiveness varies with airspeed, ArduPilot automatically scales the control gains in stabilized modes with airspeed to allow stability at low speeds and to avoid oscillations at high airspeeds. However, when an airspeed sensor is not used, an estimated airspeed based on GPS speed, accelerometer inputs, and position changes is used. During takeoffs into strong head wind, this estimate can be wrong and the gains scaled up, resulting in oscillations during the climb into the wind. Setting :ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>` bit 7 to 1, the speed scaling will be limited during the takeoff phase of automatic takeoffs to eliminate oscillations, particularly on tightly tuned vehicles.

Catapult Launch without an Airspeed Sensor
==========================================
Taking off without an airspeed sensor using a catapult may cause less than maximum throttle to be used due to high initial climb rates. For heavy vehicles, this may result in a stall due to the long time constants used in TECS to adjust throttle after the initial launch. The parameter :ref:`TKOFF_THR_MAX_T<TKOFF_THR_MAX_T>` can be used to force maximum throttle for a time, irrespective of climb rates from an initial catapult launch to allow the vehicle to obtain sufficient speed.