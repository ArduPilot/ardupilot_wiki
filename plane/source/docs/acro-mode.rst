.. _acro-mode:

=========
ACRO Mode
=========

ACRO (for acrobatic) is a mode for advanced users that provides rate
based stabilization with optional attitude lock. It is a good choice for people
who want to push their plane harder than you can in :ref:`FLY BY WIRE A (FBWA) <fbwa-mode>` or :ref:`STABILIZE <stabilize-mode>` mode without
flying in :ref:`MANUAL <manual-mode>`. This is the mode to use for rolls,
loops and other basic aerobatic maneuvers, or if you just want an "on
rails" manual flying mode.

.. note:: rate stabilization is not enabled by default for the YAW axis. Set :ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>` = 1 to enable yaw axis rate stabilization. Be careful since this will prevent any turns by aileron alone and require application of rudder also to turn. Also, do not enable this if the plane has no yaw control.

To use this mode you need to set up :ref:`ACRO_YAW_RATE<ACRO_YAW_RATE>` (if using yaw rate controller), :ref:`ACRO_ROLL_RATE <ACRO_ROLL_RATE>`
and :ref:`ACRO_PITCH_RATE <ACRO_PITCH_RATE>`. These default to 180 degrees/second (and 0, ie no limit, for yaw.However, for AUTOTUNE on yaw axis to work, it must be set to a non-zero value. 90 degrees/second is suggested), and control how responsive your
plane will be about each axis. It is also necessary to have the plane tuned well (see :ref:`tuning-quickstart`)

When flying in ACRO the aircraft will resist changes to its existing attitude
if you have no stick input. So if you roll the plane to a 30 degree bank
angle with 10 degrees pitch and then let go of the sticks, the plane
should hold that attitude short term. This applies upside down as well, so if you
roll the plane upside down and let go of the sticks the plane will try
to hold the inverted attitude short term or until you move the sticks again.

.. note:: the internal controllers will resist attitude changes, but drift due to turbulence or miss-trimming will result in gradual attitude changes. See ACRO MODE ATTITUDE LOCKING section below.

When you apply aileron or elevator stick the plane will rotate about
that axis (in body frame) at a rate proportional to the amount of stick
movement. So if you apply half deflection on the aileron stick then the
plane will start rolling at half of ``ACRO_ROLL_RATE``.

.. note:: the :ref:`MAN_EXPO_ROLL<MAN_EXPO_ROLL>`, :ref:`MAN_EXPO_PITCH<MAN_EXPO_PITCH>`, and :ref:`MAN_EXPO_RUDDER<MAN_EXPO_RUDDER>` parameters will apply exponential to the stick inputs, if non-zero, in this mode. This is for users with transmitters which do not provide this function and desire to "soften" stick feel around neutral.

So to perform a simple horizontal roll, just start in level flight then
hold the aileron stick hard over while leaving the elevator stick alone.
The plane will apply elevator correction to try to resist pitch changes while
rolling, including applying inverse elevator while inverted. But to exactly hold the
pitch attitude during multiple rolls without drift, :ref:`ACRO_LOCKING<ACRO_LOCKING>` must be enabled.

Performing a loop is just as simple - just start with wings level then
pull back on the elevator stick while leaving the aileron alone. The
controller will try to hold your roll attitude through the loop. You can
stop the loop upside down if you like as part of maneuvers such as
Immelman turns or cuban eights.

Note that if you are using ACRO mode to try and teach yourself aerobatic
flying then it is highly recommended that you setup a
:ref:`geo-fence <geofencing>` in case you get disoriented.

.. warning::

   It is very easy to stall your plane in ACRO mode, and if you
   stall you should change to MANUAL mode to recover.

-  make sure you know the limitations of your airframe, and what the
   correct stall recovery procedure is. This varies a lot between
   airframes. Search for stall recovery tutorials for R/C aircraft and
   read them
-  don't overload your airframe, only fly ACRO mode with a plane capable of surviving full control surface deflections at any speed.
-  make sure you have enough airspeed for whatever maneuver you are
   attempting. Throttle and speed control is completely under manual
   pilot control in ACRO mode
-  practice stall recovery before trying anything too fancy. Make sure
   you practice when you have plenty of altitude to give you time to try
   different recovery strategies

It can be a lot of fun flying ACRO mode, but you can also easily stall
and crash hard. Automatic stall detection and recovery in autopilots is
an area of research, and is not yet implemented in Plane, so if you do
stall then recovery is up to you. The best mode for recovery is MANUAL.

ACRO MODE ATTITUDE LOCKING
==========================

By enabling the :ref:`ACRO_LOCKING<ACRO_LOCKING>` parameter, whatever attitude (roll and pitch angle) the pilot places the plane in, upon releasing the sticks, the autopilot will not only resist rate changes (caused by trim or turbulence), but also attempt to hold and correct back to that attitude. Note that his requires that the plane be properly tuned (see :ref:`Tuning<common-tuning>` ).

It is recommended that it be set to "2", instead of "1", in order to use a quarternion based control system with much better performance than the older system. In order for this to be effective, yaw rate control (:ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>`) must be "1" and the yaw rate controller tuned using :ref:`Autotune <automatic-tuning-with-autotune>` for best performance.

ACRO Mode YAW Rate Control
==========================

As of version 4.2, ArduPilot provides the option for utilization of a rate controller for YAW, which behaves in the same manner as the pitch and roll controllers, explained above, but for the YAW axis controlled by the Rudder stick, assuming the vehicle has a rudder control surface.

To enable this functionality, set :ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>` to 1. When enabled, the :ref:`ACRO_YAW_RATE<ACRO_YAW_RATE>` parameter can be used to adjust maximum yaw rate demanded at rudder stick full deflections in ACRO mode.

Before use, the controller should be tuned, either manually or using AutoTune. See :ref:`automatic-tuning-with-autotune` or the YAW tuning section of the :ref:`Manual Tuning page<new-roll-and-pitch-tuning>`.

.. note:: using this controller will give the feel of a 'heading hold' yaw axis. While not exactly "heading" holding, it does resist any yaw rate change not commanded by the pilot. This means the pilot will need to "fly the tail" in turns. Just banking will not generate a clean turn.
