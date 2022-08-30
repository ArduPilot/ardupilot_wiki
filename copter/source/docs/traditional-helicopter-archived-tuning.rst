.. _traditional-helicopter-archived-tuning:

=====================================================================
Archived: Traditional Helicopter – Tuning with Copter 3.3 and earlier
=====================================================================

PID Tuning
==========

Now, we come to the PID tuning. This part is a bit trickier. The first
tip is that whenever you go into the Copter PID configuration screen;
make sure that Lock Pitch/Roll is unchecked. Helis need separate
pitch/roll values because the moment of inertia of the airframe is very
different in the two different axes. This is because the tail boom is so
long and there's a lot more mass in the fore/aft axis, so it's slower.

On the Copter Config tab, the STB_RLL_P value should be the default of
4.5 and STB_PIT_P value should be the default value of 3.5. This
softens the pitch axis a bit compared to the roll axis and helps prevent
bouncing. The I-term on both should be zero.

Now, the Rate part is a little trickier.

You will need to change the RATE_PIT_P and RATE_RLL_P to get your
heli to fly properly but to start with, set these to zero. They should
be zero by default.

Something to keep in mind here, the Rate PID numbers you come up with
will be heavily influenced by the following: # Your swash servo speed #
Your swash servo motion ratio i.e. servo arm length, swash plate
dimensions and blade grip arm length # H_PIT_MAX and H_ROL_MAX
values.

Any time you change these you'll probably have to retune your Rate PID's
a bit. We expect that with faster servos you'll be able to tune Rate P
up and get better control without oscillation. Start with RATE_RLL_I
of 0.1000 (default) and RATE_PIT_I of 0.0500 (default is 0.1000).

Now set Ch6 tuning in the Copter Config tab to CH6_RATE_KP and set the
Min to 0.0100 and the Max to 0.0750 and make sure you turn your Ch6 knob
all the way down. Refresh parameters and make sure that RATE_PIT_P and
RATE_RLL_P show up as 0.0100 so that you know the Ch6 tuning is
working properly.

Here's where it gets tricky and we really recommend you do this in a
large area with no wind. Maybe indoors in a 20m by 20m area, but only if
you're a good pilot.

Spool up your heli **VERY CAREFULLY**. Don't just take off because you
will have very little control over the heli! It might be best to do this
with training gear if you can. Lift off just one inch but be ready to
put it down because you will have little or no control. It's going to be
bad, so be ready for it! You should have no oscillation with a P-term of
0.0100.

If you do, you have big problems and will need to consult the
specialists on DIYDrones!

Now try turning the knob up a bit, maybe to 0.0200. Try just lifting off
remembering that you will still have little control but hopefully you
will have no oscillation. Keep doing this, testing more and more P-term
until you get oscillation. Be careful with that P-term because once you
find the point where it oscillates, it gets bad REALLY fast and you
could destroy your helicopter! You will find that the oscillation will
always happen in the roll direction first. This is because of its lower
moment of inertia.

You should get to a RATE_RLL_P of about 0.0400 (on a 450 size heli) or
maybe even a little bit more but hopefully you end up in this range.
However, you will find that you still have very little control, even up
to the point where it starts to oscillate!

Once you find the oscillation point, back off a bit, maybe 0.005 to
0.010. Now, set the Ch6 tuning to "None", and make sure you have a good
Rate P-term in the RATE_RLL_P and RATE_PIT_P window. If you fly
right at the limit of oscillation you will find that sometimes if you
bump the skids or there's a wind gust, it can start to oscillate. That's
why you want to back off a bit.

Remember that when you get your P value to high the heli will **rock and
roll** something fierce. Be careful and only lift of the ground a few
inches or a foot at most and be ready to put it down if it starts to
oscillate out of control.

Now you have established your RATE_RLL_P value, your RATE_PIT_P
value can be a little higher. For example, use 0.050 with 0.040 for Roll
and remember to unchecked "Lock Pitch/Roll"!

Now you can play with the I-term. You can do so similarly, with Ch6.
It’s important to remember the RATE_PIT_I term will be LOWER than the
RATE_RLL_I. Try using Roll I = 0.250 and Pitch I = 0.150. These
I-values are less critical than the P-values - you're not likely to
destroy your heli with a high value, more likely to just get a "bounce".
So, at this point, you should have a heli that will fly without shaking
but it still flies like it's “drunk”. It'll just wander around with you
chasing it but now it's time to fix that.

Go into Advanced Parameters>Parameters List and find RATE_RLL_FF and
set it to maybe 0.020 to start. Do the same for RATE_PIT_FF. Write
these and then give it a test fly. You should now find that it flies
much better, you will have more control, it should hover better. Play
with these numbers a bit and always remember to “write” the parameters
to the APM. For a 450 size heli try using 0.040 on Roll and 0.050 on
Pitch. It’s not clear how high you can push these but it will start to
oscillate eventually.

Here’s a question that might come up as you begin to fly more: It it’s a
little windy and I do not have enough control over the swash, what can I
do?

Increase STB_RLL_P from, say, 3.8 to 4.5 is probably a step in the
right direction... sort of. It will make the heli try to make bulk moves
more strongly, which is good for fighting winds but maybe not so good
for camera work as it could jerk the camera around. It does indirectly
increase the servo movement.

Yes, increasing your RATE_RLL_P will cause it to shake. That's not a
great solution. Fill your plate with RATE_RLL_FF? This gives you much
more control authority with less chance of shake you get with P-term. It
can still crop up however.

RATE_RLL_I and I_MAX are also important. You should be able to
increase I and get more travel without getting the shakes.

Moving the control link out on the servo horn will have the exact same
effect as increasing the PID terms, so it's not a solution. You'll find
you just have to back off the rate PID's to avoid shake again. And then
you can get into a situation where you don't have enough resolution on
the servo.

One part of this situation is the fact that the Rate I term is not a
classic I-term, but a "leaky" I term. It bleeds down 2% every 100ms...
it's complicated but it's there for a few reasons. One of these reasons
is because if we use a standard I-term, and you leave your heli sitting
on off-level ground, the swash will slowly tip over as the I-term builds
up. If you spool up, the heli will tip over. It's not fun. The problem
with this is it really limits how much control movement we can get out
of the I-term. It's sort of a lesser of two evils. If we could use a
regular I-term, it would help. But we can't... well, we’re working on
something but it will only work if you are using the Ch8 rotor speed
controller so make sure you have this set up properly.

Rudder Tuning
=============

At the same time, think about adding some RATE_YAW_FF. Maybe try using
0.040 here. You might expect to back-off on the RATE_YAW_P just a bit
as RATE_YAW_FF increases. Start with a RATE_YAW_P value of 0.25
(default) and move downward to say P=0.120 or even P=0.100. Try I = 0.02
and D = 0.002

For a 600 size heli the rudder (yaw) settings may look like this:

+-------------------+---------+
| RATE_YAW_D        | 0.004   |
+-------------------+---------+
| RATE_YAW_FF       | 0.05    |
+-------------------+---------+
| RATE_YAW_I        | 0.15    |
+-------------------+---------+
| RATE_YAW_IMAX     | 2000    |
+-------------------+---------+
| RATE_YAW_P        | 0.38    |
+-------------------+---------+

You’re not finished fiddling with this yet but what you will find is
that these settings make the yaw control much stronger then on previous
versions of the software and also help reduce the very small oscillation
it used to have.

Acro Mode Tuning
================

The AXIS_ENABLE parameter should be set to 1 otherwise Acro mode will
be unflyable. AXIS_ENABLE basically turns on a 3-D "heading lock" gyro
mode that works very well.

There are a few other parameters you should know about:

ACRO_BAL_PITCH and ACRO_BAL_ROLL: These create a "virtual dihedral"
which attempts to return the heli to level. This is what Leonard calls
"Acro with training wheels". 200 makes it feel almost like stabilize. If
you fly with it on 50 it feels nice. 0 would obviously turn it off
completely.

ACRO_TRAINER: This is in addition to ACRO_BAL_ROLL and prevents the
heli from rolling past 45°. You can push it a bit beyond but it won't go
too far. If you don’t want it, just turn it off.

If you want to speed up the angular rate, you will want to play with
ACRO_P which defaults to 4.5 which gives you a rate of 202°/s. Set at 6
will give about 270°/s and 8 will give about 360°/s. The gyros have a
full scale of 2,000°/s

Stabilization Mode Tuning
=========================

One of the things you should look at is your STAB_PIT_P and
STAB_RLL_P numbers? These were probably quite low in version 2.7.3,
probably less than 1, and they should now be 3.5 to 4.5. This is a
really big factor in getting the servo movement you are expecting.

Throttle Tuning
===============

THR_MID convert's the pilot's throttle input (0~1000) to the motor
output (130~1000). It has two different scales so that the pilot's 0~500
input maps to the 130~THR_MID value...then another scale so pilot's
501~1000 input is mapped to the THR_MID~1000 output for the motor.

Essentially THR_MID should be the throttle setting for hover.

For a 600 size heli a possible throttle set up could be:

+--------------------+---------+
| THR_ACCEL_D        | 0.001   |
+--------------------+---------+
| THR_ACCEL_I        | 0.6     |
+--------------------+---------+
| THR_ACCEL_IMAX     | 500     |
+--------------------+---------+
| THR_ACCEL_P        | 0.3     |
+--------------------+---------+
| THR_ALT_I          | 0       |
+--------------------+---------+
| THR_ALT_IMAX       | 300     |
+--------------------+---------+
| THR_ALT_P          | 2       |
+--------------------+---------+
| THR_MAX            | 1000    |
+--------------------+---------+
| THR_MID            | 500     |
+--------------------+---------+
| THR_MIN            | 130     |
+--------------------+---------+
| THR_RATE_D         | 0.001   |
+--------------------+---------+
| THR_RATE_I         | 0       |
+--------------------+---------+
| THR_RATE_IMAX      | 300     |
+--------------------+---------+
| THR_RATE_P         | 3       |
+--------------------+---------+

Aerobatics in Acro Mode
=======================

Acro Mode in APM:Copter now supports full acrobatic flight!  The
function is similar to any other acrobatic FBL controller available on
the market.  The performance might not be suitable for helicopter
competition, but it certainly adequate for sport flying.

Many maneuvers have been fully tested, loops, rolls, inverted flight,
etc.  Caution should be exercised if aggressive 3D type maneuvers are
attempted, such as tic-tocs, etc.  While the control algorithms are
fine, it's unproven if the system is able to maintain its orientation
relative the ground.  In other FBL controllers, if this were to happen,
the controller would simply turn off the self-leveling function but the
acrobatic flight can continue as normal.  But with APM:Copter, this
could lead to loss of control even in Acro mode. We will attempt to
remedy this situation in the future so that all maneuvers can be
performed.

ACRO_BAL_PITCH = 50 

ACRO_BAL_ROLL = 50 

ACRO_P = 4.5 

ACRO_TRAINER =
0 

AXIS_ENABLE = 1

ACRO_P is basically the angular rate. 4.5 gives you 202.5°/s. 9 would
give you 405. But nobody has pushed that high.

ACRO_BAL_ROLL is like a faked "dihedral" effect. It makes the copter
return to level gently at center stick. 50 is fairly low, 200 makes it
feel almost like Stabilize.

ACRO_TRAINER is an addition thing that makes the heli not want to roll
past 45°.

AXIS_ENABLE turns the whole angle-lock thing on. Sort of like Futaba
AVCS in all 3 axes. Without it, it's pure rate control and your
experience will be pretty bad.
