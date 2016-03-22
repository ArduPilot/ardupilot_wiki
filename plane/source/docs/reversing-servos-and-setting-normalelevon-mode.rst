.. _reversing-servos-and-setting-normalelevon-mode:

===============================================
Normal/Elevon/VTail Mode &amp; Reversing Servos
===============================================

Every aircraft is different, and as people familiar with RC know, you've
got to tell your RC equipment which way the servos go and how that moves
the control surfaces, a process that involves "reversing" channels if
needed. The same goes for an autopilot. This section will walk you
through this process.

Overview
========

**First**, make sure your control surfaces are going the right way in
Manual Mode.  In this mode, the RC controls are sent straight through to
the servos, without going through the autopilot (this is the
safety/failsafe mode that allows you to regain control even in the case
of autopilot failure). If any control surface is going in the wrong
direction when you move your RC sticks, use your transmitter's channel
reverse function to reverse it.

**Second**, switch into FBWA mode and do the same thing, while connected
to the Mission Planner. You're under autopilot control now, and you need
to configure it for your particular setup. As you move the RC sticks
again, watch the control surfaces. If any is going in the wrong
direction, click the Reverse checkbox in the RC setup screen shown
below.

Do this for all four channels shown. **Just because it's right in
RC/Manual doesn't mean it will be right under Autopilot control. Check
it in both modes!**

.. image:: ../../../images/mpreverse.png
    :target: ../_images/mpreverse.png

Getting the rudder direction right
==================================

A very common mistake is to have the rudder reversal set incorrectly. If
you have it set incorrectly then your plane can fly very badly, and may
not be able to navigate at all.

To check your rudder reversal you need to do the following:

Ensure the KFF_RDDRMIX parameter is set to a non-zero value. For this
test you should set it to a high value (such as 0.8) to ensure the
rudder movement is large. Remember to reset it back to a lower value
afterwards.

-  put the plane into FBWA mode
-  with no stick input (hands off the transmitter) roll the aircraft to
   the right. The rudder should turn towards the left as it tries to
   correct the roll.
-  Now roll the aircraft to the left. The rudder should turn to the
   right as it tries to correct the roll.

If the rudder moves in the wrong direction you should change the
RC4_REV parameter. A value of 1 means no reversal. A value of -1 means
to reverse the rudder.

Standard (non-elevon) reversal setup
====================================

For non-elevon setups (ie. setups with ELEVON_MIXING set to 0 and
ELEVON_OUTPUT set to 0), you have 4 parameters that control the servo
reversals, one for each channel you can reverse.

The 4 parameters are:

+------------+---------------------+--------------------------+---------------------------------------+
| RC1_REV    | aileron reversal    | set to -1 for reversal   | defaults to 1 (meaning no reversal)   |
+------------+---------------------+--------------------------+---------------------------------------+
| RC2_REV    | elevator reversal   | set to -1 for reversal   | defaults to 1 (meaning no reversal)   |
+------------+---------------------+--------------------------+---------------------------------------+
| RC3_REV    | throttle reversal   | set to -1 for reversal   | (for some gas planes)                 |
+------------+---------------------+--------------------------+---------------------------------------+
| RC4_REV    | rudder reversal     | set to -1 for reversal   | defaults to 1 (meaning no reversal)   |
+------------+---------------------+--------------------------+---------------------------------------+

New style Elevon mixing setup (ELEVON_OUTPUT option)
=====================================================

As of Plane 2.73 there is a new ELEVON_OUTPUT option. This option
allows you to setup your transmitter for normal aileron/elevator control
and for Plane to add elevon mixing on the output of channels 1 and 2.
Using ELEVON_OUTPUT has a big advantage over ELEVON_MIXING that your
inputs won't saturate the roll/pitch control in FBWA mode, which means
you can get better control of your plane.

Using this setup method it doesn't matter which channel you have plugged
into which aileron.

Note that you cannot use the ELEVON_OUTPUT option on an APM1 board if
you have the flight mode channel (FLTMODE_CH) set to 8, as on the APM1
this will lead to hardware pass through of controls when in manual,
which means your transmitter must be setup for elevon mixing. If using
an APM2 or PX4 this is not a problem, and the ELEVON_OUTPUT option is
recommended.

The ELEVON_OUTPUT option is designed to operate exactly as a hardware
elevon mixer would operate. To set it up follow these steps:

-  Setup your transmitter with no elevon mixing
-  Set both RC1_REV and RC2_REV to 1 and ELEVON_MIXING to 0
-  Start by setting ELEVON_OUTPUT to 1. In later steps you may adjust
   this to 2, 3 or 4.
-  Put your APM into FBWA mode
-  Roll the plane to the right and observe what happens to the elevons
-  If the two elevons move in the same direction, then change
   ELEVON_OUTPUT to 2, and try again.
-  If the elevons move in opposite directions, but the APM is correcting
   the roll in the wrong direction (the left elevon is going down and
   the right one is going up) then change RC1_REV to -1
-  Next try pitching your plane up. If the elevons move in the wrong
   direction then change RC2_REV to -1
-  Now change to MANUAL mode and adjust your transmitter reversals for
   channels 1 and 2 to produce the right movement in MANUAL mode.

Please make sure that you do careful ground testing after setting these
parameters!

Please also see the note about the MIXING_GAIN parameter below.

Old style elevon mixing setup (ELEVON_MIXING option)
=====================================================

Note: We STRONGLY suggest you use the new style mixing above rather then
this old style.

For old elevon based setups where you have set ELEVON_MIXING to 1, you
have 3 different parameters to setup. They are:

+------------------------+------------------------------------------+--------------------------------------+
| ELEVON_REVERSE         | reverse the sense of the elevon mixing   | set to 1 to reverse, defaults to 0   |
+------------------------+------------------------------------------+--------------------------------------+
| ELEVON_CH1_REVERSE     | reverse channel 1 elevon                 | set to 1 to reverse, defaults to 0   |
+------------------------+------------------------------------------+--------------------------------------+
| ELEVON_CH2_REVERSE     | reverse channel 2 elevon                 | set to 1 to reverse, defaults to 0   |
+------------------------+------------------------------------------+--------------------------------------+

To select elevon mode or reverse elevon channels, use the elevon
checkboxes at the bottom:

Roll to the right illustrated below.

.. image:: ../../../images/mavelevon1.png
    :target: ../_images/mavelevon1.png

It takes a little trial-and-error to set up elevons on any particular
aircraft, but here are the basic steps:

#. First, set it up in manual mode by setting up elevon mixing on your
   RC transmitter. It matters which elevon is plugged into which
   channel! **Shown above, the left wing aileron is plugged into Ch1 and
   the right wing into Ch2.**
#. Still in manual mode, check to see if you have to reverse any
   channels on your RC transmitter to ensure the control surfaces move
   the way they should in both pitch and roll.
#. Now that it's working in manual, connect to your APM board with the
   Mission Planner. Go through the regular setup process. When
   calibrating your RC input, \ **don't just move the elevator and
   aileron sticks to the normal up down, left right positions. Instead,
   you must move the stick to the CORNERS** or the calibration will be
   wrong and the servos will try to move too far. This is because now
   that you've switched your RC transmitter into elevon mode, the
   elevator and airelon inputs are added when the stick is in the corner
   (full left and full up as an example).
#. While still in the MP RC setup screen, switch into FBWA Mode. Move
   move the plane around to test and watch the control surfaces. When
   you tip the nose of the plane down, the two elevons should go up and
   vice versa. Likewise with roll; when you roll the plane, the elevons
   should move to counteract that and return the plane to level. You'll
   probably have to reverse something with the check boxes on that
   screen for correct motion. Just change one thing at a time!
#. If you just can't seem to find the right combination that works, try
   swapping your servo cables, so that Right is in Output 1 and Left is
   Output 2. This is something of a last resort, because you'll have to
   start the setup from the top of this list again.

Please make sure that you do careful ground testing after setting these
parameters. Also remember that your RC transmitter must be set up to do
elevon mixing, too!

.. note::

   It is possible to configure differential spoilers with old style
   elevon mixing, although the feature is not widely used and not well
   tested. Differential spoilers cannot currently be configured with the
   new type elevon mixing.

Setting up a VTAIL plane
========================

To setup a VTAIL plane, you can enable a software VTAIL mixer using the
VTAIL_OUTPUT option. The VTAIL_OUTPUT option works the same way as the
ELEVON_OUTPUT option, except that it operates on the elevator and
rudder output channels (channels 2 and 4).

Note that you cannot use the VTAIL_OUTPUT option on an APM1 board if
you have the flight mode channel (FLTMODE_CH) set to 8, as on the APM1
this will lead to hardware pass through of controls when in manual,
which means your transmitter must be setup for vtail mixing. If using an
APM2 or PX4 this is not a problem, and the VTAIL_OUTPUT option is
recommended for vtail planes. On an APM1 use a hardware vtail mixer
instead.

The VTAIL_OUTPUT option is designed to operate exactly as a hardware
vtail mixer would operate. To set it up follow these steps:

-  Setup your transmitter with no vtail mixing
-  Set both RC2_REV and RC4_REV to 1 and KFF_RDDRMIX to 0.5
-  Start by setting VTAIL_OUTPUT to 1. In later steps you may adjust
   this to 2, 3 or 4.
-  Put your APM into FBWA mode
-  Pitch up the nose of the plane observe what happens to the vtail
-  If the two vtail segments move in opposite directions, then change
   VTAIL_OUTPUT to 2, and try again.
-  If the two vtail segments move in the same direction, but the APM is
   correcting the pitch in the wrong direction (both segments are moving
   up) then change RC2_REV to -1
-  Next try rolling your plane to the right. The two vtail segments
   should move to try to turn the plane left (to correct for the right
   roll). If they move in the wrong direction then set RC4_REV to -1
-  Now change to MANUAL mode and adjust your transmitter reversals for
   channels 2 and 4 to produce the right movement in MANUAL mode.
-  Finally adjust the KFF_RDDRMIX to a value that gives the right
   amount of rudder movement for coordinated turns on your plane. This
   may require some inflight tuning. A initial guess of around 0.5 is
   likely to work for most planes.

Here's a V-Tail movement diagram courtesy of \ *Miami Mike*:

.. image:: ../../../images/v-tail-300x200.gif
    :target: ../_images/v-tail-300x200.gif

Please make sure that you do careful ground testing after setting these
parameters!

Please also see the note about the MIXING_GAIN parameter below.

Using MIXING_GAIN to control mixing throws
===========================================

If you use the ELEVON_OUTPUT or VTAIL_OUTPUT options, you may find the
MIXING_GAIN parameter useful to control the gain of the mixer.

The default is a gain of 0.5, which ensures that over the full range of
the mixer both inputs have authority (it can't saturate). That also
means that if you have one input of the mixer (eg. aileron on an elevon
plane) at full range, and the other input neutral, then the output is
only 1750. That may not be enough roll authority for some planes.

If you change the MIXING_GAIN to 1.0 then you will get the full range
of output from a single channel, although if you have full aileron
deflection and full elevator at the same time you will saturate the
mixer. It will clip output outside of the valid range of 900 to 2100
microseconds.

So if you have found ELEVON_OUTPUT doesn't have enough authority then
try raising the MIXING_GAIN.

Important notes
===============

-  Whenever you change your firmware your parameter (EEPROM) settings
   will revert to the defaults if the new firmware has an incompatible
   parameter (EEPROM) format. The release notes for a release will
   contain a note if this happens. The developers are careful to try to
   minimise the number of times this is needed. Please use the APM
   mission planner or your ground control station to save your settings,
   and \ **carefully check them after any firmware change**.

-  make sure you \ **always do ground tests** before every flight to
   ensure your channel mixing and reversals are all correct. Be careful
   to check that not only are your transmitter controls correct, but
   that the APM responds correctly to attitude changes in the plane when
   in FBWA mode.
