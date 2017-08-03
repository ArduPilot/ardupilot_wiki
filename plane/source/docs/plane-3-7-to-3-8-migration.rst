.. _plane-3-7-to-3-8-migration:

===============================
Migration from Plane 3.7 to 3.8
===============================

This is a guide to migrating from plane 3.7 to plane 3.8. There have
been a large number of configuration changes, so please read the guide
carefully, and make sure you do careful ground tests before your first
flight.

The firmware will try to auto-migrate your parameters where possible,
but it is a complex migration and there is the possibility of errors,
so careful testing is critical.

Change to Servo Range Parameters
================================

One of the biggest changes between plane 3.7 and 3.8 is the change
from RCn_* parameters to SERVOn_* parameters.

In plane 3.7 and before, the range, trim and function of all input and
output channels was controlled by a single set of parameters starting
with RC. For example, the handling of input channel 1 was defined by 4
parameters:

- RC1_MIN
- RC1_MAX
- RC1_TRIM
- RC1_REV


these same parameters also controlled the range of output values for
the first output channel. That means you could not have a different
range for your RC inputs and your servo outputs.

In plane 3.8 the above 4 parameters are replaced with 8 parameters:

- RC1_MIN
- RC1_MAX
- RC1_TRIM
- RC1_REVERSED
- SERVO1_MIN
- SERVO1_MAX
- SERVO1_TRIM
- SERVO1_REVERSED

the RC parameters control an input channel, the SERVO parameters
control an output channel. This gives much more flexibility, and means
that when you re-calibrate your RC inputs it doesn't affect the tuning
of your aircraft.

You may also notice that the "REV" parameters have been renamed to
"REVERSED". This is because the old parameters had a convention like
that "reversed" was chosen by setting RC1_REV to -1, and "not
reversed" was chosen by setting RC1_REV to 1. This caused quite a lot
of confusion, so for 3.8 we now use the convention that RC1_REVERSED=0
means not reversed and RC1_REVERSED=1 means reversed.

Change to Servo Functions
=========================

The second big change is to the parameters that defines what each
output is used for. In plane 3.7 the output function of the first 4
channels was hard coded, following the AETR "aileron, elevator,
throttle, rudder" convention. Channels 5 and above had FUNCTION
parameters, allowing their function to be set.

For example, in plane 3.7 you could set RC5_FUNCTION to 4 to make
output channel 5 an additional aileron.

In plane 3.8 the functions are chosen using the SERVOn_FUNCTION
parameters. You can set these on any output channel. The default for
the first 4 channels is the same, but you can change it to any output
type you like. This gives a lot more flexibility, and is especially
useful for flight boards with a smaller number of output channels.

So if you wanted the 5th output channel to be an aileron you could
set SERVO5_FUNCTION=4.

You can see more details on output functions in the :ref:`Output Functions <channel-output-functions>` documentation.

Change to Elevon, Vtail and Flaperon Configuration
==================================================

The method of setting up elevon, vtail and flaperon planes has
completely changed in plane 3.8.

In the 3.7 release if you wanted to setup an elevon plane you would
set the variable ELEVON_OUTPUT to an integer from 1 to 8. This then
changed the behaviour of the first aileron and elevator channel to do
elevon mixing. The value of ELEVON_OUTPUT controlled the possible
channel order and reversal options (8 combinations are possible).

In plane 3.8 a much simpler and more flexible system is used. If you
want channel 1 to be a left elevon and channel 2 to be a right elevon
then you use:

- SERVO1_FUNCTION=77
- SERVO2_FUNCTION=78

the value 77 means "left elevon". The value 78 means "right
elevon". You can then set the reversals as needed, using
SERVO1_REVERSED and SERVO2_REVERSED.

Changing to this system means you can have elevons on any channel (and
multiple elevons if needed), plus you can directly control the trim
and range of each elevon,

The same system is used for vtail, flaperon and differential spoiler
setups. Please see the following guides for more details:

- :ref:`Elevon Planes <guide-elevon-plane>`
- :ref:`VTail Planes <guide-vtail-plane>`
- :ref:`Differential Spoilers <differential-spoilers>`
- :ref:`Flaperons <flaperons-on-plane>`

  
Tuning Changes
==============

Some users may notice the tuning of their aircraft changes a bit
between 3.7 and 3.8. The likely cause of this is the change in the RC
and SERVO parameters. With the separation of the RCn_MIN/MAX
parameters from the SERVOn_MIN/MAX parameters you may find that the
"throw" of your servos changes a bit, which can affect tuning.

While the automatic upgrade when you install plane 3.8 onto a plane
running 3.7 should produce a very similar servo range, please do check
it carefully.

Reversal Checking
=================

Please do very careful checking for servo reversal before your first
flight with plane 3.8. The automatic parameter upgrade when you
install plane 3.8 should get this right, but it is very complex and
there may be some situations that aren't covered. Please make sure
that both of the following are checked carefully after you update:

 - that RC inputs give the correct movements of your control surfaces
   in MANUAL mode
 - that the control surfaces move in the right direction in FBWA mode
   when you roll and pitch the aircraft with no RC input

There is a detailed guide to checking for reversals in the :ref:`four channel plane <guide-four-channel-plane>` document.

