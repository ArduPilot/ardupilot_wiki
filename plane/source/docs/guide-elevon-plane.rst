.. _guide-elevon-plane:

=============
Elevon Planes
=============

Elevon planes (also known as delta-wings) are popular for their
simplicity and robustness.

A typical elevon plane will have 2 servo outputs and one throttle
output.

Setting Up Your Plane
=====================

The recommended setup for APM:Plane with elevon planes is:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th><th>Meaning</th></tr>
   <tr><td>SERVO1_FUNCTION</td><td>77</td><td>Left elevon</td></tr>
   <tr><td>SERVO2_FUNCTION</td><td>78</td><td>Right elevon</td></tr>
   <tr><td>SERVO3_FUNCTION</td><td>70</td><td>Throttle</td></tr>
   </table>

.. warning:: You should remove the propeller from your aircraft before
             starting your setup.

RC Input Setup & Reversal
=========================

Set up your :ref:`RC inputs <rc-throw-trim>` through the calibration
process, and verify them for reversal. Reversals are critical to the
process. Commanding pitch up and roll right must result in higher PWM
values for RCn_in channnels. If the value does not respond correctly
reverse the channel before continuing.

You can connect your 3 servo cables to any output of your autopilot,
although using the defaults listed above is recommended.

Servo Setup & Reversal
======================

The next step is to get the servo reversals right. You should connect the
battery (with propeller removed) and turn on your RC transmitter. Now
switch to FBWA mode and press the safety switch (if fitted) to enable
servo outputs.

At this point both the autopilot and RC transmitter should have control
of the elevons. You now should adjust the reversal and function of the two 
servos so that you get correct movement.

Correct FBWA (automatic stabilization)movement for an elevon plane WITHOUT 
PROVIDING RC INPUT is:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Input</th><th>Action</th></tr>
   <tr><td>Roll right</td><td>Left elevon goes up and right elevon goes down</td><tr>
   <tr><td>Roll left</td><td>Right elevon goes up and left elevon goes down</td><tr>
   <tr><td>Pitch down</td><td>Both elevons go up</td></tr>
   <tr><td>Pitch up</td><td>Both elevons go down</td></tr>
   </table>

If your movements are incorrect then you need to adjust which servo
output is left/right and the reversals of each elevon.

The parameters you should adjust are SERVO1_REVERSED, SERVO2_REVERSED,
SERVO1_FUNCTION and SERVO2_FUNCTION.

If your left elevon on servo 1 is moving the wrong way for both pitch and
roll corrections, set SERVO1_REVERSED to 1.

If your left elevon on servo 1 responds correctly to pitch, but incorrectly
to roll, change the SERVO1_FUNCTION.

Repeat the servo reversal or function change for the right elevon.

.. note:: In rare instances, both servo 1 and 2 will indivicually respond
          correctly with the same FUNCTION. This is OK.
          
.. note:: while rolling the aircraft the autopilot will automatically
          try to put in some up pitch, as it knows that upward pitch is needed
          in turns. So you will probably see an asymmetry in elevon
          movement. The elevon that is going down will not go down very far, or
          (depending on your settings) may not go down at all.
          

Verify RC Inputs
================

Now that the elevons are configured correctly, verify your RC inputs.
In FBWA with the airplane level, command pitch-up from your transmitter
and confirm that the elevons both rise. Command a roll to the right
from your transmitter and confirm that the the right elevon rises and
the left elevon lowers. If this is incorrect, read the :ref:`RC inputs <rc-throw-trim>` 
page to fix your rc

Switch the plane to MANUAL mode and confirm the same behavior.

Servo Trim
==========

Now stay in MANUAL mode in order to adjust the servo trim
values. The servo trim is in the SERVOn_TRIM parameters.

You should adjust the trim values so that the servo is centered when
your transmitter sticks are centered. If you find you need to adjust
the trim value by more than 50 PWM from the default of 1500 then it is
recommended that you instead adjust the trim mechanically.

Servo Throw
===========

Finally you should adjust your servo throw. The throw is the range of
movement for each of your servos.

Check any instructions that came with your plane for suggested throw
values. These are often specified in millimeters or inches of movement
of the edge of the control surface. If your aircraft doesn't come with
any suggested throw values then choose a throw that doesn't cause your
servos to "bind" (which is indicated by a high pitched sound when your
servos move too far).

To adjust the throw, change the SERVOn_MIN and SERVOn_MAX values. The
defaults are 1100 to 1900. On many aircraft you will want more throw
than that, and can change to a throw of 1000 to 2000.

Mixing Gain
===========

The MIXING_GAIN parameter is critical for elevon aircraft. It is the
gain used in mixing between roll and pitch output and your elevon
movement.

For example, if your MIXING_GAIN is 0.5, then the following outputs
are used:

- LEFT_ELEVON = (roll+pitch)*0.5
- RIGHT_ELEVON = (roll-pitch)*0.5

By adjusting the MIXING_GAIN you can quickly setup the right throws of
your elevon aircraft.
  
Final Setup
===========

After completing the above you should move onto the final setup of
your aircraft.

- :ref:`ESC Calibration <guide-esc-calibration>`
- :ref:`Center of Gravity <guide-center-of-gravity>`
