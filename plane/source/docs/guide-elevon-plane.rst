.. _guide-elevon-plane:

=============
Elevon Planes
=============

Elevon planes (also known as delta-wings) are popular for their
simplicity and robustness.

A typical elevon plane will have 3 servo outputs. One will be the left
elevon, another for right elevon and the 3rd for throttle.

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

Setting Up Your Plane
=====================

After you have setup your :ref:`RC inputs <rc-throw-trim>`, the next
step is to setup your 3 outputs.

You can connect your 3 servo cables to any output of your autopilot,
although using the defaults listed above is recommended.

Servo Reversal
==============

The next step is to get the reversals right. You should connect the
battery (with propeller removed) and turn on your RC transmitter. Now
switch to MANUAL mode and disable the output safety (if enabled).

At this point your RC transmitter should have control of your 2
elevons. You now should adjust the reversal of the two elevons and the
order of the two elevons so that you get correct movement.

Correct movement for an elevon plane is:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Input</th><th>Action</th></tr>
   <tr><td>Right Roll</td><td>Left elevon goes down and right elevon goes up</td><tr>
   <tr><td>Left Roll</td><td>Right elevon goes down and left elevon goes up</td><tr>
   <tr><td>Pull back on pitch</td><td>Both elevons go up</td></tr>
   <tr><td>Push forward on pitch</td><td>Both elevons go down</td></tr>
   </table>

If your movements are incorrect (which they probably will be!) then
you need to adjust which servo output is left/right and the reversals
of each elevon.

The parameters you should adjust are SERVO1_REVERSED, SERVO2_REVERSED,
SERVO1_FUNCTION and SERVO2_FUNCTION.

For example, if your left elevon on servo 1 is moving the wrong way,
and SERVO1_REVERSED is currently 0, then changing it to 1 will change
the direction it moves.

Confirm Servo Reversal
======================

The above servo reversal test in MANUAL mode assumes your RC inputs
have been correctly setup. As it is so easy to get that wrong, you
should also do a stabilisation check.

Switch the plane to FBWA mode and with the transmitter sticks centered
move the plane as follows:

.. raw:: html
         
   <table border="1" class="docutils">
   <tr><th>Movement</th><th>Action</th></tr>
   <tr><td>Pitch nose up</td><td>Both elevons go down</td><tr>
   <tr><td>Pitch nose down</td><td>Both elevons go up</td><tr>
   <tr><td>Roll plane right</td><td>Right elevon goes down and left elevon goes up</td><tr>
   <tr><td>Roll plane left</td><td>Left elevon goes down and right elevon goes up</td><tr>
   </table>

Note that while rolling the aircraft the autopilot will automatically
try to put in some up pitch, as it knows that upward pitch is needed
in turns. So you will probably see an asymmetry in elevon
movement. The elevon that is going down will not go down very far, or
(depending on your settings) may not go down at all.

Servo Trim
==========

Now switch back to MANUAL mode in order to adjust the servo trim
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
