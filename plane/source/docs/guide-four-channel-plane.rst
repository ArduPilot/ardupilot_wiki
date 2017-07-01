.. _guide-four-channel-plane:

===================
Four Channel Planes
===================

The most common sort of fixed wing plane is a 4 channel plane. As the
name implies, it has 4 output servos, most commonly in this order:

- servo output 1 is aileron
- servo output 2 is elevator
- servo output 3 is throttle
- servo output 4 is rudder

These are the default outputs for APM:Plane as it is such a common
setup. It is commonly referred to as an AETR setup.

.. warning:: You should remove the propeller from your aircraft before
             starting your setup.

Setting Up Your Plane
=====================

After you have setup your :ref:`RC inputs <rc-throw-trim>`, the next
step is to setup your 4 outputs.

You can connect your 4 servo cables to any output of your autopilot,
although using the defaults listed above is recommended.

Next check that the SERVOn_FUNCTION values are correct. The following
table shows the right settings for the default AETR output ordering.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th><th>Meaning</th></tr>
   <tr><td>SERVO1_FUNCTION</td><td>4</td><td>aileron</td></tr>
   <tr><td>SERVO2_FUNCTION</td><td>19</td><td>elevator</td></tr>
   <tr><td>SERVO3_FUNCTION</td><td>70</td><td>throttle</td></tr>
   <tr><td>SERVO4_FUNCTION</td><td>21</td><td>rudder</td></tr>
   </table>

Servo Reversal
==============

The next step is to get the reversals right. You should connect the
battery (with propeller removed) and turn on your RC transmitter. Now
switch to MANUAL mode and disable the safety switch (if fitted).

At this point your RC transmitter should have control of your 3
control surfaces (aileron, elevator and rudder). You should now adjust
the reversal of the 3 outputs so that in MANUAL mode the surfaces move
in the right direction.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Input</th><th>Action</th></tr>
   <tr><td>Right Roll</td><td>Left aileron goes down and right aileron goes up</td><tr>
   <tr><td>Left Roll</td><td>Right aileron goes down and left aileron goes up</td><tr>
   <tr><td>Pull back on pitch</td><td>Elevator goes up</td></tr>
   <tr><td>Push forward on pitch</td><td>Elevator goes down</td></tr>
   <tr><td>Right Yaw</td><td>Rudder goes right</td></tr>
   <tr><td>Left Yaw</td><td>Rudder goes left</td></tr>
   </table>

If any of the directions are incorrect then you need to change the
corresponding SERVOn_REVERSED setting. So for example if your ailerons
move the wrong way then you should change SERVO1_REVERSED to 1.

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
   <tr><td>Roll plane right</td><td>Right aileron goes down and left aileron goes up</td><tr>
   <tr><td>Roll plane left</td><td>Left aileron goes down and right aileron goes up</td><tr>
   <tr><td>Pitch nose up</td><td>Elevator goes down</td><tr>
   <tr><td>Pitch nose down</td><td>Elevator goes up</td><tr>
   </table>

Finally you should double check rudder direction. This one is
particularly easy to get wrong. First check that ground steering is
disabled (by checking that GROUND_STEER_ALT is zero) and that you have
a non-zero rudder gain in KFF_RDDRMIX.

Now check the following:

.. raw:: html
         
   <table border="1" class="docutils">
   <tr><th>Movement</th><th>Action</th></tr>
   <tr><td>Roll plane right</td><td>Rudder goes left</td><tr>
   <tr><td>Roll plane left</td><td>Rudder goes right</td><tr>
   </table>

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
of the leading edge of the control surface close to the fuselage. If
your aircraft doesn't come with any suggested throw values then choose
a throw that doesn't cause your servos to "bind" (which is indicated
by a high pitched sound when your servos move too far).

To adjust the throw, change the SERVOn_MIN and SERVOn_MAX values. The
defaults are 1100 to 1900. On many aircraft you will want more throw
than that, and can change to a throw of 1000 to 2000.

Final Setup
===========

After completing the above you should move onto the final setup of
your aircraft.

- :ref:`ESC Calibration <guide-esc-calibration>`
- :ref:`Center of Gravity <guide-center-of-gravity>`
