.. _guide-vtail-plane:

============
VTail Planes
============

A common alternative to an elevator and rudder is a VTail, or
sometimes an ATail (which is an upside down VTail).

A VTail aircraft has the same functionality as a standard aircraft, 
but it requires special configuration of the servo outputs. Do not use
V-Tail mixing on your transmitter.

Although you can choose custom channels, the typical VTail setup uses 
channel 2 and 4 for the servo outputs:

.. warning:: You should remove the propeller from your aircraft before
             starting your setup.

Setting Up Your Plane
=====================

The most important step to setting up your plane is having the correct inputs, 
outputs, and reversals. Inputs are covered on the :ref:`RC input setup page <rc-throw-trim>`. 
After you have setup your :ref:`RC inputs <rc-throw-trim>`, the next step 
is to setup your outputs.

.. warning:: Make sure the AHRS_ORIENT is set correctly for your autopilot. If it is incorrect, 
             this setup will not work, and the plane may crash upon entry into any stabilize mode.
             
You can connect your servo cables to any output of your autopilot,
although using the defaults listed below is recommended. Set the 
SERVOn_FUNCTIONS to the appropriate values.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th><th>Meaning</th></tr>
   <tr><td>SERVO1_FUNCTION</td><td>4</td><td>aileron</td></tr>
   <tr><td>SERVO2_FUNCTION</td><td>79</td><td>left vtail</td></tr>
   <tr><td>SERVO3_FUNCTION</td><td>70</td><td>throttle</td></tr>
   <tr><td>SERVO4_FUNCTION</td><td>80</td><td>right vtail</td></tr>
   </table>


Servo Function & Reversal
=========================

The next step is to get the V-Tail functions and reversals correct. 
Both the function and reversal go hand-in-hand, so modifying one may 
partially change the behavior of the other. Connect the battery 
(with propeller removed) and turn on the RC transmitter. Switch to
FBWA mode using the function switch or a ground station command and 
disable the safety switch (if fitted).

When the plane is level, the servos should be near their trim values. 
Move the plane and leave the transmitter sticks centered while 
monitoring the control surfaces. This will help to determine if the 
function and reversals are correct. See the table 
for the correct control surface response to the movements. In each 
instance, the plane should move its control surfaces to level itself.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Movement</th><th>Action</th></tr>
   <tr><td>Roll Plane Right</td><td>Left aileron moves up and right aileron goes down</td><tr>
   <tr><td>Roll Plane Left</td><td>Right aileron moves down and left aileron goes up</td><tr>
   <tr><td>Pitch plane up</td><td>Both tail surfaces move down</td></tr>
   <tr><td>Pitch plane down</td><td>Both tail surfaces move up</td></tr>
   <tr><td>Yaw Plane Right</td><td>Both tail surfaces move left</td></tr>
   <tr><td>Yaw Plane Left</td><td>Both tail surfaces move right</td></tr>
   </table>

If the ailerons do not respond correctly, reverse the output by changing 
the corresponding SERVOn_REVERSED setting (from 0 to 1, or from 1 to 0).

If your V-tails do not respond correctly, change parameters using the 
following table to enable the correct behavior. Work on one control surface 
at a time to avoid confusion.

   <table border="1" class="docutils">
   <tr><th>Control Surface Response</th><th>Corrective Action</th></tr>
   <tr><td>Correct for 1 movement (pitch or roll), but not the other</td><td>Change the function (from 79 to 80; or 80 to 79)</td><tr>
   <tr><td>Incorrect for both movements (pitch and roll)</td><td>Change the reversal of that channel</td><tr>
   </table>

.. note:: Your KFF_RDDRMIX mut not be set to 0 for rudder setup. 
          If your plane actually needs 0, then reset it after this setup.


Confirm RC Transmitter Input
============================

Keep the plane level in FBWA mode and command the following inputs:

.. raw:: html
         
   <table border="1" class="docutils">
   <tr><th>Input</th><th>Action</th></tr>
   <tr><td>Roll Right</td><td>Right aileron goes up and left aileron goes down</td><tr>
   <tr><td>Roll Left</td><td>Left aileron goes up and right aileron goes down</td><tr>
   <tr><td>Pitch up</td><td>Both tail surfaces go up</td><tr>
   <tr><td>Pitch down</td><td>Both tail surfaces go down</td><tr>
   <tr><td>Yaw right</td><td>Both tail surfaces go right</td><tr>
   <tr><td>Yaw left</td><td>Both tail surfaces go left</td><tr>
   </table>

Double check MANUAL mode for the inputs as well. If everything is setup correctly, 
your plane should be almost ready to fly.
   
ATail Planes
============

If you have an "A-Tail" plane (an inverted V-Tail) the control surface movements 
referenced above should still be the same directions. It is likely that your servo 
reversal or function will be opposite from a similar V-Tail setup.

Servo Trim
==========

Switch back to MANUAL mode in order to adjust the servo trim
values. The servo trim is in the SERVOn_TRIM parameters.

Adjust the trim values so that the servo is centered when
your transmitter sticks are centered. If the trim value is not 
between 1450 and 1550 PWM, then it is recommended that you 
instead adjust the trim mechanically.

Servo Throw
===========

Finally you should adjust your servo throw. The throw is the range of
movement for each of your servos.

Check any instructions that came with your plane for suggested throw
values. These are often specified in millimeters or inches of movement
of the trailing edge of the control surface close to the fuselage. If
your aircraft doesn't come with any suggested throw values then choose
a throw that doesn't cause your servos to "bind" (which is indicated
by a high pitched sound when your servos move too far).

To adjust the throw, change the SERVOn_MIN and SERVOn_MAX values. The
defaults are 1100 to 1900. On many aircraft you will want more throw
than that, and can change to a throw of 1000 to 2000 or beyond. 
Make sure that your servos are still moving when nearing the extrememe 
values.

.. tip:: To get to maximum throw on V-Tail control surfaces, command pitch and yaw 
         at the same time in MANUAL mode.

Mixing Gain
===========

The MIXING_GAIN parameter is critical for vtail aircraft. It is the
gain used in mixing between yaw and pitch output and your vtail
movement.

For example, if your MIXING_GAIN is 0.5, then the following outputs
are used:

- LEFT_VTAIL = (yaw+pitch)*0.5
- RIGHT_VTAIL = (yaw-pitch)*0.5

By adjusting the MIXING_GAIN you can adjust the percentabe of throws 
due to pitch vs yaw.

Final Setup
===========

After completing the above you should move onto the final setup of
your aircraft.

- :ref:`ESC Calibration <guide-esc-calibration>`
- :ref:`Center of Gravity <guide-center-of-gravity>`
