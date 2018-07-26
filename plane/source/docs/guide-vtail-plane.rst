.. _guide-vtail-plane:

=============
V-Tail Planes
=============

A common alternative to a traditional elevator and rudder is a V-Tail, or 
an ATail (an upside down V-Tail).

A V-Tail aircraft has the same functionality as a standard aircraft, 
but it requires special configuration of the servo outputs. Do not use
V-Tail mixing on the transmitter. Although you may choose custom 
channels, the typical V-Tail setup uses channel 2 and 4 for the servo 
outputs.

.. warning:: Remove the propeller from the aircraft before
             starting the setup process.

Configuration & Setup
=====================

The most important step to setting up the plane is having the correct inputs, 
outputs, and reversals. Inputs are covered on the :ref:`RC input setup page <rc-throw-trim>`. 
After the RC inputs are configured, configure the outputs.

.. warning:: Make sure the AHRS_ORIENT is set correctly for the autopilot. If it is incorrect, 
             this setup will fail, and the plane may crash upon entry into any stabilize mode.
             
Servo cables can be connected to any output of the autopilot,
but using the default channels 1-4 listed below is recommended. 
Set the SERVOn_FUNCTIONS to the appropriate values.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th><th>Meaning</th></tr>
   <tr><td>SERVO1_FUNCTION</td><td>4</td><td>aileron</td></tr>
   <tr><td>SERVO2_FUNCTION</td><td>79</td><td>left V-tail</td></tr>
   <tr><td>SERVO3_FUNCTION</td><td>70</td><td>throttle</td></tr>
   <tr><td>SERVO4_FUNCTION</td><td>80</td><td>right V-tail</td></tr>
   </table>


Servo Function & Reversal
=========================

The next step is to correct the V-Tail functions and reversals. 
Both the function and reversal go hand-in-hand, so modifying one may 
partially change the behavior of the other. Connect the battery 
(with propeller removed) and turn on the RC transmitter. Switch to
FBWA mode using the function switch or a ground station command, and 
disable the safety switch (if fitted).

When the plane is level, the servos should be near their trim values. 
Move the plane and leave the transmitter sticks centered while 
monitoring the control surfaces to determine if the function 
and reversals are correct. See the table for the correct control 
surface response to the movements. In each instance, the plane 
should move its control surfaces to level itself.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Movement</th><th>Action</th></tr>
   <tr><td>Roll Plane Right</td><td>Left aileron moves up and right aileron moves down</td><tr>
   <tr><td>Roll Plane Left</td><td>Left aileron moves down and right aileron moves up</td><tr>
   <tr><td>Pitch plane up</td><td>Both tail surfaces move down</td></tr>
   <tr><td>Pitch plane down</td><td>Both tail surfaces move up</td></tr>
   <tr><td>Roll Plane Right</td><td>Both tail surfaces move left</td></tr>
   <tr><td>Roll Plane Left</td><td>Both tail surfaces move right</td></tr>
   </table>

If the ailerons do not respond correctly, reverse the output by changing 
the corresponding SERVOn_REVERSED setting (from 0 to 1, or from 1 to 0).

If the V-tails do not respond correctly, change parameters using the 
following table to enable the correct behavior. Work on one channel 
at a time to avoid confusion.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Control Surface Response</th><th>Corrective Action</th></tr>
   <tr><td>Correct for 1 movement (pitch or roll), but not the other</td><td>Change the function (from 79 to 80; or 80 to 79)</td><tr>
   <tr><td>Incorrect for both movements (pitch and roll)</td><td>Change the reversal of that channel</td><tr>
   </table>

.. note:: KFF_RDDRMIX mut not be set to 0 for rudder setup. If the 
          plane actually needs 0, then reset it after this setup.

          KFF_RDDRMIX should cause the tail surfaces point in the 
          direction of the raised aileron.


Confirm RC Transmitter Input
============================

Keep the plane level in FBWA mode and command the following inputs:

.. raw:: html
         
   <table border="1" class="docutils">
   <tr><th>Input</th><th>Action</th></tr>
   <tr><td>Roll Right</td><td>Right aileron moves up and left aileron moves down</td><tr>
   <tr><td>Roll Left</td><td>Left aileron moves up and right aileron moves down</td><tr>
   <tr><td>Pitch up</td><td>Both tail surfaces moveup</td><tr>
   <tr><td>Pitch down</td><td>Both tail surfaces move down</td><tr>
   <tr><td>Yaw right</td><td>Both tail surfaces move right</td><tr>
   <tr><td>Yaw left</td><td>Both tail surfaces move left</td><tr>
   </table>

Double check MANUAL mode for the inputs as well. If everything is setup correctly, 
the plane should be almost ready to fly.
   
ATail Planes
============

With "A-Tail" planes (an inverted V-Tail), the control surface movements 
referenced above should still be the same directions. It is likely that the servo 
reversal or function will be opposite from a similar V-Tail setup.

Servo Trim
==========

Switch back to MANUAL mode in order to adjust the servo trim
values. The servo trim is in the SERVOn_TRIM parameters.

Adjust the trim values so that the servo is centered when
the transmitter sticks are centered. If the trim value is not 
between 1450 and 1550 PWM, mechanical trim adjustment is recommended.

Servo Throw
===========

Finally adjust the servo throws (range of
movement for each of the servos).

Check any instructions that came with the plane for suggested throw
values. These are often specified in millimeters or inches of movement
of the trailing edge of the control surface close to the fuselage. If
suggested throw values are not found, then choose a throw that doesn't 
cause the servos to "bind" (often indicated by a high pitched sound 
when servos stall).

To adjust the throw, change the SERVOn_MIN and SERVOn_MAX values. The
defaults are 1100 to 1900. On many aircraft, more throw may be desired.
Changing throws to 1000 to 2000 or beyond is normal. Make sure that 
the servos are still moving when nearing the extrememe values.

.. tip:: To get to maximum throw on V-Tail control surfaces, command pitch and yaw 
         at the same time in MANUAL mode.

Mixing Gain
===========

The MIXING_GAIN parameter is critical for vtail aircraft. It is the
gain used in mixing between yaw and pitch output and the vtail
movement. For example, if MIXING_GAIN is 0.5, then the following outputs
are used:

- LEFT_VTAIL = (yaw+pitch)*0.5
- RIGHT_VTAIL = (yaw-pitch)*0.5

Adjusting the MIXING_GAIN controls the percentabe of throws from pitch vs yaw.

Final Setup
===========

After completing the V-Tail guide, move onto the final setup of
the aircraft.

- :ref:`ESC Calibration <guide-esc-calibration>`
- :ref:`Center of Gravity <guide-center-of-gravity>`
