.. _guide-four-channel-plane:

===================
Four Channel Planes
===================

The most common type of fixed wing plane is a 4 channel plane. As the
name implies, it has 4 output channels, and they control the roll, pitch, yaw,
and throttle independently.

These are the default outputs for ArduPlane as it is such a common
setup. It is commonly referred to as an AETR setup.

.. warning:: Remove the propeller from your aircraft before
             starting your setup.

Setting Up Your Plane
=====================

The most important step to setting up the plane is having the correct inputs, 
outputs, and reversals. Inputs are covered on the :ref:`RC input setup page <rc-throw-trim>`. 
After the RC inputs are configured, configure the outputs.

.. warning:: Make sure the :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>` is set correctly for the autopilot. If it is incorrect, 
             this setup will fail, and the plane may crash upon entry into any stabilize mode.
             
Servo cables can be connected to any output of the autopilot,
but using the default channels 1-4 listed below is recommended. 
Set the SERVOn_FUNCTION to the appropriate values.

You can do this by setting the  parameter directly using Ground Station software such as Mission Planner or QGroundControl using either the full parameter lists or setup tab for outputs. In Mission Planner you can do this on the SETUP -> Mandatory Hardware -> Servo Output page. Each channel has a dropdown which can be used to select the function of the channel.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th><th>Meaning</th></tr>
   <tr><td>SERVO1_FUNCTION</td><td>4</td><td>aileron</td></tr>
   <tr><td>SERVO2_FUNCTION</td><td>19</td><td>elevator</td></tr>
   <tr><td>SERVO3_FUNCTION</td><td>70</td><td>throttle</td></tr>
   <tr><td>SERVO4_FUNCTION</td><td>21</td><td>rudder</td></tr>
   </table>
   
.. tip:: Most 4-channel aircraft utilize a Y-splitter for ailerons. It works well if 
         the servos have equal travel ranges and mirrored movement. If you wish to set
         the trim, max, or min values for each aileron servo independently, then use
         another output like channel 5 for the second aileron. Be sure to set that
         channel's function correctly (4). This principle applies to any additional
         servo or motor output.

Servo Function & Reversal
=========================

The next step is to correct the servo reversals. 
Connect the battery (with propeller removed) and turn on the RC transmitter.
Switch to FBWA mode using the function switch or a ground station command, and 
disable the safety switch (if fitted). We use FBWA for setup because it prevents
double-reversing inputs and outputs. Double-reversing causes correct manual
behavior, but dangerous and destabilizing behavior in other flight modes.

When the plane is level, the servos should be near their trim (neutral) values. 
Move the plane and leave the transmitter sticks centered while 
monitoring the control surfaces to determine if the reversals are correct.
See the table for the correct control surface response to the movements. 
In each instance, the plane should move its control surfaces to level itself
and coordinate its turns.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Movement</th><th>Action</th></tr>
   <tr><td>Roll Plane Right</td><td>Left aileron moves up and right aileron moves down</td><tr>
   <tr><td>Roll Plane Left</td><td>Left aileron moves down and right aileron moves up</td><tr>
   <tr><td>Pitch plane up</td><td>Elevator moves down</td></tr>
   <tr><td>Pitch plane down</td><td>Elevator moves up</td></tr>
   <tr><td>Roll Plane Right</td><td>Rudder moves left</td></tr>
   <tr><td>Roll Plane Left</td><td>Rudder moves right</td></tr>
   </table>

If the any of the control surfaces do not respond correctly, reverse the 
output by changing the corresponding SERVOn_REVERSED setting (from 0 to 1, 
or from 1 to 0).

The output can be reversed in Mission Planner on the SETUP/Mandatory Hardware/Servo Output page, just check the box for the channel that needs to be reversed. This will set the output's SERVOn_REVERSED parameter.

.. note:: :ref:`KFF_RDDRMIX<KFF_RDDRMIX>` mut not be set to 0 (the default value is 0.5) for checking rudder movement in FBWA as the plane is rolled away from level. If no automatic coupling of rudder to aileron for coordinated turns by the autopilot is desired, then reset it to zero after this setup check.

Confirm RC Transmitter Input
============================

Keep the plane level in FBWA mode and command the following inputs by moving the sticks on your transmitter:

.. raw:: html
         
   <table border="1" class="docutils">
   <tr><th>Input</th><th>Action</th></tr>
   <tr><td>Roll Right</td><td>Right aileron moves up and left aileron moves down</td><tr>
   <tr><td>Roll Left</td><td>Left aileron moves up and right aileron moves down</td><tr>
   <tr><td>Pitch up</td><td>Elevator moves up</td><tr>
   <tr><td>Pitch down</td><td>Elevator moves down</td><tr>
   <tr><td>Yaw right</td><td>Rudder moves right</td><tr>
   <tr><td>Yaw left</td><td>Rudder moves left</td><tr>
   </table>

If the control surfaces do not respond correctly, change the RCn_reversed
parameter (from 0 to 1, or from 1 to 0). Do NOT reverse the output on your transmitter. It must be changed in the autopilot!
Double check MANUAL mode for the
same inputs. If everything is setup correctly, the plane should be almost
ready to fly.

If in MANUAL mode the surfaces move backward with stick movements now, the corresponding RC input should be reversed. To do this, in Mission Planner RCn_REVERSED can be easily set on the SETUP/Mandatory Hardware/Radio Calibration page. There is a check box "Reverse" next to each input bar. You can reverse the correct parameter by checking the box. However, if :ref:`common-radio-control-calibration` was followed correctly, changing RC input reversal will not be necessary.

Servo Trim
==========

Switch to MANUAL mode in order to adjust the servo trim
values. The servo trim is in the SERVOn_TRIM parameters.

Adjust the trim values so that the servo is centered when
the transmitter sticks are centered. If the trim value is not 
between 1450 and 1550 PWM, mechanical trim adjustment is recommended.

Servo Throw
===========

Finally adjust the servo throws (range of
movement for each of the servos). This should also be done in MANUAL mode.

Check any instructions that came with the plane for suggested throw
values. These are often specified in millimeters or inches of movement
of the trailing edge of the control surface close to the fuselage. If
suggested throw values are not found, then choose a throw that doesn't 
cause the servos to "bind" (often indicated by a high pitched sound 
when servos stall).

To adjust the throw, change the SERVOn_MIN and SERVOn_MAX values. The
defaults are 1100 to 1900. On many aircraft, more throw may be desired.
Changing throws to 1000 to 2000 or beyond is normal. Make sure that 
the servos are still moving when nearing the extreme values.

Final Setup
===========

After completing this guide, move onto the final setup of
the aircraft.

- :ref:`ESC Calibration <common-esc-calibration>`
- :ref:`Center of Gravity <guide-center-of-gravity>`
