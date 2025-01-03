.. _guide-rudder-only-plane:

==============================
Rudder Only Planes (3 Channel)
==============================

A less common type of fixed wing plane is a 3 channel plane. As the
name implies, it has 3 output channels. While pitch and throttle are directly controlled,
roll and yaw are coupled.
They are controlled through a single channel on the transmitter.
Compared to using 4 channel airplanes, 3 channel planes use less servos, have less failure points,
but provide less direct control over the vehicle's motion and less redundance against servo failure.

.. warning:: Remove the propeller from your aircraft before
             starting your setup.

Setting Up Your Plane
=====================

The most important step to setting up the plane is having the correct inputs, 
outputs, and reversals. Inputs are covered on the :ref:`RC input setup page <rc-throw-trim>`.

For a 3-channel plane, configure the following parameters:

* :ref:`RUDDER_ONLY<RUDDER_ONLY>` =1
* :ref:`KFF_RDDRMIX<KFF_RDDRMIX>` =1.0
* :ref:`YAW2SRV_DAMP<YAW2SRV_DAMP>` =0.5

:ref:`YAW2SRV_DAMP<YAW2SRV_DAMP>` can be tuned later; the above value is a good starting value.

If you prefer to control the throttle with one stick and the attiude of the vehicle with the other stick,
set up your transmitter to send roll stick inputs to the :ref:`RCMAP_YAW<RCMAP_YAW>` channel (normally channel 4).
The rudder servo should be attached to the :ref:`RCMAP_YAW<RCMAP_YAW>` channel as well (normally channel 4).

In the Radio Calibration setup page, you should now observe that a right roll command on the sticks results in the same
movement as a right yaw command.

Later, when you set up :ref:`arming-your-plane`, remember that you can now arm with the configured roll stick.

After the RC inputs are configured, configure the outputs.

.. warning:: Make sure the :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>` is set correctly for the autopilot. If it is incorrect, 
             this setup will fail, and the plane may crash upon entry into any stabilize mode.
             
Servo cables can be connected to any output of the autopilot,
but using the default channels 2-4 listed below is recommended. 
Set the SERVOn_FUNCTION to the appropriate values

You can do this by setting the  parameter directly using Ground Station software such as Mission Planner or QGroundControl using either the full parameter lists or setup tab for outputs.
In Mission Planner you can do this on the SETUP -> Mandatory Hardware -> Servo Output page. Each channel has a dropdown which can be used to select the function of the channel.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th><th>Meaning</th></tr>
   <tr><td>SERVO2_FUNCTION</td><td>19</td><td>elevator</td></tr>
   <tr><td>SERVO3_FUNCTION</td><td>70</td><td>throttle</td></tr>
   <tr><td>SERVO4_FUNCTION</td><td>21</td><td>rudder</td></tr>
   </table>


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
   <tr><td>Roll Plane Right</td><td>Rudder moves left</td><tr>
   <tr><td>Roll Plane Left</td><td>Left aileron moves down and right aileron moves up</td><tr>
   <tr><td>Pitch plane up</td><td>Elevator moves down</td></tr>
   <tr><td>Pitch plane down</td><td>Elevator moves up</td></tr>
   <tr><td>Yaw Plane Right</td><td>Rudder moves left</td></tr>
   <tr><td>Yaw Plane Left</td><td>Rudder moves right</td></tr>
   </table>

If the any of the control surfaces do not respond correctly, reverse the 
output by changing the corresponding SERVOn_REVERSED setting (from 0 to 1, 
or from 1 to 0).

The output can be reversed in Mission Planner on the SETUP/Mandatory Hardware/Servo Output page, just check the box for the channel that needs to be reversed. This will set the output's SERVOn_REVERSED parameter.

Confirm RC Transmitter Input
============================

Keep the plane level in FBWA mode and command the following inputs by moving the sticks on your transmitter:

.. raw:: html
         
   <table border="1" class="docutils">
   <tr><th>Input</th><th>Action</th></tr>
   <tr><td>Roll Right</td><td>Rudder moves right</td><tr>
   <tr><td>Roll Left</td><td>Rudder moves left</td><tr>
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

If in MANUAL mode the surfaces move backward with stick movements now, the corresponding RC input should be reversed. To do this, in Mission Planner RCn_REVERSED can be easily set on the SETUP/Mandatory Hardware/Radio Calibration page.
There is a check box "Reverse" next to each input bar. You can reverse the correct parameter by checking the box. However, if :ref:`common-radio-control-calibration` was followed correctly, changing RC input reversal will not be necessary.

.. include:: guide-four-channel-plane.rst
    :start-after: Confirm RC Transmitter Input

