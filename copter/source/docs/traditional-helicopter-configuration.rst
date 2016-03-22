.. _traditional-helicopter-configuration:

============================================================
Traditional Helicopter – Configuration using Mission Planner
============================================================

Setup using AP Mission Planner
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Connect your APM to your computer using the USB, set the COM port and
the baud rate and then press "Connect" and wait for the parameters on
the APM to be transferred to your computer. Now, from the Firmware
screen, push the "APM Setup (Plane and Quad)" button, then click on the
Radio Calibration tab.

.. image:: https://awsive-images.googlecode.com/git-history/master/Firmware%20v1-2.JPG
    :target: ../_images/Firmware%20v1-2.JPG

Radio Calibration
~~~~~~~~~~~~~~~~~

When it comes to calibrating your radio in Mission Planner, all the
physical sticks should move in the same direction as the green bars on
the Radio Calibration Screen with the exception of Pitch which should
move in the opposite direction to the physical sticks.

To get the bars moving in the right direction you may need to reverse a
channel in your radio but you really shouldn’t have to with the
exception of throttle/collective for Futaba radios.

.. image:: https://awsive-images.googlecode.com/git-history/master/MP%20Radio%20Calibration.jpg
    :target: ../_images/MP%20Radio%20Calibration.jpg

Accelerometer calibration
~~~~~~~~~~~~~~~~~~~~~~~~~

Before you attach your APM to the heli I would recommend you do an
accelerometer calibration which is well
described \ `here <http://vimeo.com/56224615>`__. It’s easier to do if
the APM is not mounted in the heli and it’s essential when running
firmware 2.9 or higher.

Configuring the Servos
~~~~~~~~~~~~~~~~~~~~~~

.. image:: https://awsive-images.googlecode.com/git-history/master/Swash%20Setup%20v1-2.JPG
    :target: ../_images/Swash%20Setup%20v1-2.JPG

1. Ensure the Collective pitch bar moves up as you move your collective
pitch up. If it does not, repeat the Radio Calibration. You may need to
reverse the collective pitch setting on your radio.

2. Check that all your servos are moving in the correct direction as you
move your collective pitch up and down. If any move in the opposite
direction, click the rev check box beside the appropriate servo. Don't
be surprised or concerned if one or two of the servos move in the wrong
direction just click the rev check box. I sure to be needed on at least
one servo.

3. Type in the correct positions of each servo around the swash plate in
degrees. 0 = at the front of the swash. Refer to the compass image on
the set-up screen if necessary. Note: after updating one of the position
values you must leave the field (or click on the compass) and you should
then notice the swash twitches momentary as it is reinitialised.

Capturing the Swash Plate Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. image:: https://awsive-images.googlecode.com/git-history/master/Swash%20Setup%20Slide%202%20v1-2.JPG
    :target: ../_images/Swash%20Setup%20Slide%202%20v1-2.JPG

4. Push the \ **Manual** button. This will momentarily allow the servos
to move freely and input from the radio will directly control the
servos.

Change the collective pitch on your radio to move the swash to it's
upper and lower limits. If you accidentally move the swash too far so
the servos bind, manually set the Top and Bottom of the range in the
field provided.

Push the \ **Zero** button after making sure your main blade's pitch is
zero. If you want to tune your heli even better, set the pitch at hover
pitch, say 3 or 4 degrees, and your heli won't climb when you switch
modes but this is a refinement that you may want to come back to after
your initial set-up.

Push the \ **Save** button.

If you want to re-do this step, close the setup window and re-open it.

5. Update the maximum roll and pitch values into the fields provided as
a number of degrees. Default is 45 degrees which is what most people
use.

**Note: Avoid reversing the collective pitch channel (RC_3) in Mission
Planner because it will affect the way the arming system works. Instead,
reverse your \ *servos* in Mission Planner OR reverse the \ *channel* in
your radio.**

Configuring the Rudder
~~~~~~~~~~~~~~~~~~~~~~

.. image:: https://awsive-images.googlecode.com/git-history/master/Rudder%20Setup%20v1-2.JPG
    :target: ../_images/Rudder%20Setup%20v1-2.JPG

6. Tick the Rev check box if you find the tail moves in the wrong
direction. Again, its not atypical to have to reverse this function.
Remember, stick to the right, heli should rotate clockwise, Stick to the
left, heli should rotate anti-clockwise (when viewed from above).

7. Push the \ **Manual** button and move the rudder channel on your
radio to it's full range to capture the min and max rudder range while
avoiding any binding.

Push the \ **Save** button.

As with the collective pitch you can update the Min and Max manually if
necessary.

8. If you have an external gyro, click the Gyro \ **Enable** button.
Enter the Gain manually. This is a value from 1000~2000. this servo
value is simply output on Channel 7 which should be connected to your
external gyro's gain channel (thus freeing up a radio channel for other
purposes). An external gyro really is not needed and this is a legacy
option. The APM2.5+ will lock the tail beautifully.

Rotor Speed Control (RSC) using Output Channel 8 on APM2.5
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To use the APM to control the rotor speed (ie.throttle/ESC), all you
have to do is send whatever throttle signal you want in on Ch8(in) and
then plug the throttle servo or ESC into Ch8(out) on the APM. Ch8
throttle control is important because it \ **forces you to arm the APM
before you can fly.** Without arming, the motor will not start nor will
the collective servos work. So Ch8 is used for switching the
motor/collective on and off something like a throttle-hold.

RCS can be set on the Heli Setup Tab of Mission Planner

.. image:: https://awsive-images.googlecode.com/git-history/master/Rotor%20speed%20control%20setup%20in%20MP.jpg
    :target: ../_images/Rotor%20speed%20control%20setup%20in%20MP.jpg

There are three settings to control how the APM sets up Ch8 control:

**Note the following error on Mission Planner: Mode 1 is a direct pass
through of the throttle input signal and Mode 2 is for an ESC with built
in governor. MP asks you for the "Mode 1 Setpoint" This is a typo and it
should ask for the "Mode 2 Setpoint". Please test with caution.
Hopefully this will be rectified in the next release.**

Disable (H_RSC_MODE set to 0)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you don’t want to use Ch8 control then set H_RSC_MODE to 0. Now you
can arm the APM without Ch8 but the collective will only work once
armed. Hopefully you can live with that and it achieves the important
step of insuring that you do not fly unarmed.

Mode_1 (H_RSC_MODE set to 1)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

With H_RSC_Mode set to 1, you have a direct Ch8 pass-through; however
it is still set low when disarmed. The motor will ramp-up subject to
H_RSC_RAMP which can be set to 0 if you want to rely solely on your
ESC’s start-up characteristics. Once ramped up Ch8(out) it is slaved to
the Ch8(in) so you can pass through a variable throttle signal if you
want.

All H_RSC_RAMP does is, after arming, when you first engage the
throttle in Mode 1 \ **or** 2, it ramps up the output slowly. It's like
a super-soft-start.

RSC_Ramp set to 1000 = 10 seconds.

Also note the APM won't arm in Mode 1 or 2 unless Ch8 is within 10 of
RC8_Min.

Mode_2 (H_RSC_MODE set to 2)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When H_RSC_MODE is set to 2 the APM now only sets Ch8(out) to be high
or low and, when high, it sends out H_RSC_SETPOINT (this is the
setpoint in MP) while when low it sends out RC8_Min.

Plug the ESC into Ch8 on the APM and put it in governor mode.

Ch8 can be driven by a 2 position switch. When the switch is down, it
outputs whatever the minimum is (endpoint is set to -100%) and when the
switch is up it sends the maximum signal, so endpoint +100%.

The only trick is to "calibrate" the ESC to the signal which is easily
done if you can plug the ESC into the Rx.

To calibrate it using the APM, take the blades off or loosen off the
pinion. First make sure you have done a radio calibration in APM using
the 100%/100% endpoint on Ch8. Unplug the ESC from the APM. Boot the
system up, the ESC should be beeping at you because no signal. Now, make
sure the Ch8 switch is low or you can’t arm the APM. Now arm it, set the
collective stick to the middle (this prevents it disarming due to
inactivity) and then switch the Ch8 switch high and wait about 15
seconds. It should now be outputting a high signal on Ch8. Plug the ESC
in. It should give you a confirmation that it has gotten a high signal
and waiting for low. Now, turn Ch8 off. The PWM output will immediately
drop to the minimum. Your ESC should beep to tell you it has read the
minimum, and is ready to go. Now, if you switch Ch8 high again, the
motor should go to full power. It will ramp slowly over 10 seconds if
you have left the R_RSC_RAMP at 1000. You could set this to 0 if you
have a good soft-start on your ESC. Or you could change it to 500 for 5
seconds, 2000 for 20 seconds, whatever you want.

Then after all this is done, go into the endpoints and I change the high
endpoint so that it is outputting only 80% throttle when it's switched
on. Now the governor gets an 80% signal when running and you get a nice
even head speed while flying.

Now it's really easy to use. Put Ch8 off, collective down. Arm the heli,
switch Ch8, the motor starts and gets to the target speed. Now you can
take off.

The only catch here is that if you arm and leave the collective at full
negative it will disarm after 10 to 15 seconds. When this happens the
APM will immediately shut down the motor. So it's safe but it can be a
nuisance. To avoid this, while waiting for spool up, move the collective
up just off the bottom, not enough to fly and this will prevent the
disarming.

The APM will always arm with the stick down/right and disarm with
down/left.

Getting to your First Flight
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

So here is what you need to do to get to your first flight.

Swash Set-up
~~~~~~~~~~~~

What flight mode should you have your radio set to while tuning the
servos?

Usually Stab mode but sometimes it’s worth looking at Acro mode too,
depending on what you’re doing. But NEVER in auto mode. If you are
trying to adjust the collective you should be using the button for that
in the Mission Planner Heli tab. If you are trying to do it the manual
way, by just twiddling the settings in the Advanced Param tab, then you
should use Acro for collective adjustments. This ensures that the
STAB_COL params are not in play. So you would set your ABSOLUTE min and
max in Acro mode. Then set the STAB_COL in Stab mode.

Remember, any time you change any of the numbers in the Advanced
Parameter list you must got to H_SV_MAN, set it to 1, then “write”,
back to 0, then “write”. This resets the swash calculations. If you
don't do that, it messes things up. If you modify the swash through
Mission Planner this is done automatically by the Mission Planner
software.

Set your heli up with +/-10° pitch. Then, you will need to go to
Configuration>Advanced Parameters>Parameters List and find
H_STAB_COL_MIN and H_STAB_COL_MAX and set these to 30 and 90
respectively. This will give you a collective pitch range of about -2°
to +8° in Stabilisation mode.

Now, any time you change these numbers (ie: change them in Mission
Planner and "Write" to the APM), they won't take effect right away.
There's a number that gets calculated and it is only recalculated when
you reset the swash. The swash is reset any time you reboot, however,
you can force it by finding H_SV_MAN in the Parameter List and setting
it to 1, “write” and then set it back to 0, and “write”. This forces it
to reset the swash and you'll see the effect of the H_STAB_COL changes
immediately. Hopefully this will be changed in a future version of
Mission Planner with all of this in the heli setup tab.

You should be able to switch between Acro and Stab, move your throttle
and see the difference in the swash plate movement.

We do recommend setting up the swash with lots of negative pitch and
then set the H_STAB_COL_MIN for whatever negative pitch you are
comfortable with. Even if you never use negative pitch, this still
allows the
Alt_Hold\ `? <https://code.google.com/p/arducopter/w/edit/Alt_Hold>`__ controller
to have access to full negative pitch will sometimes be needed.

Set H_COL_MID to be hover point rather than 0° pitch to avoid having
the heli ascend when you switch to Loiter or Alt Hold Mode. So that
covers the swash plate setup.

Flybar Mode
~~~~~~~~~~~

So here's what
H_Flybar_Mode\ `? <https://code.google.com/p/arducopter/w/edit/Flybar_Mode>`__
= 1 does.

First, in Acro mode, it skips ALL stabilization/rate controllers. Your
stick inputs go DIRECTLY to the servos. The only thing the APM does is
the CCPM mixing. It becomes completely dumb. So Acro = Full Manual. The
only real issue with this is that there is basically no trim. If you are
trying to hover, and the swash isn't setup right mechanically, it'll
roll. The only way you could stop that is by adjusting your swash
linkages, or using radio trim. Using radio trim is not good, because
then that will mess up all your other modes because the APM will think
you are holding the sticks.

So the second big thing
Flybar_Mode\ `? <https://code.google.com/p/arducopter/w/edit/Flybar_Mode>`__
does is that it makes the Rate I term only "active" near zero rate
command. It won't move whenever you're asking the heli to move. It will
only move the Integrator, basically in a hover. So it's sort of like an
auto-trim for hover. Whenever you are moving the sticks, it's frozen.
Again, I did that because I didn't want the Integrator doing whacky
things to the flybar, because the flybar and the rate integrator do the
exact same thing, but neither one of them knows what the other is doing!
