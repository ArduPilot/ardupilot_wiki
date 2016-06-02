.. _quadplane-esc-calibration:

ESC calibration
===============

Most models of PWM based ESC need to be calibrated to ensure that all
the ESCs respond to the same input with the same speed. To calibrate
them they need to receive maximum PWM input when initially powered on,
then receive minimum PWM input when they have beeped to indicate that
the maximum has registered.

The method for calibrating the ESCs on an ArduPilot QuadPlane depends
on what version of the firmware you have loaded. The old method is
used on versions prior to the 3.6.0 release. The new method is used on
3.6.0 and later.

.. warning::
   You must remove all propellers from your vehicle before doing any
   ESC calibration. Calibrating with propellers installed is dangerous.

ESC Calibration Procedure (3.6.0 and later)
-------------------------------------------

This process uses the :ref:`Q_ESC_CAL <Q_ESC_CAL>` parameter to enable
ESC calibration in QSTABILIZE mode. There are two modes of operation
available:

#. with Q\_ESC_CAL=1 the output to the motors will come directly from
   the throttle stick in QSTABILIZE mode when the vehicle is armed
#. with Q\_ESC_CAL=2 the output to the motors will be full throttle
   when the motors are armed

The process when using Q\_ESC_CAL=1 is

#. remove your propellers for safety
#. power up just the flight board and not your motors. If you don't have
   the ability to isolate power to the ESCs when on battery power then
   power up your flight board on USB power
#. set the Q\_ESC_CAL parameter to 1
#. change to QSTABILIZE mode
#. set the safety switch off to activate the outputs
#. arm your aircraft. The PWM output on all quad motors will now be
   controlled by your throttle stick
#. move the throttle stick to maximum
#. add power to your ESCs by connecting the battery
#. wait for the ESCs to beep to indicate they have registered the
   maximum PWM
#. lower the throttle stick to zero and disarm your aircraft
#. you should hear a beep from your ESCs to indicate they have
   registered the throttle range

The process when using Q\_ESC_CAL=2 is

#. remove your propellers for safety
#. power up just the flight board and not your motors. If you don't have
   the ability to isolate power to the ESCs when on battery power then
   power up your flight board on USB power
#. set the Q\_ESC_CAL parameter to 2
#. change to QSTABILIZE mode
#. set the safety switch off to activate the outputs
#. arm your aircraft. The PWM output on all quad motors will now be at maximum
#. add power to your ESCs by connecting the battery
#. wait for the ESCs to beep to indicate they have registered the
   maximum PWM
#. disarm your aircraft
#. you should hear a beep from your ESCs to indicate they have
   registered the throttle range

Note that using Q\_ESC_CAL=1 can be useful for testing your motors
response. This is the only mode when you are able to directly control
the throttle level on all your motors at once. While in this mode you
can use a laser tachometer to test your motor speeds at different
throttle levels if you have one.


Old ESC Calibration Procedure (3.5.3 and earlier)
-------------------------------------------------

#. remove your propellers for safety
#. power up just the flight board and not your motors. If you don't have
   the ability to isolate power to the ESCs when on battery power then
   power up your flight board on USB power
#. set both the parameters :ref:`Q_M_SPIN_ARMED <Q_M_SPIN_ARMED>` and
   :ref:`Q_THR_MID <Q_THR_MID>` to 1000.
   This sets the PWM output when armed at zero throttle to full power
#. set the safety switch off to activate the outputs
#. arm your aircraft. The PWM output on all quad motors will now climb
   to maximum.
#. add power to your ESCs by connecting the battery
#. wait for the ESCs to beep to indicate they have registered the
   maximum PWM
#. disarm your aircraft. The ESCs should beep again indicating they have
   registered minimum PWM

Now set the Q_M\_SPIN_ARMED and Q_THR_MID parameters back to the
correct values. A value of 50 for Q_M\_SPIN_ARMED is a reasonable
starting point. For Q_THR_MID a value of between 500 and 600 is good
depending on the power of your motors

