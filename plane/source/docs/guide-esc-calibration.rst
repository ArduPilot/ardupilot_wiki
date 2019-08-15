.. _guide-esc-calibration:

===============
ESC Calibration
===============

Planes using a traditional PWM controlled ESC (electronic speed controller) for motor output
will likely need to be calibrated.

.. warning:: Remove the propeller from your aircraft before
             starting ESC calibration

Calibrating an ESC involves teaching the ESC what range of throttle
inputs it should respond to. It needs to know what PWM value on the
throttle channel corresponds with the commanding the motor to be off,
and what PWM value corresponds with full throttle. These values are
stored inside the ESC.

.. note:: You should not try to calibrate your ESC until you have
          completed both your :ref:`RC inputs <rc-throw-trim>`
          calibration and your :ref:`servo setup <servo-functions>`. 
          Mission Planner's motor calibration does not work on 
          traditional planes.

Typical ESC Calibration
=======================

The most common type of ESC calibration for small electric aircraft is
the max-throttle/min-throttle method.

To perform this type of calibration you will need to be able to
control power to your motor separately from power to the autopilot. If
you don't have separate power, then you can temporarily power your
autopilot from a USB cable for the purposes of this calibration.

The steps for calibration are:

- start with the autopilot powered, but with the ESC unpowered and
  propeller removed
- switch to MANUAL flight mode
- disable the safety switch (if fitted) and arm the airplane
- move the throttle stick on your transmitter to maximum
- add power to the ESC

At this stage the ESC/motor should beep to indicate that it is in ESC
calibration mode. Typically it will be 2 or 3 quick beeps, but the
motor won't turn.

- now lower the throttle stick rapidly to zero
- the ESC should beep to indicate it has accepted the new calibration
  range
- now slowly raise the throttle and check that the motor responds
  correctly

Alternative ESC Calibration
===========================

If the typical ESC calibation method above doesn't work,
instead adjust the SERVOn_MIN and SERVOn_MAX values on 
the appropriate channel(s) to match the existing range of your ESC.

To use this method slowly raise the SERVOn_MIN value until it is set
to a value just below the point that the motor comes on. A value about
20 PWM below the point where the motor starts turning is usually a
good choice.

Next move the transmitter stick to full throttle and adjust SERVO3_MAX
until maximum RPM is reached. You can use a tachometer, or just use
the sound of the motor as a guide.

Other ESC Protocols
===================

New ESC's using protocols other than PWM often do not require calibration. 
Some examples include CAN, OneShot, and DShot. The digital signals replace 
the need for PWM calibration.
