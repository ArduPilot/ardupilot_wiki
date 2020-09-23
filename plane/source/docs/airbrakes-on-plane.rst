.. _airbrakes-on-plane:


==========
Airbrakes
==========

`Airbrakes <https://en.wikipedia.org/wiki/Air_brake_(aeronautics)>`__ are control surfaces that primarily add drag to your aircraft. 

These control surfaces can be driven by:

 - manual input via an RC channel.
 - automatic airbrake control, especially useful for steeper and more accurate automatic landing.
 - a combination of both, in which case the greater value is used.

Servo Output
============

Connect the airbrake servo to a servo output channel on your flight controller. Set the corresponding servo output function to Airbrake (110). Set the servo minimum value SERVOX_MIN to the value for airbrake fully retracted. Set the maximum value SERVOX_MAX to the value for airbrake fully deployed.



RC Channel Input
================

Assign a spare RC channel on your transmitter to a dial, slider or 3 position switch. On the flight controller, ensure this channel is calibrated and assign this channel the RC channel ``RCx_OPTION`` option Airbrake (210).

.. _airbrake-setup:


Automatic Airbrake Setup
========================

You can have airbrakes automatically used in an automated landing or in conjunction with Reverse Thrust operation during certain throttled controlled modes or automatic landings and throttle-controlled descents, such as CRUISE mode while soaring. If manual control has been enabled using the ``RCX_OPTION`` (64), then the airbrake deployment will be the greater of either the automatic demand or manual control.

Use With Reverse Thrust on Motor
--------------------------------

Setup :ref:`Reverse Thrust<reverse-thrust>` as desired. Airbrake operation will be in sync with reverse throttle applied.

Use Without Reverse Thrust ESC
------------------------------

Same as above, however:

- Set :ref:`SERVO3_TRIM<SERVO3_TRIM>` = :ref:`SERVO3_MIN<SERVO3_MIN>` assuming SERVO 3 output is the throttle. This will assure that low stick and minimum demanded throttle is always SERVOx_MIN.


