.. _tuning-quickstart:

=================
Tuning QuickStart
=================

This article provides information to help you get started with airframe
tuning. It includes an overview of the main steps, tools and concepts.

How to tune the airframe
========================

With the default PID settings, Plane will fly the majority of
lightweight RC airframes (not full size, full speed airframes) safely,
right out of the box. To fly well, with tight navigation and reliable
performance in wind, you'll want to tune your autopilot.

The most important configuration is Roll and Pitch tuning, as this is
essential for responsive and stable flight as well as effective
navigation. The best way to tune the roll and pitch is to use :ref:`Automatic Tuning with AUTOTUNE <automatic-tuning-with-autotune>` (a flight mode
that uses changes in flight attitude input by the pilot to learn the key
values needed).

.. tip::

   To use AUTOTUNE you must to be able to fly the aircraft. Plane will
   fly the majority of lightweight RC airframes right out of the box. We
   also provide :ref:`configuration values for many common aircraft <configuration-files-for-common-airframes>` that you can use
   to get your aircraft flying before doing further tuning.

   If AUTOTUNE doesn't work with your plane, a fully manual approach is
   described in the :ref:`Manual Roll, Pitch and Yaw Controller Tuning Guide <new-roll-and-pitch-tuning>`.

After tuning the Roll, Pitch (and optionally yaw) you should tune the
height controller using the :ref:`TECS tuning guide <tecs-total-energy-control-system-for-speed-height-tuning-guide>`
and the horizontal navigation using the \ :ref:`L1 controller tuning guide <navigation-tuning>`.

Information on how to tune other aspects of Plane are linked from the
:ref:`Tuning landing page. <common-tuning>`

PID gain values
===============

The control of roll or pitch angle is adjusted using a
`Proportional-Integral-Derivative (PID) Controller <https://en.wikipedia.org/wiki/PID_controller>`__.

The final control applied to the plane's control surface, is a
combination of the effects of four gain values:

-  *Proportional gain (P)* is the simplest form of control, it is the
   "present" error. Autopilot wants 10 degrees of pitch, has 5 degrees,
   that is an error of 5: apply some amount of elevator (the amount
   applied for the amount of error is determined - scaled - by the P
   number).
-  *Integral gain (I)* takes into consideration previous errors and is
   able to compensate for steady errors. It can be thought of as an
   automatic trim adjustment. The disadvantage with the "I" gain is that
   because it is always reacting to past errors, it reduces damping of
   the control loop as it is always playing 'catchup'.
-  *Derivative gain (D)* adds damping because it feeds back the rate of
   change of the angle. It can also be thought of as attempting to
   anticipate future changes in angle. The disadvantage of the "D"
   gain is that it increases the amount of noise driving the servo and
   if turned up too high will cause rapid pitch or roll oscillation
   that can in some cases damage the aircraft.
-  *FeedForward gain (FF)* is perhaps the most important since it directly drives
   the control surfaces from the demanded rate input from the autopilot, much as the
   pilot does in manual mode. The P,I, and D rate error-based contributions add to this
   to correct any trim,CG, or external disturbance impacts.

Tuning FF, P, PI or PID values can improve how quickly an observed error
between desired attitude (pitch, speed, bearing, whatever) and actual
attitude can be canceled out, without undue oscillation.

.. tip::

   A simple configuration can use just the FF and P terms, with I = FF and D = 0.

Refer to :ref:`Manual Roll, Pitch and Yaw Controller Tuning Guide <new-roll-and-pitch-tuning>` for more information.
