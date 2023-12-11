.. _balance_bot-working:


=====================
How Balance Bots work
=====================

This section gives a brief insight into how the Balance Bot works. Balance Bot is implemented as a frame class within the Rover firmware.

Balancing
=========

.. image:: /images/balance_bot-balancing.png

A Balance Bot is inherently unstable about its pitch axis. If the vehicle is not perfectly verticle (e.g. pitch angle of zero), gravity applies a torque that causes the vehicle to fall over in the direction of pitch. To counteract this, the motors must apply a reverse torque which causes the vehicle to accelerate in the direction of pitch.

.. image:: /images/balance_bot-control.png

A Pitch PID controller is used to calculate the required throttle to keep the vehicle pitched at a desired angle (e.g. zero to stay balanced without moving forward).

Accelerate/Decelerate
=====================

.. image:: /images/balance_bot-accelerate.png

As explained previously, when the pitch is non zero, the vehicle is forced to accelerate as it tries to keep itself from falling. Taking this idea forward, if the control system is set to keep the vehicle at a non zero pitch, the vehicle will be forced to accelerate continuously as it tries to maintain that pitch. To accelerate forward, the vehicle can be commanded to pitch forward and to decelerate or accelerate backwards, the vehicle can be commanded to pitch back.


Pilot Control of a Balance Bot
==============================

With more typical Rovers (e.g. not balance bots), while in Manual mode, the pilot directly controls the vehicle's throttle output using Ch3 of the transmitter (see top section of the image below).

With Balance Bots however the pilot's throttle controls the desired pitch angle (up to the maximum held in :ref:`BAL_PITCH_MAX<BAL_PITCH_MAX>`).  The throttle output is then calculated by the pitch controller in order to keep the vehicle at the desired pitch.

.. image:: /images/balance_bot-working.png


Steering
========
Balance Bots have only two wheels and hence use a skid steering or differential drive system. Steering is done by varying the speed of one wheel relative to the other.

