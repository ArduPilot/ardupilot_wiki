.. _balance_bot-working:


=====================
How Balance Bots work
=====================

This section gives a brief insight into how the Balance Bot works. Balance Bot is implemented as a frame class within the Rover firmware.

Balancing
=========

.. image:: /images/balance_bot-balancing.png

A Balance Bot is inherently unstable about its wheel axis(pitch axis). If the vehicle is not in upright position(zero pitch angle), gravity applies a torque that causes the vehicle to fall over in the direction of pitch. To counteract this, the motors must apply a reverse torque, which then causes the vehicle to accelerate in the direction of pitch. 

.. image:: /images/balance_bot-control.png

A control system is used to decide the appropriate motor signal to keep the vehicle balanced. The control system takes in a desired pitch value(zero for balancing), uses the sensors on the vehicle to determine its actual pitch angle and calculates the motor signal required to drive the actual pitch angle to the desired value.

Accelerate/Decelerate
=====================

.. image:: /images/balance_bot-accelerate.png

As explained previously, when the pitch is non zero, the vehicle is forced to accelerate as it tries to keep itself from falling. Taking this idea forward, if the control system is set to keep the vehicle at a non zero pitch, the vehicle will be forced to accelerate continuously as it tries to maintain that pitch. To accelerate forward, the vehicle can be made to pitch forward and to decelerate or accelerate backwards, the vehicle can be pitched back.


Rover vs Balance Bot:
+++++++++++++++++++++
The working of a Balance Bot is closer to that of a Copter than a conventional Rover. The motion of a Balance Bot is actually caused by the vehicle pitching, rather than simply a motor throttle. This is unlike in a Rover, where the user can directly control motor throttle, to control the vehicle's acceleration. In a Balance Bot, the user controls the pitch angle, which in turns causes the vehicle to accelerate. The motor throttle in a Balance Bot, is determined by the control system and is meant to correct and hold it's pitch.

.. image:: /images/balance_bot-working.png


Steering
========
Balance Bots have only two wheels and hence use a skid steering or differential drive system. Steering is done by varying the speed of one wheel relative to the other.

