.. _rover-steering-input-type-and-reversing-behaviour:

===========================================
Steering Input Type and Reversing Behaviour
===========================================

The :ref:`PILOT_STEER_TYPE <PILOT_STEER_TYPE>` parameter provides control of two behaviours related to the pilot's steering input.

Two Paddle Input
----------------

Normally the user (aka Pilot) controls the vehicle using one transmitter channel for steering (i.e. move a stick left-right) and another for throttle (move a stick forward-back) but it is also possible to control the steering and direction with two sticks (both moving forward-back) as if controlling two wheels of a skid-steer vehicle (like R2D2).

This Two Paddle Input method can be enabled by setting :ref:`PILOT_STEER_TYPE <PILOT_STEER_TYPE>` to "1".

Once enabled, raising RC input 1 channel high is like controlling the left motor of a skid-steer vehicle so the vehicle will turn right.  Raising RC input channel 3 is like controlling the right motor so the vehicle will turn left.  Raising both channels high will cause the vehicle to move forward, lowering both will cause the vehicle to backup.

.. note::

   Most users of skid steering vehicles should leave the :ref:`PILOT_STEER_TYPE <PILOT_STEER_TYPE>` parameter at the default of "0" meaning the pilot controls the vehicle using one RC input for throttle and another for steering.

Reversing Behaviour
-------------------

ArduPilot Rover's reversing behaviour is just like a full sized car, regardless of whether the vehicle is a regular steering-throttle style vehicle or a skid-steering vehicles.

- if moving forward with wheel turned right, vehicle turns in clockwise direction
- if moving backwards with wheel turned right, vehicles turns in counter-clockwise direction

Some users, especially users of skid-steering vehicles, may wish the vehicle's turning direction to be the same whether moving forward or reversing.  This can be accomplished by setting :ref:`PILOT_STEER_TYPE <PILOT_STEER_TYPE>` to "3"
