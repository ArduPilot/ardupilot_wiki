.. _balance_bot-modes:

=========================
Balance Bot Control Modes
=========================
Some Rover Control Mode work slightly different in Balance Bots. This section explains some of the important differences to be kept in mind while using these modes. For more information on any mode, check the :ref:`Control Modes<rover-control-modes>` page for Rover.

Hold/Manual
-----------
Hold and Manual Mode in Balance Bots use a Pitch Control System to control and maintain pitch angle. In Hold Mode, the input pitch to the control system is set to 0 degrees, to keep the Vehicle upright(equivalent to zero throttle input). In Manual Mode the input throttle from the user is mapped to a pitch angle and sent to the Control System. The control system in turn calculates the required throttle and sends it to the motors. As explained in the :ref:`working <balance_bot-working>` section, this causes the vehicle to accelerate in the direction of pitch. The steering input from the user is handled the same way as in Rover.

.. image:: /images/balance_bot-manual.png


In Manual Mode, continuous acceleration in one direction can saturate the motors, as they reach maximum speed. The vehicle will not be able to balance as further acceleration is not possible. This is the disadvantage of running Balance Bots in Manual Mode. 

In Manual and Hold modes, we can only control the pitch angle of the vehicle, not the velocity. It is possible to be at zero pitch angle(upright position) and have a non zero velocity. To stop the vehicle from drifting in Manual mode, it must be made to pitch in the reverse direction. If these modes are entered with a non zero initial speed(from Acro or Auto modes), the vehicle can accelerate and crash.

Acro
----
This is the recommended drive mode for Balance Bots.

Acro Mode lets you control the speed by internally controlling the pitch angle. Seperately, it also allows you to control the the turn rate.

.. image:: /images/balance_bot-acro.png

Acro mode uses a speed and turn rate control system. In Rover the output from the Acro control system goes straight to the motors. In Balance Bots, the output from the Speed Controller is mapped to a desired pitch and sent to the Pitch Controller used in Manual/Hold Mode.


Guided/Auto/RTL
---------------
All of these modes are implemented as they are in Rover. No Balance Bot specific differences exist. Instructions on configuring and using these modes can be found in the :ref:`Rover Control Modes page<rover-control-modes>`.