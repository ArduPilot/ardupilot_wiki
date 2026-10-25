.. _servo-rc-setup:

==============
SERVO/RC SETUP
==============

Frame Class
===========

Blimp supports two frame types, selected with :ref:`FRAME_CLASS<FRAME_CLASS>` (reboot required after changing):

- :ref:`FRAME_CLASS<FRAME_CLASS>` = 1: flapping fin blimp
- :ref:`FRAME_CLASS<FRAME_CLASS>` = 2: four motor (propeller) blimp

The default of 0 is not a valid frame. With it, a "Bad frame class" error is shown at boot, no outputs are driven, and arming fails with "Check firmware or FRAME_CLASS", so :ref:`FRAME_CLASS<FRAME_CLASS>` must be set before the first flight.

:ref:`FINS_THR_MAX<FINS_THR_MAX>` limits the output of every fin or motor (in both directions) for either frame. The default of 1 allows full output.

Flapping Fin Blimp
==================

The flapping fin Blimp requires four servos for movement and altitude control. Therefore, four outputs on the autopilot will need to be assigned to the Front, Rear, Right, and Left side servos. This is accomplished by setting the ``SERVOx_FUNCTION`` for each output connected to these servos as follows:

==============      ===============
SERVO POSITION      SERVOx_FUNCTION
==============      ===============
REAR                  Motor 33
FRONT                 Motor 34
RIGHT                 Motor 35
LEFT                  Motor 36
==============      ===============

The fins oscillate at :ref:`FINS_FREQ_HZ<FINS_FREQ_HZ>`. :ref:`FINS_TURBO_MODE<FINS_TURBO_MODE>` doubles the oscillation speed when a fin's offset is high.

Four Motor Blimp
================

The four motor blimp uses two forward-facing motors at the front for forward/back movement and yaw, one vertical motor for altitude, and one sideways motor for lateral movement:

==================================    ===============
MOTOR POSITION / DIRECTION            SERVOx_FUNCTION
==================================    ===============
FRONT LEFT (forward thrust)             Motor 33
FRONT RIGHT (forward thrust)            Motor 34
VERTICAL (up/down thrust)               Motor 35
LATERAL (sideways thrust)               Motor 36
==================================    ===============

Each motor is driven in both directions, so reversible (bidirectional) ESCs are needed, with the output's trim (``SERVOx_TRIM``) set to the ESC's stopped (neutral) value. Front left and front right thrust together to move forward or back, and differentially to yaw.

ARMING SWITCH
=============

While it is possible to use :ref:`ARMING_RUDDER<ARMING_RUDDER>` = 2, to allow arming and disarming via the throttle and rudder sticks, this could allow accidental disarming in the air. Therefore, it is better to the use an RC channel, controlled by a two or three position switch, to arm and disarm. This can be accomplished by setting the RC channel's ``RCx_OPTION`` to "153" .
