.. _servo-rc-setup:

==============
SERVO/RC SETUP
==============

The flapping fin Blimp requires for servos for movement and altitude control. Therefore, four outputs on the autopilot will need to be assigned to the Front,Rear, Right, and Left side servos. This is accomplished by setting the ``SERVOx_FUNCTION`` for each output connected to these servos as follows:

==============      ===============
SERVO POSITION      SERVOx_FUNCTION
==============      ===============
REAR                  Motor 33
FRONT                 Motor 34
RIGHT                 Motor 35
LEFT                  Motor 36
==============      ===============


ARMING SWITCH
=============

While it is possible to use :ref:`ARMING_RUDDER<ARMING_RUDDER>` = 2, to allow arming and disarming via the throttle and rudder sticks, this could allow accidental disarming in the air. Therefore, it is better to the use an RC channel, controlled by a two or three position switch, to arm and disarm. This can be accomplished by setting the RC channel's ``RCx_OPTION`` to "153" .