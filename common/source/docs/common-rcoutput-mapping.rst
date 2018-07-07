.. _common-rcoutput-mapping:

=================
RC Output Mapping
=================

This page describes how the RC output channels used for motors (and other features) can be changed from their defaults.
These instructions focus mainly on the multicopter motors because individual feature setup pages (like the :ref:`Servo Gripper <common-gripper-servo>` setup page)
normally document how to assign the output channel used using RCx_FUNCTION or SERVOx_FUNCTION parameters.

.. note::

   The ability to remap multicopter motors is available in Copter-3.4 (and higher).
   The parameters used to control the output channels has changed from RCx_FUNCTION to SERVOx_FUNCTION in Copter-3.5 and higher.

Configuration
=============

Depending upon the firmware version either the RCx_FUNCTION or SERVOx_FUNCTION can be set to any of `85+ different values <https://github.com/ArduPilot/ardupilot/blob/master/libraries/SRV_Channel/SRV_Channel.h#L40>`__
to specify what the output channel should be used for (Note: most ground stations should display the entire list of possible options).
The "x" in this case is the channel number so RC1_FUNCTION or :ref:`SERVO1_FUNCTION <SERVO1_FUNCTION>` assigns what "MAIN OUT 1" pin on the back of a Pixhawk is used for.
:ref:`RC9_FUNCTION <RC9_FUNCTION>` or :ref:`SERVO9_FUNCTION <SERVO9_FUNCTION>` assigns what the "AUX OUT1" pin is used for.

.. image:: ../../../images/rcoutput-mapping.png
    :target: ../_images/rcoutput-mapping.png

To assign multicopter motors these function values can be used:

- 33: motor #1
- 34: motor #2
- 35: motor #3
- 36: motor #4
- 37: motor #5
- 38: motor #6
- 39: motor #7
- 40: motor #8
- 82: motor #9
- 83: motor #10
- 84: motor #11
- 85: motor #12

For example, if you wished to re-order a quad-x frame's motors from the :ref:`kitty-corner default <connect-escs-and-motors>` to a more logical clockwise method, make these changes:

- :ref:`SERVO1_FUNCTION <SERVO1_FUNCTION>` leave as 33 (aka "motor1", front-right)
- :ref:`SERVO2_FUNCTION <SERVO2_FUNCTION>` change from 34 (aka "motor2", back-left) to 36 (motor #4, back-right)
- :ref:`SERVO3_FUNCTION <SERVO3_FUNCTION>` change from 35 (aka "motor3", front-left) to 34 (motor #2, back-left)
- :ref:`SERVO4_FUNCTION <SERVO4_FUNCTION>` change from 36 (aka "motor4", back-right) to 35 (motor #3, front-left)

.. note::

   It is only possible to modify the output channel used, it is not possible to redefine the direction the motor spins with these paramaters.
   Copter-3.5 (and earlier) do not support assigning the same function to multiple output channels but this feature will be present in Copter-3.6 (and higher).
