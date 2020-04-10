.. _quadplane-frame-setup:

QuadPlane Frame setup
=====================

The QuadPlane code supports several frame arrangements of quadcopter,
hexacopter, octacopter and octaquad multicopter frames.

The motor order and output channel is the same as for copter (see :ref:`Copter motor layout <copter:connect-escs-and-motors>`)
except that the default output channel numbers start at 5 instead of 1.

.. note:: :ref:`guide-tailsitter` configuration is a special case. See Tailsitter notes below

For example, with the default Quad-X frame the motors are on outputs
5 to 8. The arrangement is:

-  **Channel 5:** Front right motor, counter-clockwise
-  **Channel 6:** Rear left motor, counter-clockwise
-  **Channel 7:** Front left motor, clockwise
-  **Channel 8:** Rear right motor, clockwise

You can remember the clockwise/counter-clockwise rule by "motors turn
in towards the fuselage", except for the H configuration, there all directions are inverted!
   
Another common setup is an octa-quad, which uses the following ordering

-  **Channel 5:** Front right top motor, counter-clockwise
-  **Channel 6:** Front left top motor, clockwise
-  **Channel 7:** Rear left top motor, counter-clockwise
-  **Channel 8:** Rear right top motor, clockwise
-  **Channel 9:** Front left bottom motor, counter-clockwise
-  **Channel 10:** Front right bottom motor, clockwise
-  **Channel 11:** Rear right bottom motor, counter-clockwise
-  **Channel 12:** Rear left bottom motor, clockwise

You can remember the clockwise/counter-clockwise rule for an octa-quad
by "top motors turn in towards the fuselage, bottom motors turn out
away from the fuselage".
   
The normal plane outputs are assumed to be on 1 to 4 as usual. Only
vertical lift outputs (5 to 8 on a quad setup) run at high PWM rate
(400Hz). In a quad setup you can also use channels 9 to 14 in any way
you like, just as with the normal Plane code.

You can optionally move the quad motors to be on any other channel above
4, using the procedure outlined below.

To use a different frame type you can set :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` and
:ref:`Q_FRAME_TYPE<Q_FRAME_TYPE>` . :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` can be:

-  1 for quad
-  2 for hexa
-  3 for octa
-  4 for octaquad
-  5 for Y6
-  7 for Tri
-  10 for Tailsitter

Within each of these frame classes the :ref:`Q_FRAME_TYPE<Q_FRAME_TYPE>` chooses the motor
layout

-  0 for plus frame
-  1 for X frame
-  2 for V frame
-  3 for H frame
-  11 for FireFly6Y6 (for Y6 only)

Tailsitter
----------

Tailsitters (:ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` =10) are a special case. Since this can be used on a conventional single motor plane for "3D" style flying, the standard single motor configuration with the Throttle (motor) output on SERVO3 is automatically configured. If you are using twin engines, then you may want to manually configure two outputs for Throttle Left and Throttle Right, instead of Throttle, if you want differential thrust. If you are using a twin, vectored thrust tailsitter, then you will need to manually configure four outputs for Throttle Left, Throttle Right, Left Motor Tilt and Right Motor Tilt. See :ref:`Tailsitter <guide-tailsitter>` section for more information


Using different channel mappings
--------------------------------

You can remap what output channels the quad motors are on by setting
values for SERVOn_FUNCTION. This follows the same approach as :ref:`other output functions <channel-output-functions>`.

.. note::
   Note that you do not need to set any of the SERVOn_FUNCTION values unless
   you have a non-standard motor ordering, using vectored thrust, or are a Tailsitter. It is highly recommended that
   you use the standard ordering and do not set the SERVOn_FUNCTION
   parameters, leaving them at zero. They will be automatically set to
   the right values for your frame on boot.

The output function numbers are:

-  33: motor1
-  34: motor2
-  35: motor3
-  36: motor4
-  37: motor5
-  38: motor6
-  39: motor7
-  40: motor8

So to put your quad motors on outputs 9 to 12 (the auxillary channels on
a Pixhawk) you would use these settings in the advanced parameter list:

-  :ref:`SERVO9_FUNCTION<SERVO9_FUNCTION>` = 33
-  :ref:`SERVO10_FUNCTION<SERVO10_FUNCTION>` = 34
-  :ref:`SERVO11_FUNCTION<SERVO11_FUNCTION>` = 35
-  :ref:`SERVO12_FUNCTION<SERVO12_FUNCTION>` = 36

