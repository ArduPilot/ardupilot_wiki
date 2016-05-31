.. _quadplane-frame-setup:

QuadPlane Frame setup
=====================

The QuadPlane code supports several frame arrangements of quadcopter,
hexacopter, octacopter and octaquad multicopter frames.

The motor order and output channel is the same as for copter (see :ref:`Copter motor layout <copter:connect-escs-and-motors>`)
except that the output channel numbers start at 5 instead of 1.

For example, with the default Quad-X frame the motors are on outputs
5 to 8. The arrangement is:

-  **Channel 5:** Front right motor, counter-clockwise
-  **Channel 6:** Rear left motor, counter-clockwise
-  **Channel 7:** Front left motor, clockwise
-  **Channel 8:** Rear right motor, clockwise

You can remember the clockwise/counter-clockwise rule by "motors turn
in towards the fuselage".
   
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

To use a different frame type you can set Q_FRAME_CLASS and
Q_FRAME_TYPE. Q_FRAME_CLASS can be:

-  0 for quad
-  1 for hexa
-  2 for octa
-  3 for octaquad

Within each of these frame classes the Q_FRAME_TYPE chooses the motor
layout

-  0 for plus frame
-  1 for X frame
-  2 for V frame
-  3 for H frame

Using different channel mappings
--------------------------------

You can remap what output channels the quad motors are on by setting
values for RCn_FUNCTION. This follows the same approach as :ref:`other output functions <channel-output-functions>`.

.. note::

   Note that you do not need to set any of the RCn_FUNCTION values unless
   you have a non-standard motor ordering. It is highly recommended that
   you use the standard ordering and do not set the RCn_FUNCTION
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

-  RC9_FUNCTION = 33
-  RC10_FUNCTION = 34
-  RC11_FUNCTION = 35
-  RC12_FUNCTION = 36
