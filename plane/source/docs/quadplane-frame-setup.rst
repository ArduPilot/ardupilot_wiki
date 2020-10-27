.. _quadplane-frame-setup:

=====================
QuadPlane Frame setup
=====================

The QuadPlane code supports several frame arrangements of quadcopter,
hexacopter, octacopter and octaquad multicopter frames which use lifting motors in addition to the normal forward motor(s). Also configurations in which the VTOL motors tilt for transitions or control, as well as choice between horizontal VTOL stance or vertical (Tailsitters).


Tailsitters
===========

Frame setup for Tailsitters is in this section: :ref:`guide-tailsitter`. Once the frame is configured, proceed with the other :ref:`quadplane-setup` instructions.

Plane VTOL Stance
=================

These configurations add multicopter style lifting motors to a conventional fixed wing configuration. Some or all of these motors may also be configured as tilting motors to be used in fixed wing flight.

Frame Types and Classes
-----------------------

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
layout. For Tri and Y6 this parameter is ignored.

-  0 for plus frame
-  1 for X frame
-  2 for V frame
-  3 for H frame
-  11 for FireFly6Y6 (for Y6 only)

Motor Ordering
--------------

The motor order and output channel is the same as for copter (see :ref:`Copter motor layout <copter:connect-escs-and-motors>`)
except that the default output channel numbers start at 5 instead of 1.

.. note:: :ref:`guide-tailsitter` configuration is a special case. See Tailsitter notes below

For example, with the default Quad-X frame the motors are on outputs
5 to 8. The arrangement is:

-  **Output 5:** Front right motor, counter-clockwise
-  **Output 6:** Rear left motor, counter-clockwise
-  **Output 7:** Front left motor, clockwise
-  **Output 8:** Rear right motor, clockwise

You can remember the clockwise/counter-clockwise rule by "motors turn
in towards the fuselage", except for the H configuration, there all directions are inverted!
   
Another common setup is an octa-quad, which uses the following ordering

-  **Output 5:** Front right top motor, counter-clockwise
-  **Output 6:** Front left top motor, clockwise
-  **Output 7:** Rear left top motor, counter-clockwise
-  **Output 8:** Rear right top motor, clockwise
-  **Output 9:** Front left bottom motor, counter-clockwise
-  **Output 10:** Front right bottom motor, clockwise
-  **Output 11:** Rear right bottom motor, counter-clockwise
-  **Output 12:** Rear left bottom motor, clockwise

You can remember the clockwise/counter-clockwise rule for an octa-quad
by "top motors turn in towards the fuselage, bottom motors turn out
away from the fuselage".

The normal plane outputs are assumed to be on 1 to 4 as usual. Only
vertical lift outputs (5 to 8 on a quad setup) run at high PWM rate
(400Hz). In a quad setup you can also use channels 9 to 14 in any way
you like, just as with the normal Plane code.

You can optionally move the quad motors to be on any other channel above
4, using the procedure outlined in the section further below.

Tricopter
---------

Frame Type 7 is Tricopter and can be either non-Tiltrotor configuration, or :ref:`Tiltrotor<guide-tilt-rotor>` configured using either Vectored or Non-Vectored yaw control. If using non-Tiltrotor or Non-Vectored Yaw Tilt-rotor, the yaw control output is setup as Motor 7 (``SERVOn_FUNCTION`` = 39) using a tilt mechanism for the yaw motor, Motor 4. You should set up the yaw servo’s maximum lean angle in degrees with :ref:`Q_M_YAW_SV_ANGLE<Q_M_YAW_SV_ANGLE>`. This lean angle assumes that ``SERVOn_MIN`` and ``SERVOn_MAX``, represent +/- 90 degrees, with ``SERVOn_TRIM`` representing 0 degrees lean.


Tilt-Rotors
===========

See :ref:`guide-tilt-rotor`

Using different channel mappings
================================

You can remap what output channels the lifting motors are on by setting
values for SERVOn_FUNCTION. This follows the same approach as :ref:`other output functions <common-rcoutput-mapping>`.

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

