.. _common-camera-gimbal:

=========================
Camera Gimbal with Servos
=========================

This article explains how to connect a camera gimbal with servos to a
autopilot and configure it for use with ArduPilot (in Mission
Planner).

Overview
========

Copter, Plane or Rover can stabilize a gimbal with up to three axis of
motion using any of the free output channels. Combining stabilization
with input from the pilot is also possible using additional RC channels for manual control of the axes.

The examples shown below use Copter screen shots, but work exactly the
same for Plane and Rover.

A camera's shutter can also be triggered from the autopilot once connected with a
servo or "relay". The act of triggering the shutter can be controlled
through receiver channel or automatically during
missions.

For best performance, ensure that after you add the extra weight of the
camera gimbal your copter still hovers ideally at 50% throttle but
certainly no more than 70% throttle while in Stabilize mode because an
underpowered copter also has less power to recover from disturbances.

Camera mount and build
======================

The camera needs to be mounted securely to the gimbal, but in such a way
that reduces/dampens vibrations from the motor. It is difficult to
achieve both aims at the same time!

Common methods for mounting the camera on the gimbal include using: soft
foam, stiff foam, neoprene tubes (mount camera on tube side), surgical
tube, rubber bands, nylon bolt (direct stiff attachment) and velcro.
Tubes have been used mostly on traditional helicopters. We make no
specific recommendation here as there is no single obvious "best choice"
and to some extent this depends on the airframe. That said, using velcro
and rubber bands (for extra security) is quick to set up and can give
reasonable results.

Connecting the servos to an Autopilot
=====================================

Connect the gimbal's roll, pitch(tilt), and/or yaw stabilization motor control inputs to the autopilot's output pins, as shown below for a Pixhawk.

.. image:: ../../../images/pixhawk_to_gimbal_connection.jpg
    :target: ../_images/pixhawk_to_gimbal_connection.jpg

Gimbal configuration through Mission Planner
============================================

Under the SETUP/Optional Hardware Menu, you will find a ``Camera
Gimbal`` set-up screen. (see image below)

The **Type** box should be set to the type of gimbal. "Servo" for either non-stabilized manual control, or ArduPilot stabilization, or use of an external gimbal stabilizer. The autopilot will need to be rebooted in order for this to take effect and allow changes to the options below.

For each axis (**TILT, ROLL, PAN**) of your camera gimbal select the appropriate output
channel that you connected above and ensure the appropriate **"Stabilise"** checkbox is checked.

.. note:: if only directional control is desired without correcting for vehicle attitude, then leave the **"Stabilise"** checkbox unchecked.

The **Servo Limits** should be adjusted to ensure the gimbal servos
don't bind.

The **Angle Limits** should correspond to the tilt angle of the gimbal
itself at the servo limits. If you find during testing that your gimbal
is not properly remaining stabilised (for example it's over or
undercorrecting as you tilt the copter), adjust the angle limits up or
down slightly.

(These are not really 'angle' limits but how much the servo is commanded
to move within the limits set by the Min/Max parameters set in **Servo Limits**.

eg. If set to -60/+60 the output will reach Min/Max (its limit) when the
'copter reaches -60°/+60°. If set to -15/+15 the servo will reach Min/Max (its limit) when the
'copter reaches -15°/+15°)

**Retract Angles** refer to the position of the gimbal when the
mount's mode is "retracted" (i.e. MNT_MODE=0). "Retracted" normally
means when the gimbal is pulled into the body of the aircraft which is
generally not relevant for multicopters.

**Neutral Angles** refer to the position of the gimbal when the mount
is first initialised. This is normally facing straight forward.

**Control Angles** are parameters to allow control of the gimbal from
a ground station perhaps using a joystick. These values are overwritten
by the ground station so there's no point in updating them on the MP
screen.

If you find your gimbal is moving in the wrong direction, check the
**Reverse** checkbox.

.. figure:: ../../../images/MPCameraAndGimbalSetupScreen.jpg
   :target: ../_images/MPCameraAndGimbalSetupScreen.jpg

   Mission Planner: Camera andGimbal Setup Screen

If you wish to adjust the gimbal tilt, roll or pan while flying, you can
set the :ref:`MNT_RC_IN_PAN<MNT_RC_IN_PAN>`, :ref:`MNT_RC_IN_ROLL<MNT_RC_IN_ROLL>`, and/or :ref:`MNT_RC_IN_TILT<MNT_RC_IN_TILT>` to the RC channel to be used for this. Be sure that those channels do not have any ``RCx_OPTION`` also assigned.

.. tip:: if only a manual pan, and or tilt is desired using servos, you can use this setup without stabilizaton. In addition, by extending the ``SERVO LIMITS`` you can sometimes obtain almost 180 degree servo rotation, on some servos. Increase them slowly, testing as you do, and do not over extend the PWM values. Stop when motion stops increasing.

Aligning Min and Max PWM values with full throw of gimbal
---------------------------------------------------------

This section shows how to align the maximum and minimum PWM servo
settings:

-  Tilt airframe over hard left ( just past where the servo stops
   moving, or ~45 deg ), and raise the "Roll" "Servo" "Min" value until
   the servo starts to physically move a tiny bit, stop there.
-  Tilt airframe over hard right ( just past where the servo stops
   moving, or ~45 deg ), and lower the "Roll" "Servo" "Max" value until
   the servo starts to physically move a tiny bit, stop there.

-  Repeat for Pitch ( forward and backward motion)

Leveling/centering the gimbal
-----------------------------

To level and centre the gimbal:

-  Keep the airframe perfectly straight-and-level
-  If the gimbal is not quite perfectly level, tweak the hardware first,
   eg, get servo horn/s so that gimbal is as close to level as possible
   before doing next step/s .. do this by unscrewing horn from servo and
   repositioning it, and/or if using push-rods to the gimbal, by
   adjusting the length of them).
-  If "tilt" is still not quite level, you can "trim" it by adjusting
   the Tilt->Angle->Min and Tilt->Angle->Max ... BOTH by one click in
   the same direction ( eg, click both down arrows once each) This will
   ensure that the difference between them remains constant ( important
   ), but will adjust the "centre" position of the gimbal by small
   amounts ( do not do this too much as it affects the maximum throw/s
   at the extremeties by the same amount).

Common fixes for poor video
===========================

Some of the more common causes and solutions for poor video are listed
below:

-  "Jello" effect (or rolling shutter) is a by-product of using a camera
   with a CMOS sensor (GoPro, et al) caused by vibration from unbalanced
   props/motors and can be mitigated by mounting the camera on soft
   rubber, silcone, foam ear plugs or sometimes just on velcro.
-  digital and optical stabilization systems found in many cameras often
   do not perform well because of the vibrations found on many
   multicopters.

   -  Exceptions: the Sony video camera balanced steady shot system is
      very effective even at maximum 30 power zoom.

-  For better and smoother Yaw, use Expo control on your RC and lower
   the Acro_Yaw_P gain in the autopilot.

It is important to remember that even with a perfect setup, photography
is an art as well as a science. Using the camera pointing straight to
ground is a good place to start, but more dramatic viewpoints can be
achieved with angles other than vertical. Mount about 40 degrees
deviation from vertical to obtain mainly ground photos but with oblique
view. About 70 degrees off vertical will give you a lot more sky giving
scenic photos. ArduPilot will stabilise the gimbal to whatever position you set.

Shutter configuration
=====================

See :ref:`Camera Shutter Configuration in Mission Planner <common-camera-shutter-with-servo>` for information on how to integrate shutter triggering with ArduPilot.

Camera Mount Mode/Targeting
===========================

The camera/gimbal direction can be controlled by the pilot using RC control(RC Targeting) if RC channels for control have been assigned (default on startup unless changed), by the autopilot during missions using the DO_SET_ROI or DO_MNT_CONTROL commands (GPS and MAVLink Targeting), not at all (just stabilizing and set to a given angle on the axes, called NEUTRAL), or when RETRACTED if a retractable mount is used to rotate the camera as it retracts for clearance.

If a retractable mount is employed, the overall mount may be deployed or retracted using an output assigned with ``SERVOx_FUNCTION`` set to "MountOpen". This will be automatically controlled by the autopilot as if it were landing gear (see :ref:`common-landing-gear`), or by pilot using an rc channel whose ``RCx_OPTION`` is set to "Landing Gear".

The default targeting mode for the camera/gimbal is set by the :ref:`MNT_DEFLT_MODE<MNT_DEFLT_MODE>` parameter.

The direction the axes are set for the NEUTRAL and RETRACTED modes are set by:

- :ref:`MNT_NEUTRAL_X<MNT_NEUTRAL_X>`
- :ref:`MNT_NEUTRAL_Y<MNT_NEUTRAL_Y>`
- :ref:`MNT_NEUTRAL_Z<MNT_NEUTRAL_Z>`
- :ref:`MNT_RETRACT_X<MNT_RETRACT_X>`
- :ref:`MNT_RETRACT_Y<MNT_RETRACT_Y>`
- :ref:`MNT_RETRACT_Z<MNT_RETRACT_Z>`

Other Parameters
================

Since servos in the gimbal may react slower to position/angle changes in the vehicle's roll and pitch as the vehicle moves about a target, the camera shot may have some visible lag in it. This can be reduced by using these parameters to have the gimbal outputs move a bit ahead of the movements of the vehicle.

- :ref:`MNT_LEAD_RLL<MNT_LEAD_RLL>`
- :ref:`MNT_LEAD_PTCH<MNT_LEAD_PTCH>`