.. _modes:

=========
Sub Modes
=========
Sub has the following modes:

legend for requirements: P - needs Position(GPS,etc.), D - needs Depth sensor, R - needs Rangefinder

=============   =================================================  =========
Mode            Description                                        Requires
=============   =================================================  =========
MANUAL          Pilot control with no stabilization                 \-
ACRO            Body-frame rate control, manual depth control       \-
STABILIZE       Manual angle control, manual depth control          \-
ALT_HOLD        Stabilize with automatic depth control              D
AUTO            Automatic mission command list execution            P/D
GUIDED          Swim to location or velocity/direction using GCS    P/D
CIRCLE          Circle swim with depth control                      P/D
SURFACE         Return to surface, pilot directional control        \-
POSHOLD         Loiter with depth control and pilot overrides       P/D
MOTOR_DETECT    Automatically determine motor rotation and adjust   \-
SURFTRAK        Hold distance above seafloor while stabilizing      R
=============   =================================================  =========

See :ref:`Pilot Control <pilot-control>` for more details on modes.

GUIDED Mode Targets
===================

In GUIDED mode the vehicle is commanded by a GCS, companion computer or Lua script. Position, velocity and acceleration targets are sent using `SET_POSITION_TARGET_LOCAL_NED <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`__ or `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`__, with the ``type_mask`` field selecting which of the three are being supplied. The accepted combinations are:

- position only
- velocity only
- position and velocity
- position, velocity and acceleration

Acceleration is used only when it is accompanied by both position and velocity; it is ignored in any other combination. The supplied acceleration is fed forward into the position controller, which lets an external controller or script command a smoothly changing trajectory instead of a series of steps.

Lua scripts can send the same targets with ``vehicle:set_target_posvelaccel_NED()``, and can offset the vehicle's current target using the ``poscontrol`` bindings, ``poscontrol:set_posvelaccel_offset()`` and ``poscontrol:get_posvelaccel_offset()``. The ``guided_above_terrain_posvelaccel_sub.lua`` example script uses these to swim at a constant forward speed while holding a set height above the seafloor.

Mode Specific Parameters
========================

ACRO Mode
---------
* :ref:`ACRO_RP_P<ACRO_RP_P>`
* :ref:`ACRO_YAW_P<ACRO_YAW_P>`
* :ref:`ACRO_BAL_ROLL<ACRO_BAL_ROLL>`
* :ref:`ACRO_BAL_PITCH<ACRO_BAL_PITCH>`
* :ref:`ACRO_TRAINER<ACRO_TRAINER>`
* :ref:`ACRO_EXPO<ACRO_EXPO>`

Stabilization Related
---------------------
* :ref:`ATC_ANGLE_MAX<ATC_ANGLE_MAX>`
* :ref:`JS_GAIN_DEFAULT<JS_GAIN_DEFAULT>`

ALT_HOLD Mode
-------------
* :ref:`SURFACE_MAX_THR<SURFACE_MAX_THR>`

Depth Control Related
---------------------
* :ref:`SURFACE_DEPTH<SURFACE_DEPTH>`
* :ref:`PILOT_SPD_UP<PILOT_SPD_UP>`
* :ref:`PILOT_SPD_DN<PILOT_SPD_DN>`
* :ref:`PILOT_SPEED<PILOT_SPEED>`

Auto/Guided Waypoint Navigation Related
---------------------------------------
* :ref:`WP_SPD<WP_SPD>`
* :ref:`WP_RADIUS_M<WP_RADIUS_M>`
* :ref:`WP_SPD_UP<WP_SPD_UP>`
* :ref:`WP_SPD_DN<WP_SPD_DN>`
* :ref:`WP_ACC<WP_ACC>`
* :ref:`WP_ACC_Z<WP_ACC_Z>`
* :ref:`WP_RFND_USE<WP_RFND_USE>`
* :ref:`WP_YAW_BEHAVIOR<WP_YAW_BEHAVIOR>`

CIRCLE Mode
-----------
* :ref:`CIRCLE_RADIUS_M<CIRCLE_RADIUS_M>`
* :ref:`CIRCLE_RATE<CIRCLE_RATE>`
* :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>`

Mode Selection
--------------
By default modes are controlled via MAVLink command from the GCS or companion computer, or by Joystick Buttons. But RC control can be enabled by setting:

* :ref:`RC_PROTOCOLS<RC_PROTOCOLS>`  not equal to zero ("1" is usually used)
* :ref:`FLTMODE_CH<FLTMODE_CH>`   to the RC channel which will control swim mode selection
* ``FLTMODE1 - FLTMODE6`` to desired Swim Mode
