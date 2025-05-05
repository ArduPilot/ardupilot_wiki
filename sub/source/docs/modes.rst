.. _modes:

Sub Modes
=========
Sub has the following modes:

legend for requirements: P - needs Position(GPS,etc.), D - needs Depth sensor, R - needs Rangefinder

=============   =================================================  =========
Mode            Description                                        Requires
=============   =================================================  =========
MANUAL          Pilot control with no stabilization                 -
ACRO            Body-frame rate control, manual depth control       -
STABILIZE       Manual angle control, manual depth control          -
ALT_HOLD        Stabilize with automatic depth control              D
AUTO            Automatic mission command list execution            P/D
GUIDED          Swim to location or velocity/direction using GCS    P/D
CIRCLE          Circle swim with depth control                      P/D
SURFACE         Return to surface, pilot directional control        P/D
POSHOLD         Loiter with depth control and pilot overrides       P/D
MOTOR_DETECT    Automatically determine motor rotation and adjust   D
SURFTRAK        Hold distance above seafloor while stabilizing      R
=============   =================================================  =========

See Pilot Control(to be added) for more details on modes.

Mode Specific Parameters
========================

CIRCLE Mode
-----------
* :ref:`CIRCLE_RADIUS<CIRCLE_RADIUS>`
* :ref:`CIRCLE_RATE<CIRCLE_RATE>`
* :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>`

Depth Control Related
---------------------
* :ref:`SURFACE_DEPTH<SURFACE_DEPTH>`
* :ref:`PILOT_SPEED_UP<PILOT_SPEED_UP>`
* :ref:`PILOT_SPEED_DN<PILOT_SPEED_DN>`
* :ref:`PILOT_SPEED<PILOT_SPEED>`

Stabilization Related
---------------------
* :ref:`ANGLE_MAX<ANGLE_MAX>`
* :ref:`JS_GAIN_DEFAULT<JS_GAIN_DEFAULT>`

Auto/Guided Waypoint Navigation Related
---------------------------------------
* :ref:`WPNAV_SPEED<WPNAV_SPEED>`
* :ref:`WPNAV_RADIUS<WPNAV_RADIUS>`
* :ref:`WPNAV_SPEED_UP<WPNAV_SPEED_UP>`
* :ref:`WPNAV_SPEED_DN<WPNAV_SPEED_DN>`
* :ref:`WPNAV_ACCEL<WPNAV_ACCEL>`
* :ref:`WPNAV_ACCEL_Z<WPNAV_ACCEL_Z>`
* :ref:`WPNAV_RFND_USE<WPNAV_RFND_USE>`
* :ref:`WP_YAW_BEHAVIOR<WP_YAW_BEHAVIOR>`

Mode Selection
--------------
By default modes are controlled via MAVLink command from the GCS or companion computer, or by Joystick Buttons. But RC control can be enabled by setting:

* :ref:`RC_PROTOCOLS<RC_PROTOCOLS>`  not equal to zero ("1" is usually used)
* :ref:`FLTMODE_CH<FLTMODE_CH>`   to the RC channel which will control swim mode selection
* ``FLTMODE1 - FLTMODE6`` to desired Swim Mode
