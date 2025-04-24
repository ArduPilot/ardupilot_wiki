.. _sub-hardware:

==================================
Default Sub Hardware Configuration
==================================

While the Sub firmware is currently being generalized to allow more flexibility, such as RC control, re-arrangement of outputs and inputs, etc.,
current code is targeted for a specific ROV configuration with the following hardware setup:

- Control is via a surface tether from a ground control station via MAVLink commands to the Sub's autopilot telemetry port, usually employing a joystick and multiple buttons for mode changes and peripheral hardware control. Control functions and autopilot outputs for thrusters and peripheral hardware have specific defaults (see :ref:`Sub defaults <sub-defaults>` below)
- A depth sensor (underwater barometer) is required.
- Frame configuration is defaulted to Vectored with 6 thrusters.

.. _sub-defaults:

Sub Defaults
============
* Default :ref:`frame type <FRAME_TYPE>` is VECTORED
* Radio control is disabled via :ref:`RC_PROTOCOLS<RC_PROTOCOLS>` = 0 and :ref:`FLTMODE_CH<FLTMODE_CH>` = 0. Water absorbs radio waves, so RC is generally impractical unless the vehicle is at the surface, or the signal is converted to some other medium in between (e.g. using a tether cable, or acoustic modems).

MAVLink Overrides
-----------------
ArduPilot vehicles normally use radio control (RC) to provide pilot directional control. Since Sub has RC disabled by default, it instead provides directional control with MAVLink RC overrides, and the channel mapping is fixed:

===============  ===================
Pilot control    RC channel override
===============  ===================
Roll             1
Pitch            0
Yaw(Turn)        3
Vertical(Heave)  2
Forward(Surge)   4
Lateral(Sway)    5
Camera Pan       6 (via AUX function 214 on RC7)*
Camera Tilt      7 (via AUX function 213 on RC8)*
Lights1          8
Lights2          9
Video Switch     10
===============  ===================

\*function depends on type of mount used and can be either angle control or rate control depending on :ref:`MNT1_RC_RATE<MNT1_RC_RATE>` parameter (default for SUB is 30 deg/s maximum by default)

Output Assignments
------------------
The default output assignments are:

======       ========
Output       Function
======       ========
1            Motor1
2            Motor2
3            Motor3
4            Motor4
5            Motor5
6            Motor6
9            RCin9
10           Mount1 Pitch
======       ========

Button Assignments
------------------

Default Sub uses GCS joystick buttons to implement mode switching, light activation, etc.

======      ========
Button      Function
======      ========
1           Mode Manual
2           Mode Depth_Hold
3           Mode Stabilize
4           Disarm
5           Shift Function (when pressed buttons provide alternate second function)
6           Arm
7           Center Mount1
8           Joystick Input Hold Set
9           Tilt Mount1 Down/Pan Mount Left
10          Tilt Mount1 Up/Pan Mount Right
11          Joystick Sensitivity Gain Increase/Trim Pitch Decrease
12          Joystick Sensitivity Gain Decrease/Trim Pitch Increase
13          Lights1 Dimmer/Trim Roll Decrease
14          Lights1 Brighter/Trim Roll Increase
======      ========


