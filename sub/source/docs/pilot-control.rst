.. _pilot-control:

=============
Pilot Control
=============

Control of the vehicle can be accomplished in three ways:

- Via Ground Control Stations using:
   - GCS attached Joysticks, or command lines, to send MAVLink `RC override <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`__ commands that act as RC inputs (even if no RC receiver is attached)
   - MAVLink ` MANUAL CONTROL <https://mavlink.io/en/messages/common.html#MANUAL_CONTROL>`__ messages which includes axis commands and button states.

- RC control (not enabled by default)

Pilot Control Inputs
====================
If RC is not enabled, pilot inputs are via the GCS joystick, controlling the vehicle's available motion axes, including yaw and attitude rotations, along with body-frame translations laterally, forward/back, and up/down in depth. Additionally, lights and camera can be controlled from the GCS, as well as some generic servo and/or relay outputs.

If RC is enabled, then the pitch/roll/yaw/throttle/lateral/forward inputs are sent from the RC transmitter and mapped from RC channels to these inputs via the ``RCMAP_`` parameters.

Pilot Control Freedoms by Mode
==============================
The following assumes the axis is controllable. Some frame configurations do not provide all axes. For example the SimpleROV-3 frame has three thrusters providing only yaw, forwad, and depth control. The pitch and roll attitude is uncontrolled and relies on CG/ballast trimming and has no lateral movement capability.

ANG = Angle target. Stick deflection indicates desired axis angle.

RAT = Rate target. Stick deflection indicates desired axis movement/rotation rate.

D = Direct motor output based on stick deflection.

=========== ===== ===== ==== ===== ======= ======= ======= =========
Mode        Roll  Pitch Yaw  Depth Forward Lateral PosHold DepthHold
=========== ===== ===== ==== ===== ======= ======= ======= =========
MANUAL       D     D     D     D     D       D       No      No
ACRO         RAT   RAT   RAT   RAT   RAT     RAT     No      No
STABILIZE    ANG   ANG   RAT   RAT   RAT     RAT     No      No
ALT_HOLD     ANG   ANG   RAT   RA    RAT     RAT     No      Yes
SURFTRAK     ANG   ANG   RAT   RAT   RAT     RAT     No      Yes*
POSHOLD      ANG   ANG   RAT   RAT   RAT     RAT     Yes     Yes
SURFACE      ANG   ANG   RAT   -     RAT     RAT     No      No
CIRCLE       ANG   ANG   RAT   RAT   -       -       Yes     Yes
MOTORDETECT  -     -     -     -     -       -        -       -
=========== ===== ===== ==== ===== ======= ======= ======= =========

* SURFTRAK maintains depth as a relative position above the sea floor using a rangefinder when there in no depth input change requested by the pilot. 

- MODE AUTO: Autopilot commands necessary controls to navigate the mission.
- MODE GUIDED: Pilot can control Roll and Pitch if in velocity control mode with stick input signifying angle targets if in HEADING HOLD, or rate targets, otherwise. Yaw input is rate demand. LATERAL and FORWARD inputs ignored.

Failsafes
=========

Several failsafes are provided for pilot control methods, if they fail, any or all of which may be configured and enabled:

- :ref:`Pilot Control failsafe <pilot-control-failsafe>`
- :ref:`RC Failsafe <radio-failsafe>` (if RC is enabled)
- :ref:`GCS Failsafe <gcs-failsafe>`
