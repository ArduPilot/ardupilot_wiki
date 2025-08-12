.. _pilot-control:

=============
Pilot Control
=============

Control of the vehicle can be accomplished in three ways:

- Via Ground Control Stations using:

  - GCS attached Joysticks, or command lines, to send MAVLink `RC override <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`__ commands that act as RC inputs (even if no RC receiver is attached)
  - MAVLink `MANUAL CONTROL <https://mavlink.io/en/messages/common.html#MANUAL_CONTROL>`__ messages which includes axis commands and button states.
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
SURFACE      ANG   ANG   RAT   \-    RAT     RAT     No      No
CIRCLE       ANG   ANG   RAT   RAT   \-      \-      Yes     Yes
MOTORDETECT  \-    \-    \-    \-    \-      \-      \-      \-
=========== ===== ===== ==== ===== ======= ======= ======= =========

\*SURFTRAK maintains depth as a relative position above the sea floor using a rangefinder when there in no depth input change requested by the pilot. 

- MODE AUTO: Autopilot commands necessary controls to navigate the mission.
- MODE GUIDED: Pilot can control Roll and Pitch if in velocity control mode with stick input signifying angle targets if in HEADING HOLD, or rate targets, otherwise. Yaw input is rate demand. LATERAL and FORWARD inputs ignored.

Failsafes
=========

Several failsafes are provided for pilot control methods, if they fail, any or all of which may be configured and enabled:

- :ref:`Pilot Control failsafe <pilot-control-failsafe>`
- :ref:`RC Failsafe <radio-failsafe>` (if RC is enabled)
- :ref:`GCS (Heartbeat) Failsafe <gcs-failsafe>`

Responsiveness
==============
Depending on vehicle design and pilot experience, it can be desirable to limit how quickly and/or strongly the vehicle responds to control inputs. There are a few different approaches to doing this, which affect different parts of the control pipeline:

- If a GCS joystick is used, the ``JS_GAIN_*`` parameters scale the pilot input range, which determines  how much pilot joystick inputs can affect motor outputs

  - In MANUAL mode there are only pilot inputs, so the total output range (and hence power usage) of the vehicle is determined by the pilot's input control range, whether GCS joystick based or RC based
  - In modes with autopilot stabilisation, stabilisation can still cause the vehicle to use its full power capacity (e.g. when operating in strong currents)
  - Joystick gain can be controlled by the pilot using ``gain_*`` :ref:`buttons`, and/or can be configured with the parameters

    - :ref:`JS_GAIN_MIN<JS_GAIN_MIN>`, :ref:`JS_GAIN_MAX<JS_GAIN_MAX>`, and :ref:`JS_GAIN_DEFAULT<JS_GAIN_DEFAULT>` for its limits and starting value
    - :ref:`JS_THR_GAIN<JS_THR_GAIN>` for an additional scaling applied to vertical inputs, to either reduce them further or boost them relative to horizontal/turning inputs
    - :ref:`JS_GAIN_STEPS<JS_GAIN_STEPS>` to determine how much the gain changes with each increment/decrement button press
- When using radio channels for RC control of the vehicle, it is possible to ignore small RC stick adjustments and timing inconsistencies using :ref:`RCn_DZ<RC1_DZ>` to specify the deadzone for each RC channel
- Attitude stabilisation limits the vehicle's maximum lean angle (roll/pitch away from level) to :ref:`ANGLE_MAX<ANGLE_MAX>`
- Automatic depth control (in modes like ALT_HOLD, SURFTRAK, POSHOLD, and CIRCLE) can be configured with the parameters

  - :ref:`THR_DZ<THR_DZ>` for the throttle deadzone, to avoid small joystick movements adjusting the current depth target
  - :ref:`PILOT_SPEED_UP<PILOT_SPEED_UP>` and :ref:`PILOT_ACCEL_Z<PILOT_ACCEL_Z>`, for limiting maximum vertical speeds, and setting the vertical acceleration to reach those speeds

    - :ref:`PILOT_SPEED_DN<PILOT_SPEED_DN>` can be optionally used to set the maximum descent rate independently, instead of defaulting to :ref:`PILOT_SPEED_UP<PILOT_SPEED_UP>`
  - :ref:`SURFACE_MAX_THR<SURFACE_MAX_THR>` scales down upwards thrust when near the surface, to avoid pushing through the water surface and sucking air
- Automatic horizontal velocity control in POSHOLD mode can be limited by :ref:`PILOT_SPEED<PILOT_SPEED>`
- It is also possible to configure the outputs directly, using

  - :ref:`MOT_SLEW_UP_TIME<MOT_SLEW_UP_TIME>` and :ref:`MOT_SLEW_DN_TIME<MOT_SLEW_DN_TIME>`, to limit the rate the motors can increase or decrease thrust levels when a change is commanded
  - :ref:`MOT_PWM_MIN<MOT_PWM_MIN>` and :ref:`MOT_PWM_MAX<MOT_PWM_MAX>` define the available PWM range of all motor outputs

    - This is typically used to ensure the motor output commands match the expected input range of the ESCs, but can also be used to intentionally set a reduced range, to reduce performance and/or power usage

    .. note:: Dshot ESCs ignore these values. Range is fixed at 1000µs to 2000µs.

    - If one or more PWM ESC controlled motors are moving while disarmed, it may be necessary to adjust the relevant :ref:`SERVOn_TRIM<SERVO1_TRIM>` parameter(s) to correct the neutral points
- Motor output scaling can also be adjusted automatically, :ref:`in response to feedback from the power/battery monitoring system <current-limiting-and-voltage-scaling>`, using

  - :ref:`MOT_BAT_CURR_MAX<MOT_BAT_CURR_MAX>` to reduce the power output after excessive current draw occurs for more than :ref:`MOT_BAT_CURR_TC<MOT_BAT_CURR_TC>`, and/or
  - :ref:`MOT_BAT_VOLT_MIN<MOT_BAT_VOLT_MIN>` and :ref:`MOT_BAT_VOLT_MAX<MOT_BAT_VOLT_MAX>` to scale up motor outputs to compensate for sagging voltage over time
