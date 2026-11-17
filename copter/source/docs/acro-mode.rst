.. _acro-mode:

=========
Acro Mode
=========

Acro mode uses the RC sticks to control the angular velocity
of the copter in each axis. Release the sticks and the vehicle will maintain its
current attitude and will not return to level (attitude hold). Acro mode is useful for
aerobatics such as flips or rolls, or FPV when smooth and fast control
is desired. Raising the :ref:`INS_GYRO_FILTER<INS_GYRO_FILTER>` cutoff frequency and retuning may allow better responsiveness, but appropriate harmonic notch filtering to prevent noise impacts must be setup prior to retuning. 
See :ref:`common-imu-notch-filtering`  for more information.

Pure rate stabilization, bypassing the attitude (angle) controller and using only the rate controller, can be selected by setting bit 1 of :ref:`ACRO_OPTIONS<ACRO_OPTIONS>` ("Rate Loop Only", i.e. :ref:`ACRO_OPTIONS<ACRO_OPTIONS>` = 2). In this mode the rate controller's I term resists disturbances, similar to a "heading hold" tail gyro, but the attitude itself is not held, so it can drift over time. The :ref:`Acro Trainer <acro-mode_acro_trainer>` leveling and lean angle limit still apply in this mode, so the attitude only drifts freely with :ref:`ACRO_TRAINER<ACRO_TRAINER>` set to 0.

Rate Loop Only with I-term Scaling
===================================

On multicopters, when the "Rate Loop Only" option is enabled (bit 1 of :ref:`ACRO_OPTIONS<ACRO_OPTIONS>`), the rate controller's I-term (integral term) is automatically scaled by the angle P gain for each axis. This makes the I-term contribution proportional to the attitude correction strength, mimicking the tuning approach used by Betaflight. Pilots transitioning from Betaflight should find that after ArduPilot tuning (:ref:`common-tuning`), the tune will feel more like Betaflight tuning with this option enabled.

With I-term scaling enabled:

-  The attitude controller bypasses angle-level stabilization and sends rate commands directly to the rate PID controller.
-  The I-term on each axis (roll, pitch, yaw) is multiplied by that axis's angle P gain value (e.g. :ref:`ATC_ANG_RLL_P<ATC_ANG_RLL_P>`), providing a coupling between attitude correction and integral wind-up that gives a familiar feel for Betaflight pilots.
-  Each axis is scaled independently, allowing asymmetric tuning if desired.


Overview
========

-  The throttle is completely manual with no compensation for tilt angle
   of the vehicle. If the pilot puts the throttle completely down the
   motors will go to their minimum rate and stabilization will cease. However, this behavior
   can be changed by enabling :ref:`airmode`.
-  Copter includes an Acro Trainer functionality that can be
   turned on/off to make learning to fly Acro easier.
-  Stick inputs are interpreted in the "body frame" (as opposed to Sport
   mode in which they are "earth frame"). The difference between "body
   frame" and "earth frame" is most obvious when the vehicle is leaned
   over. For example when pitched forward at 45 degrees, when left yaw
   is applied if using an earth frame controller (i.e. Sport mode) the
   copter will maintain its pitch and roll angles as its heading
   changes. With a body frame controller like Acro it will rotate about
   the vehicle's vertical axis meaning the pitch angle will become the
   roll angle and the roll angle will become the pitch angle.

.. warning::

   Acro is the most difficult flight mode to master and you can
   look forward to crashing multiple times before you get the hang of
   it. While Acro mode does not necessarily require GPS, switching to RTL in case of emergency does. Make sure you do have a reliable
   position estimate prior to arming, most commonly provided by 3D GPS fix with sufficient HDOP.

.. youtube:: tC0mF-N5z0Q
    :width: 100%

The above video was filmed in ACRO mode
using :ref:`FPV goggles <common-fpv-first-person-view>`.


AirMode
=======

Acro mode can be setup to provide full stabilization at idle throttle. See :ref:`airmode` 

.. _acro-mode_acro_trainer:

Acro Trainer
============

The :ref:`ACRO_TRAINER <ACRO_TRAINER>` parameter can be set to:

-  0 = disabled. This means the pilot operates in full Rate control with
   no automatic leveling nor angle-limiting performed by the autopilot.
-  1 = automatic leveling. The vehicle will automatically return to
   level when the pilot releases the sticks. The leveling is reduced as the
   sticks are moved away from center, and is removed at full stick. The
   aggressiveness with which it returns to level can be controlled with the
   :ref:`ACRO_BAL_ROLL<ACRO_BAL_ROLL>` and :ref:`ACRO_BAL_PITCH<ACRO_BAL_PITCH>` parameters. The default of 1.0 will
   cause it to return to level at up to 30 deg/s. Higher values will make it
   return more quickly.
-  2 (Default) = automatic leveling and lean angle limited. The vehicle
   returns to level as with option 1, but the leveling is applied at all stick
   positions, and in addition the vehicle will not lean more than
   :ref:`ATC_ANGLE_MAX<ATC_ANGLE_MAX>` (30 degrees by default).

The trainer can also be selected in flight using an RC switch, by setting
that channel's ``RCx_OPTION`` to 14 (Acro Trainer). With a 3 position switch
the low position (PWM < 1200) will disable the trainer, the middle position
will enable option 1 (automatic leveling) and the high position
(PWM > 1800) will enable option 2 (leveling and lean angle limited). With a
2 position switch only options 0 (disabled) and 2 (leveling and limited) are
possible.

Traditional Helicopters
=======================

For Traditional Helicopters, this mode operates the same way. However, experienced helicopter pilots might find that it has a slightly robotic feel, and that the attitude jumps as the helicopter lifts off the skids. The "Virtual Flybar" feature simulates the classic feel of a flybar helicopter.

Without it, if stick inputs are given while the helicopter cannot follow them, for example while sitting on the ground, the difference between the commanded attitude and the actual attitude accumulates, and the helicopter jumps to the commanded attitude when the skids clear. Multicopter pilots will be familiar with this behavior. The Virtual Flybar continuously bleeds off this difference. While landed it always does so; in flight, the rate at which it does so is set by :ref:`ACRO_BAL_ROLL<ACRO_BAL_ROLL>` and :ref:`ACRO_BAL_PITCH<ACRO_BAL_PITCH>`. Values of around 2 will usually give good results.

.. note:: The Virtual Flybar is only active when :ref:`ACRO_TRAINER<ACRO_TRAINER>` is set to 0 (disabled). The default trainer setting is 2, so it must be changed to use this feature.

Tuning Parameters
=================

-  :ref:`ACRO_RP_RATE<ACRO_RP_RATE>` is the roll and pitch rotation rate, in
   deg/s, commanded at full stick deflection (default 360 deg/s, range 1 to
   1080). It is also used as the roll and pitch rate in Sport mode.
-  :ref:`ACRO_Y_RATE<ACRO_Y_RATE>` is the yaw rotation rate, in deg/s, commanded
   at full stick deflection (default 202.5 deg/s, range 1 to 360). It is also
   used in Drift mode. In other modes the yaw rate is set by
   :ref:`PILOT_Y_RATE<PILOT_Y_RATE>`.
-  :ref:`ACRO_RP_EXPO <ACRO_RP_EXPO>` and :ref:`ACRO_Y_EXPO <ACRO_Y_EXPO>` set the
   amount of exponential applied to the pilot's stick inputs in Acro mode only.
   Positive values reduce the rotation rate commanded around center stick,
   giving finer control for small corrections, while full stick still commands
   the full rate. For example, with the default roll and pitch expo of 0.3, half
   stick commands about 41% of :ref:`ACRO_RP_RATE<ACRO_RP_RATE>`. Negative
   values do the opposite, making the response around center stick more
   sensitive. The roll and pitch expo can be set between -0.5 and 0.95
   (default 0.3) and the yaw expo between -1.0 and 0.95 (default 0, no expo).
   Typical values are 0 to 0.5. Values close to 0.95 make the response around
   center stick extremely soft, and a value of 0.95 switches the expo off
   completely (linear response).
-  :ref:`ACRO_RP_RATE_TC <ACRO_RP_RATE_TC>` and :ref:`ACRO_Y_RATE_TC <ACRO_Y_RATE_TC>` are
   time constants, in seconds, that smooth how quickly the commanded rotation
   rate follows the stick. Lower values give a sharper response and higher
   values a softer one (0.05 is very crisp, 0.5 is very soft). The default of 0
   selects a built-in time constant of 10 loop periods (0.025 seconds at the
   default 400 Hz loop rate), which is crisp, but values of 0.01 or 0.02 are
   sharper. The acceleration limits described below also apply.
   :ref:`ACRO_RP_RATE_TC <ACRO_RP_RATE_TC>` also applies in Sport mode and
   :ref:`ACRO_Y_RATE_TC <ACRO_Y_RATE_TC>` in Drift mode. See :ref:`input-shaping`.
-  On multicopters, :ref:`ACRO_THR_MID <ACRO_THR_MID>` is the throttle output at mid-stick in Acro
   mode. The default of 0 uses the hover throttle,
   :ref:`MOT_THST_HOVER <MOT_THST_HOVER>`. As with the hover throttle, the
   mid-stick output is limited to the range 0.125 to 0.6875.

.. note:: Roll and pitch stick inputs are limited to a circle, so with the
   roll and pitch sticks both at full deflection (diagonally), each axis
   receives about 70% of full stick. Also, with the default
   :ref:`ACRO_TRAINER <ACRO_TRAINER>` setting of 2, the vehicle is prevented
   from leaning beyond :ref:`ATC_ANGLE_MAX <ATC_ANGLE_MAX>`, so continuous rolls
   and flips are only possible with :ref:`ACRO_TRAINER <ACRO_TRAINER>` set to 0
   or 1.

Advanced Tuning Parameters
==========================

After you have become very confident with flying in ACRO mode, you may
want to go deeper in to tuning it based on your vehicle's performance
characteristics. These parameters should be adjusted by advanced users
only with the intent of removing "bounce" after very fast roll or flip
maneuvers on a vehicle that you are confident is very well tuned. Note
that these parameters are global parameters that apply to all flight
modes, not just ACRO.

-  :ref:`ATC_ACC_R_MAX<ATC_ACC_R_MAX>` and
   :ref:`ATC_ACC_P_MAX<ATC_ACC_P_MAX>`: maximum rotational acceleration in the
   roll and pitch axes, in deg/s/s. These limit how quickly the commanded
   rotation rate is allowed to change, so that the target does not run ahead
   of what the vehicle can physically achieve. For example, a highly nimble
   quadcopter with :ref:`ACRO_RP_RATE<ACRO_RP_RATE>` set to 720 cannot go from
   0 deg/s to 720 deg/s instantly. If the commanded rate changes faster than
   the vehicle can follow, the vehicle will overshoot the desired attitude
   when the sticks are released and then bounce back. Lowering these values
   softens the start and end of fast rolls and flips and reduces this
   overshoot. :ref:`AutoTune <autotune>` sets these values, and they should
   only be reduced after the rate controller has been properly tuned.

-  :ref:`ATC_ACC_Y_MAX<ATC_ACC_Y_MAX>`: maximum rotational acceleration in the
   yaw axis, in deg/s/s. Same principle as :ref:`ATC_ACC_R_MAX<ATC_ACC_R_MAX>`
   and :ref:`ATC_ACC_P_MAX <ATC_ACC_P_MAX>` but for the yaw axis, where the
   achievable acceleration is usually much lower.
-  :ref:`ATC_THR_MIX_MAN <ATC_THR_MIX_MAN>`: Balance between attitude and throttle control. This 
   value can be increased to improve attitude control when the throttle is cut, for better Airmode handling,
   however a stable tune is essential before it can be increased.

User Videos
===========
.. youtube:: dnygfyvXmL4
    :width: 100%
.. youtube:: rJUpmGjbU8k
    :width: 100%
.. youtube:: 42iS9d-Lrqw
    :width: 100%
    
