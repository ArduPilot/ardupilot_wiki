.. _qacro-mode:

==========
QACRO Mode
==========

QACRO (for acrobatic) is a quadplane mode for advanced users that provides rate based stabilization like :ref:`Copter ACRO <copter:acro-mode>`.  QACRO mode is best suited for aerobatic flight of a copter tailsitter, but is also available for other quadplane types.

Training mode is not implemented and this mode will behave similarly to a fixed wing plane in MANUAL mode, with no limitations on earth frame roll, pitch and yaw.

Aileron and elevator stick scalings are set by parameters ACRO_ROLL_RATE and ACRO_PITCH_RATE, respectively, with default of 180 deg/sec. Yaw stick scaling is set by Q_YAW_RATE_MAX with default of 100 deg/sec.

To avoid control surface oscillation, it is necessary to reduce control surface deflections at high airspeeds in VTOL modes.  If the vehicle has no airspeed sensor, this reduction is based on attitude (tilt angle from vertical) and throttle setting, since these are generally correlated with airspeed. If an airspeed sensor is available, the reduction is based on the measured airspeed.
There are 3 parameters which control gain scaling: Q_TAILSIT_SPDMIN and Q_TAILSIT_SPDMAX set the lower and upper airspeeds over which an interpolation technique is used to blend the deflections commanded by the multicopter and fixed-wing controllers.
If those are set equal, parameter Q_TAILSIT_THSCMX determines whether gain boost (value>=1) is applied below hover throttle (used for hovering a conventional 3D plane) or attenuation is applied at high throttle and tilt angles (used for "copter" tailsitters). When Q_TAILSIT_THSCMX is less than 1, it sets the maximum attenuation to be applied.

Parameter summary:

If vehicle has no airspeed sensor, or if attitude/throttle based gain attenuation is preferred: 

1. Set Q_TAILSIT_SPDMIN = Q_TAILSIT_SPDMAX and
2. Set Q_TAILSIT_THSCMX >= 1 for original gain scaling method (boost at low throttle) or to minimum desired scale (greater than zero and less than 1) for gain attenuation based on attitude and throttle

If vehicle has an airspeed sensor and gain interpolation is desired: 

3. Set Q_TAILSIT_SPDMIN and Q_TAILSIT_SPDMAX to the range of speeds over which multicopter and fixed-wing controller gains should be interpolated.

QACRO flying tips:

Transitions from QACRO mode to any other Q-mode are not aided by the autopilot: no throttle boost is automatically applied. This is critical when airspeed (or high throttle) is needed for attitude control, as with non-vectored dual motor tailsitters. The safest way to transition is to make sure you have sufficient airspeed for elevon authority if flying level, or to establish a stable nose-up hover before switching out of QACRO.

Hover Throttle
==============

Usually, it is desired to hover in any mode, at mid-stick on throttle, so that transitions between modes is easily accomplished without throttle position changes. This can be adjusted using the :ref:`Q_M_THST_HOVER<Q_M_THST_HOVER>` parameter, or automatically learned in QHOVER or QLOITER modes by enabling :ref:`Q_M_HOVER_LEARN<Q_M_HOVER_LEARN>`.

.. note:: If :ref:`Q_THROTTLE_EXPO<Q_THROTTLE_EXPO>` = 0 in QACRO and QSTABILIZE modes , then :ref:`Q_M_THST_HOVER<Q_M_THST_HOVER>`, whether set manually or learned via Q_M_HOVER_LEARN, is not applied, and the throttle is determined directly from the RC input.

