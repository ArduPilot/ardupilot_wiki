.. _qacro-mode:

=========+
QACRO Mode
=========+

QACRO (for acrobatic) is a quadplane mode for advanced users that provides rate
based stabilization like :ref:`Copter ACRO <copter:acro-mode>`.  QACRO mode is best suited for aerobatic flight of a copter tailsitter, but is also available for other quadplane types.

Also, set Q_M_HOVER_LEARN to 2 (learn and save) to allow the throttle midpoint to adjust automatically when hovering.

Training mode is not implemented and this mode will behave similarly to a fixed wing plane in MANUAL mode, with no limitations on earth frame roll, pitch and yaw.

Aileron and elevator stick scalings are set by parameters ACRO_ROLL_RATE and ACRO_PITCH_RATE, respectively, with default of 180 deg/sec. Yaw stick scaling is set by Q_YAW_RATE_MAX with default of 100 deg/sec.

To avoid oscillation, it is necessary to reduce control surface deflections at high airspeeds in VTOL modes.  If the vehicle has no airspeed sensor, this reduction is based on attitude (tilt angle from vertical) and throttle setting, since these are generally correlated with airspeed. If an airspeed sensor is available, the reduction is based on the measured airspeed.
There are 3 parameters which control gain scaling: Q_TAILSIT_SPDMIN and Q_TAILSIT_SPDMAX set the lower and upper airspeeds over which an interpolation technique is used to blend the deflections commanded by the multicopter and fixed-wing controllers
If those are set equal, parameter Q_TAILSIT_THSCMX determines whether gain boost (value>=1) is applied below hover throttle (used for hovering a conventional 3D plane) or attenuation is applied at high throttle and tilt angles (used for "copter" tailsitters). When Q_TAILSIT_THSCMX is less than 1, it sets the maximum attenuation to be applied.

if (Q_TAILSIT_SPDMIN == Q_TAILSIT_SPDMAX) {
  if (Q_TAILSIT_THSCMX >= 1) {
    1) original method (gain boost at low throttle)
  } else {
    2) attitude/throttle method (gain attenuation based on attitude and throttle)
  }
} else {
  3) gain interpolation based on airspeed (needs airspeed sensor)
}
