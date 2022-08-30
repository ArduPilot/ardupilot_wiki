.. _qacro-mode:

==========
QACRO Mode
==========

QACRO (for acrobatic) is a quadplane mode for advanced users that provides rate based stabilization like :ref:`Copter ACRO <copter:acro-mode>`.  QACRO mode is best suited for aerobatic flight of a copter tailsitter, but is also available for other quadplane types.

Training mode is not implemented and this mode will behave similarly to a fixed wing plane in MANUAL mode, with no limitations on earth frame roll, pitch and yaw.

Aileron and elevator stick scalings are set by parameters :ref:`Q_ACRO_RLL_RATE<Q_ACRO_RLL_RATE>` and :ref:`Q_ACRO_PIT_RATE<Q_ACRO_PIT_RATE>`, respectively, with default of 180 deg/sec. Yaw stick scaling is set by :ref:`Q_ACRO_YAW_RATE<Q_ACRO_YAW_RATE>` with default of 90 deg/sec.

To avoid control surface oscillation, it is necessary to reduce control surface deflections at high airspeeds in VTOL modes.  If the vehicle has no airspeed sensor, this reduction is based on attitude (tilt angle from vertical) and throttle setting, since these are generally correlated with airspeed. If an airspeed sensor is available, the reduction is based on the measured airspeed.
There are 2 parameters which control gain scaling: :ref:`Q_TAILSIT_GSCMSK<Q_TAILSIT_GSCMSK>` and :ref:`Q_TAILSIT_GSCMIN<Q_TAILSIT_GSCMIN>`

:ref:`Q_TAILSIT_GSCMSK<Q_TAILSIT_GSCMSK>` is a bitmask with two bits:
BOOST (bit 0): boost gain at low throttle and
ATT THR (bit 1): reduce gain at high throttle/tilt
If BOOST is set, parameter ``Q_TAILSIT_THSCMX`` determines whether gain boost (default 2) is applied below hover throttle (used for hovering a conventional 3D plane)
If ATT THR is set, attenuation is applied at high throttle and tilt angles (used for "copter" tailsitters). :ref:`Q_TAILSIT_GSC_MIN<Q_TAILSIT_GSCMIN>` (default 0.4) sets the minimum gain scaling at high throttle/tilt angle when ATT THR is active.

Parameter summary:

1. If gain boost at low throttle values is desired:
    a) Set bit 0 of :ref:`Q_TAILSIT_GSCMSK<Q_TAILSIT_GSCMSK>` (value 1).
    b) Set ``Q_TAILSIT_THSCMX`` to the maximum boost value desired.

2. If attitude/throttle based gain attenuation is desired to reduce oscillation at higher airspeeds in VTOL modes:
    a) Set bit 1 of :ref:`Q_TAILSIT_GSC_MSK<Q_TAILSIT_GSCMSK>` (value 2).
    b) Set :ref:`Q_TAILSIT_GSC_MIN<Q_TAILSIT_GSCMIN>` to minimum desired scale (greater than zero and less than 1) for gain  attenuation based on attitude and throttle. Reduce this value if oscillation occurs at high airspeeds.

3. If both gain scaling functions are desired:
    a) Set :ref:`Q_TAILSIT_GSC_MSK<Q_TAILSIT_GSCMSK>` to 3.
    b) Set ``Q_TAILSIT_THSCMX`` and :ref:`Q_TAILSIT_GSC_MIN<Q_TAILSIT_GSCMIN>` as desired.

QACRO flying tips:

Transitions from QACRO mode to any other Q-mode are not aided by the autopilot: no throttle boost is automatically applied. This is critical when airspeed (or high throttle) is needed for attitude control, as with non-vectored dual motor tailsitters. The safest way to transition is to make sure you have sufficient airspeed for elevon authority if flying level, or to establish a stable nose-up hover before switching out of QACRO.

Hover Throttle
==============

Usually, it is desired to hover in any mode, at mid-stick on throttle, so that transitions between modes is easily accomplished without throttle position changes. This can be adjusted using the :ref:`Q_M_THST_HOVER<Q_M_THST_HOVER>` parameter, or automatically learned in QHOVER or QLOITER modes by enabling :ref:`Q_M_HOVER_LEARN<Q_M_HOVER_LEARN>`.

.. note:: If :ref:`Q_THROTTLE_EXPO<Q_THROTTLE_EXPO>` = 0 in QACRO and QSTABILIZE modes , then :ref:`Q_M_THST_HOVER<Q_M_THST_HOVER>`, whether set manually or learned via :ref:`Q_M_HOVER_LEARN<Q_M_HOVER_LEARN>` , is not applied, and the throttle is determined directly from the RC input.

