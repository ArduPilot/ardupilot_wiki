.. _quadplane-transitions:

===========
Transitions
===========

QuadPlane transition is where the aircraft is changing between flying
primarily as a VTOL (copter-like) aircraft and flying as a
conventional fixed wing aircraft. Transition happens in both
directions, and can either be commanded by the pilot or happen
automatically based on airspeed/attitude/altitude(:ref:`assisted_fixed_wing_flight`) or flight mode changes during a mission.

The primary way to initiate a transition is to change flight mode,
either using the flight mode channel on your transmitter or using a
ground station to command a mode change.

During a transition, depending on type of QuadPlane and various :ref:`Q_OPTIONS<Q_OPTIONS>` settings, pilot control of the vehicle's attitude, climb rate, etc. may be modified or restricted to assure successful transition as detailed below.

.. note:: See :ref:`Tailsitter Section <guide-tailsitter>` for details on transitions for tailsitters, which are different than non-tailsitters discussed below:

Transition to Fixed Wing Mode from VTOL
=======================================

-  If you transition to :ref:`MANUAL <manual-mode>` or :ref:`ACRO <acro-mode>`, then the VTOL motors will immediately stop. In the case of a tilt-rotor, the motors will also immediately rotate to forward flight orientation.

.. warning:: If you do not have sufficient airspeed, an immediate stall will occur! Since MANUAL mode is often setup as a reflex driven "bail-out", some users move, or remove this mode, and substitute QSTABLIZE or QLOITER as an alternative "bail-out" for a QuadPlane. Also if the transition is made downwind, the transition time is short, and no airspeed sensor is used in non-tailsitters, a stall could occur also since insufficient airspeed has been obtained when VTOL assistance has terminated.
 
-  If you transition to any other fixed wing mode then the VTOL motors will continue to supply lift and stability until you have reached the :ref:`AIRSPEED_MIN <AIRSPEED_MIN>` airspeed (or airspeed estimate if no airspeed sensor). This phase is called "Transition airspeed wait".
-  In non-tilt rotor configurations, the forward motor(s) thrust is controlled by the throttle stick in a manner similar to whatever fixed wing mode was entered. Transitioning to FBWB/CRUISE, throttle stick controls forward thrust as in that mode, as a speed or throttle value, depending on whether or not an airspeed sensor is in use. In FBWA/STABILIZE transitions, it is directly controlled, ie low stick is zero thrust and the QuadPlane will just hover. The VTOL motors will behave similar to that in QHOVER and will try to maintain present altitude throughout the transition. During the transition, elevator input will act as climb/descent demand to the VTOL motors, roll input as roll attitude change, unless :ref:`Q_OPTIONS<Q_OPTIONS>` bit 0 is set.
-  In the case of tilt-rotors, the motors will tilt to :ref:`Q_TILT_MAX<Q_TILT_MAX>` for throttle stick positions at or above mid-stick to begin building forward airspeed for the transition.  In FBWA/STABILIZE transitions, throttle stick positions below mid-stick will proportionately rotate VTOL motors back towards vertical, since that controls the forward thrust component. Transitioning to FBWB/CRUISE in any configuration, throttle stick has no effect until transition is complete. Overall thrust to the motors will behave similar to that in QHOVER and will try to maintain present altitude throughout the transition. During the transition, elevator input will act as climb/descent demand to the VTOL motors, roll input as roll attitude change, unless :ref:`Q_OPTIONS<Q_OPTIONS>` bit 0 is set.

.. warning:: Unless the :ref:`Q_OPTIONS<Q_OPTIONS>` bit 0 is set, pulling back on elevator will not only pitch the nose up but also increase the VTOL motor output to assist in climbing during the transition airspeed wait phase. If bit 0 is set, only the pitch will change and altitude will not. In tilt-rotors, this can lead to delaying, or even preventing, the transition from ever completing! For tilt-rotors, do not pull back on pitch if this bit is set, until the transition is completed!

-  Once :ref:`AIRSPEED_MIN <AIRSPEED_MIN>` is reached the VTOL only motors' contribution will slowly drop in power over :ref:`Q_TRANSITION_MS <Q_TRANSITION_MS>` milliseconds (default is 5000, so 5 seconds) and will switch off after that. And tilt-rotors will slowly rotate to full forward thrust configuration. Once transition is completed, normal control of throttle and attitude resumes for whatever fixed wing mode the vehicle is now in.

.. note:: Usually by this time the VTOL motor contribution is already very low, since the QuadPlane is already flying, providing lift or climbing, and the VTOL contribution is only aiding attitude stabilization as required.

-  If :ref:`Q_TRANS_FAIL<Q_TRANS_FAIL>` is not zero, then exceeding this time before reaching  :ref:`AIRSPEED_MIN <AIRSPEED_MIN>` airspeed will cancel the transition and the aircraft will immediately execute the action specified by :ref:`Q_TRANS_FAIL_ACT<Q_TRANS_FAIL_ACT>`. The default is 0, which disables this timeout.

.. note:: if bit 19 of :ref:`Q_OPTIONS<Q_OPTIONS>` is set and :ref:`Q_TRANS_FAIL<Q_TRANS_FAIL>` is not zero, and if the airspeed is greater than 1/2 of :ref:`AIRSPEED_MIN<AIRSPEED_MIN>`, then the transition to fixed wing will immediately complete. This is useful if no airspeed sensor is being used, and the transition is into a headwind, which could prevent an accurate airspeed estimate from being obtained until a turn is made. Without using the :ref:`Q_TRANS_FAIL<Q_TRANS_FAIL>` timeout and this Q_OPTION, the transition could be indefinitely long since airspeed might be reported as below :ref:`AIRSPEED_MIN<AIRSPEED_MIN>` due to low groundspeed.

.. note:: The airspeed used during transition can be found in dataflash logs as CTUN.As. This is a canonical value which will include the airspeed sensor if enabled, or use the synthetic airspeed if not. 

Transition to a VTOL mode from Fixed Wing
=========================================

If you transition from a fixed wing mode to a QuadPlane VTOL mode then the
forward motor/thrust will immediately stop, and the control surfaces will
continue to provide stability while the plane slows down. This allows
for transitions to QuadPlane modes while flying at high speed. Tilt-rotors will, therefore, immediately move to VTOL position.

- VTOL attitude control will be provided as needed as the vehicle slows.
- Transition to altitude holding VTOL modes will manage power to the VTOL motors as necessary to hold altitude as the vehicle slows from forward fixed wing flight.
- Transition to non-altitude holding VTOL modes will provide vertical thrust as commanded by the throttle.
- Transition to position holding modes will project a stopping position to maintain based on deceleration of the vehicle and then hold it once reached.
- When transitioning to a position holding mode, like QLIOTER, QuadPlane will try to decelerate, which can result in the nose pitching up rapidly if traveling at high speeds (which will result in considerable altitude gain in most cases). In order to prevent this, the pitch is initially limited to 0 degrees, relaxing this limit to the smaller of :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>` or :ref:`PTCH_LIM_MAX_DEG<PTCH_LIM_MAX_DEG>` over the period of :ref:`Q_BACKTRANS_MS<Q_BACKTRANS_MS>`. Even with these limits altitude gain can result during the deceleration while transitioning.

.. note:: this phased-in pitch limit is applied for all fixed wing to VTOL transitions, except for transitions to QACRO.

The one exception to the forward motor stopping in QuadPlane VTOL
modes is if you have the :ref:`Q_VFWD_GAIN <Q_VFWD_GAIN>` parameter set to a non-zero
value. In that case the forward motor will be used to hold the
aircraft position in a wind. See the description of :ref:`Q_VFWD_GAIN <Q_VFWD_GAIN>`.

.. warning:: During transitions from VTOL to fixed wing mode, all motors can be running at very high levels. Battery sag below minimum levels (3.0V/cell for LiPo batteries) and resulting battery damage is possible. Extreme cases may even result in a crash due to VTOL motor output being too low. This is especially true when using high capacity, low C rating flight batteries common for long duration setups. This can be managed somewhat with manual throttle control when manually transitioning, but in AUTO mode, a VTOL to fixed wing transition is currently done with :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` on the forward motor until transition is complete, so very high currents can be experienced. Whether or not this will be an issue can be determined by examining the battery voltage during a manually initiated transition from the flight log. If too much voltage sag is seen, the best solutions are to use a higher C rating flight battery, or use separate batteries for forward motors and the VTOL motors, or to use :ref:`BATT_WATT_MAX<BATT_WATT_MAX>` and other parameters to limit excessive current draw during transitions. (See :ref:`Limiting Excessive Battery Power Draw <batt-watt-max>` )

Tailsitter Transitions
======================

Tailsitter transitions are slightly different. See :ref:`Tailsitter Section <guide-tailsitter>` for details.
