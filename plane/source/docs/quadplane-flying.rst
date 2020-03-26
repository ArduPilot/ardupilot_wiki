.. _quadplane-flying:

Flying a QuadPlane
==================

While flying a QuadPlane can actually be easier than flying a
conventional fixed wing aircraft there are some things you need to
understand. Please read the following sections carefully.

Transition
==========

QuadPlane transition is where the aircraft is changing between flying
primarily as a VTOL (copter-like) aircraft and flying as a
conventional fixed wing aircraft. Transition happens in both
directions, and can either be commanded by the pilot or happen
automatically based on airspeed and flight mode.

The primary way to initiate a transition is to change flight mode,
either using the flight mode channel on your transmitter or using a
ground station to command a mode change.

-  If you transition to :ref:`MANUAL <manual-mode>` then the VTOL motors
   will immediately stop. In the case of a tilt-rotor, the motors will immediately rotate to foward flight orientation.

.. warning:: If you do not have sufficient airspeed, an immediate stall will occur! Since MANUAL mode is often setup as a reflex driven "bail-out", some users move, or remove this mode, and substitute QSTABLIZE or QLOITER as an alternative "bail-out" for a QuadPlane
 
-  If you transition to any other fixed wing mode then the VTOL motors will
   continue to supply lift and stability until you have reached the
   :ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>` airspeed (or airspeed estimate if no airspeed sensor). In the case of tilt-rotors, the motors will tilt to :ref:`Q_TILT_MAX<Q_TILT_MAX>` to begin building forward airspeed for the transition. The VTOL motors will behave similar to that in QHOVER and will try to maintain present altitude through the transition. The forward thrust is controlled by the throttle stick in a manner similar to whatever fixed wing mode was entered. In FBWA, it is directly controlled, ie low stick is zero thrust and the QuadPlane will just hover. For tilt-rotors, stick positions below mid-stick will proportionately rotate VTOL motors back towards vertical, since that controls the forward thrust component. In FBWB/CRUISE, throttle stick controls forward thrust as in that mode, as a speed or throttle value, depending on whether or not an airspeed sensor is in use. 

.. note:: Unless the :ref:`Q_OPTIONS<Q_OPTIONS>` bit 0 is set, pulling back on elevator will not only pitch the nose up but also increase the VTOL motor output to assist in climbing during the transition airspeed wait phase.
 
-  Once that airspeed is reached the quad motors will slowly drop in
   power over :ref:`Q_TRANSITION_MS <Q_TRANSITION_MS>` milliseconds (default is 5000, so 5
   seconds) and will switch off after that. And tilt-rotors will slowly rotate to full forward thrust configuration.

.. note:: Usually by this time the VTOL motor contribution is already very low, since the QuadPlane is already flying, providing lift or climbing, and the VTOL contribution is aiding attitude stabilization.

-  If :ref:`Q_TRANS_FAIL<Q_TRANS_FAIL>` is not zero, then exceeding this time before reaching  :ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>` airspeed will cancel the transition and the aircraft will immediately change to QLAND. The default is 0, which disables this timeout.

If you transition from a fixed wing mode to a QuadPlane mode then the
forward motor/thrust will immediately stop, but the control surfaces will
continue to provide stability while the plane slows down. This allows
for transitions to QuadPlane modes while flying at high speed. Tilt-rotors will, therefore, immediately move to VTOL position.

The one exception to the forward motor stopping in QuadPlane VTOL
modes is if you have the :ref:`Q_VFWD_GAIN <Q_VFWD_GAIN>` parameter set to a non-zero
value. In that case the forward motor will be used to hold the
aircraft level in a wind. See the description of :ref:`Q_VFWD_GAIN <Q_VFWD_GAIN>`.

.. warning:: During transitions from VTOL to fixed wing mode, all motors can be running at very high levels. Battery sag below minimum levels (3.0V/cell for LiPo batteries) and resulting battery damage is possible. Extreme cases may even result in a crash due to VTOL motor output being too low. This is especially true when using high capacity, low C rating flight batteries common for long duration setups. This can be managed somewhat with manual throttle control when manually transitioning, but in AUTO mode, a VTOL to fixed wing transition is currently done with :ref:`THR_MAX<THR_MAX>`  on the forward motor until transition is complete, so very high currents can be experienced. Whether or not this will be an issue can be determined by examining the battery voltage during a manually initiated transition from the flight log. If too much voltage sag is seen, the best solutions are to use a higher C rating flight battery, or use separate batteries for forward motors and the VTOL motors, or to use :ref:`BATT_WATT_MAX<BATT_WATT_MAX>` and other parameters to limit excessive current draw during transitions. (See :ref:`Limiting Excessive Battery Power Draw <batt-watt-max>` )

Tailsitter Transitions
----------------------

Tailsitter transitions are slightly different. See :ref:`Tailsitter Section <guide-tailsitter>` for details.

VTOL vs Fixed-Wing Level Trim
=============================

Often fixed wing "level" trim, which is the pitch attitude stabilization modes attempt to maintain, is set to be several degrees positive with respect to the wing chord line in order to provide lift while cruising. This is accomplished either by running the accelerometer calibration level position set in this attitude, or after by using the "Calibrate Level" button in Mission Planner or by adjusting :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>` parameter.

However, when in VTOL modes, this can result in the vehicle leaning "backward" a few degrees, building in a tendency to drift backwards. This can be eliminated by setting the :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` parameter to correct this. This can also be used to correct minor CG imbalances caused by VTOL motor placement not being exactly balanced around the CG.

.. _assisted_fixed_wing_flight:

Assisted Fixed-Wing Flight
==========================

The QuadPlane code can also be configured to provide assistance to the
fixed wing code in any flight mode except :ref:`MANUAL <manual-mode>`. To
enable quad assistance you should set :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` parameter to the
airspeed below which you want assistance.

When :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` is non-zero then the quad motors will assist with
both stability and lift whenever the airspeed drops below that
threshold. This can be used to allow flying at very low speeds in
:ref:`FBWA <fbwa-mode>` mode for example, or for assisted automatic fixed
wing takeoffs.

.. warning:: If you are not using an airspeed sensor, airspeed will be determined by the synthetic airspeed generated internally as a backup in case of airspeed sensor failure. This estimate can be very inaccurate at times. You may want to consider not enabling  Assisted Fixed Wing Flight if not using an airspeed sensor to prevent false activations when airspeed really  is above the threshold, but is being misrepresented by the internal airspeed.

It is suggested that you do initial flights with
:ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` set to zero
just to test the basic functionality and tune the airframe. Then try
with :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` above plane stall speed if you want that
functionality.

From the 3.7.0 release an additional assistance type is available
based on attitude error. If :ref:`Q_ASSIST_ANGLE <Q_ASSIST_ANGLE>` is
non-zero then this parameter gives an attitude error in degrees above
which assistance will be enabled even if the airspeed is above
:ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>`. The attitude assistance will only be used if
:ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>` greater than zero.

And as of Plane-4.0 and later, a third trigger to provide assistance is :ref:`Q_ASSIST_ALT<Q_ASSIST_ALT>` . This is the altitude below which QuadPlane assistance will be triggered. This acts the same way as :ref:`Q_ASSIST_ANGLE <Q_ASSIST_ANGLE>` and :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>`, but triggers if the aircraft drops below the given altitude while the VTOL motors are not running. A value of zero disables this feature. The altutude is calculated as being above ground level. The height above ground is given from a Lidar used if available and :ref:`RNGFND_LANDING<RNGFND_LANDING>` =1 or from terrain data if :ref:`TERRAIN_FOLLOW<TERRAIN_FOLLOW>` =1, or comes from height above home otherwise.

What assistance the quad motors provides depends on the fixed wing
flight mode. If you are flying in an autonomous or semi-autonomous
mode then the quad motors will try to assist with whatever climb rate
and turn rate the autonomous flight mode wants when assistance is
enabled (ie. airspeed is below :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>` or attitude error is
above :ref:`Q_ASSIST_ANGLE <Q_ASSIST_ANGLE>` , or altitude is below :ref:`Q_ASSIST_ALT<Q_ASSIST_ALT>` ). In a manually navigated mode the quad will try
to provide assistance that fits with the pilot inputs.

The specific handling is:

-  In :ref:`AUTO <auto-mode>` mode the quad will provide lift to get to the
   altitude of the next waypoint, and will help turn the aircraft at the
   rate the navigation controller is demanding.
-  In fixed wing :ref:`LOITER <loiter-mode>`, :ref:`RTL <rtl-mode>` or GUIDED
   modes the quad motors will try to assist with whatever climb rate and
   turn rate the navigation controller is asking for.
-  In :ref:`CRUISE <cruise-mode>` or :ref:`FBWB <fbwb-mode>` mode the quad
   will provide lift according to the pilots demanded climb rate
   (controlled with pitch stick). The quad motors will try to turn at
   the pilot demanded turn rate (combining aileron and rudder input).
-  In :ref:`FBWA <fbwa-mode>` mode the quad will assume that pitch stick
   input is proportional to the climb rate the user wants. So if the
   user pulls back on the pitch stick the quad motors will try to climb,
   and if the user pushes forward on the pitch stick the quad motors
   will try to provide a stable descent.
-  In :ref:`AUTOTUNE <autotune-mode>` mode the quad will provide the same
   assistance as in :ref:`FBWA <fbwa-mode>`, but it is not a good idea to
   use :ref:`AUTOTUNE <autotune-mode>` mode with a high value of
   :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` as the quad assistance will interfere with the
   learning of the fixed wing gains.
-  In :ref:`MANUAL <manual-mode>`, :ref:`ACRO <acro-mode>` and
   :ref:`TRAINING <training-mode>` modes the quad motors will completely
   turn off. In those modes the aircraft will fly purely as a fixed
   wing.
-  In :ref:`STABILIZE <stabilize-mode>` mode the quad motors will try to
   provide lift if assistance is turned on.



Return to Launch (RTL)
======================

When flying a QuadPlane you have a choice of several methods of
handling return to launch. The choices are:

- circle about the return point as a fixed wing
- fly as a VTOL aircraft to the return point then land vertically
- fly as a fixed wing aircraft until close to the return point then switch to
  VTOL and land vertically

In each case a key concept is the return point. This is defined as the
closest rally point, or if a rally point is not defined then the home
location. See the :ref:`Rally Points <common-rally-points>` page for
more information on rally points.

Fixed Wing RTL
--------------

The default behaviour of the RTL mode is the same as for fixed
wing. It will fly to the nearest rally point (or home if no rally
point is defined) and circle as a fixed wing aircraft about that
point. The VTOL motors will not be used unless the aircraft drops below
the airspeed defined in :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>`. The altitude the aircraft
will circle at will be the altitude in the rally point, or the
:ref:`ALT_HOLD_RTL<ALT_HOLD_RTL>` altitude if a rally point is not being used.

VTOL RTL (QRTL)
---------------

If you prefer to do return to launch as a VTOL aircraft (like a
multirotor would do) then you can use the QRTL flight mode. That
flight mode will transition to VTOL flight and then fly at the
:ref:`Q_WP_SPEED <Q_WP_SPEED>` speed towards the return point, at an altitude of
:ref:`Q_RTL_ALT <Q_RTL_ALT>`.

Once the return point is reached the aircraft will start a vertical
descent towards the ground for landing. The initial descent rate is
set by :ref:`Q_WP_SPEED_DN <Q_WP_SPEED_DN>`. Once the aircraft reaches an altitude of
:ref:`Q_LAND_FINAL_ALT <Q_LAND_FINAL_ALT>` the descent rate will
change to :ref:`Q_LAND_SPEED <Q_LAND_SPEED>` for
the final landing phase.

In the final landing phase the aircraft will detect landing by looking
for when the VTOL motor throttle drops below a minimum threshold for 5
seconds. When that happens the aircraft will disarm and the VTOL
motors will stop.

Hybrid RTL
----------

The final option for RTL in a QuadPlane is to fly as a fixed wing
aircraft until it is close to the return point at which time it
switches to a VTOL RTL as described above. To enable this type of
hybrid RTL mode you need to set the :ref:`Q_RTL_MODE <Q_RTL_MODE>` parameter to 1.

The initial altitude that will be aimed for in the fixed wing portion
of the hybrid RTL is the same as for a fixed wing RTL. You should set
your rally point altitude and ALT\_HOLD_RTL options appropriately to
ensure that the aircraft arrives at a reasonable altitude for a
vertical landing. A landing approach altitude of about 15 meters is
good for many QuadPlanes. This should be greater than or equal to the
:ref:`Q_RTL_ALT <Q_RTL_ALT>` values.

The distance from the return point at which the aircraft switches from
fixed wing to VTOL flight is set using the :ref:`RTL_RADIUS<RTL_RADIUS>` parameter, or
if that is not set then the :ref:`WP_LOITER_RAD<WP_LOITER_RAD>` parameter is used. The
aircraft will then slow down as it approaches the return point, aiming
for an altitude set by :ref:`Q_RTL_ALT <Q_RTL_ALT>`.

Once the return point is reached the aircraft begins to descend and
land, exactly as described in the VTOL RTL mode above.

What Will Happen?
=================

Understanding hybrid aircraft can be difficult at first, so below are
some scenarios and how the ArduPilot code will handle them.

I am hovering in QHOVER/QLOITER and switch to FBWA mode
-------------------------------------------------------

The aircraft will continue to hover, setting forward thrust/throttle at whatever the throttle stick position dictates and gaining speed. If you zero throttle during the transition, the aircraft will continue to hold the current height and hold itself level, slowing to a halt. It will drift
with the wind as it is not doing position hold.

If you advance the throttle stick then the forward motor will throttle-up and
the aircraft will start to move forward. The quad motors will continue
to provide both lift and stability while the aircraft is moving slowly.
You can control the attitude of the aircraft with roll and pitch stick
input. When you use the pitch stick (elevator) that will affect the
climb rate of the quad motors. If you pull back on the elevator the quad
motors will assist with the aircraft climb. If you push forward on the
pitch stick the power to the quad motors will decrease and the aircraft
will descend.

The roll and pitch input also controls the attitude of the aircraft, so
a right roll at low speed will cause the aircraft to move to the right.
It will also cause the aircraft to yaw to the right (as the QuadPlane
code interprets right aileron in fixed wing mode as a commanded turn).

Once the aircraft reaches an airspeed of :ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>`
(or :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` if that is set and is greater than :ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>`)
the amount of assistance the quad motors provide will decrease over 5
seconds. After that time the aircraft will be flying purely as a fixed wing.

I am flying fast in FBWA mode and switch to QHOVER mode
-------------------------------------------------------

The quad motors will immediately engage and will start by holding the
aircraft at the current height. The climb/descent rate is now set by the
throttle stick, with a higher throttle stick meaning climb and a lower
throttle stick meaning descend. At mid-stick the aircraft will hold
altitude.

The forward motor will stop, but the aircraft will continue to move
forward due to its momentum. The drag of the air will slowly bring it to
a stop. The attitude of the aircraft can be controlled with roll and
pitch sticks (aileron and elevator). You can yaw the aircraft with
rudder.

I switch to RTL mode while hovering
-----------------------------------

The aircraft will transition to fixed wing flight. The quad motors will
provide assistance with lift and attitude while the forward motor starts
to pull the aircraft forward.

The normal Plane RTL flight plan will then be run, which defaults to
circling at the RTL altitude above the arming position or nearest rally
point. If you have :ref:`RTL_AUTOLAND <RTL_AUTOLAND>`
setup then the aircraft will do a fixed wing landing.

If you set :ref:`Q_RTL_MODE <Q_RTL_MODE>` to 1 then the aircraft will switch to a VTOL
landing when it gets close to return point.

Radio or Throttle Failsafe
==========================

If flying in a plane mode or AUTO, behaviour is determined by the :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` and :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` parameter settings (see Plane Failsafe Function). Quadplanes can be set such that instead of normal plane behviour on Failsafe induced RTLs, to transistion to QRTL and land once at the rally point or home, if  :ref:`Q_RTL_MODE<Q_RTL_MODE>` =1.
If not flying a mission, and are flying in any copter mode (QHOVER,QSTAB,etc.), failsafe will evoke QLAND or QRTL, depending on how :ref:`Q_OPTIONS<Q_OPTIONS>`, bit 5, is set.

Typical Flight
==============

A typical test flight would be:

-  VTOL takeoff in :ref:`QLOITER<qloiter-mode>` or :ref:`QHOVER<qhover-mode>`
-  switch to :ref:`FBWA <fbwa-mode>` mode and advance throttle over 50% and start
   flying fixed wing
-  switch to :ref:`QHOVER<qhover-mode>` mode to go back to quad mode and reduce throttle back to 50% for hover.

