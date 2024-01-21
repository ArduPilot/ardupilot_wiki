.. _assisted_fixed_wing_flight:

==========================
Assisted Fixed-Wing Flight
==========================

The QuadPlane code can also be configured to provide assistance to the
fixed wing code in any flight mode except :ref:`MANUAL <manual-mode>` or :ref:`ACRO <acro-mode>`. VTOL motor assistance is enabled if :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>` is non-zero. 

When :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` is non-zero then the quad motors will assist with
both stability and lift whenever the airspeed drops below that
threshold. This can be used to allow flying at very low speeds in
:ref:`FBWA <fbwa-mode>` mode for example, or for assisted automatic fixed
wing takeoffs.

.. warning:: If you are not using an airspeed sensor, airspeed will be determined by the synthetic airspeed generated internally as a backup in case of airspeed sensor failure. This estimate can be very inaccurate at times. You may want to consider not enabling  Assisted Fixed Wing Flight if not using an airspeed sensor to prevent false activations when airspeed really  is above the threshold but is being misrepresented by the internal airspeed. Setting :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` to -1 will disable the pre-arm warning to set this parameter to a non-zero value to enable the feature, allowing it to remain disabled, if undesired.

It is suggested that you do initial flights with
:ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` set to zero
just to test the basic functionality and tune the airframe. Then try
with :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` above plane stall speed if you want that
functionality.

A second assistance type is available if :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>` is non-zero
based on attitude error. If :ref:`Q_ASSIST_ANGLE <Q_ASSIST_ANGLE>` is
non-zero then this parameter gives an attitude error in degrees above
which assistance will be enabled even if the airspeed is above
:ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>`. 

A third trigger to provide assistance. if :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>` is non-zero, is :ref:`Q_ASSIST_ALT<Q_ASSIST_ALT>`. This is the altitude below which QuadPlane assistance will be triggered. This acts the same way as :ref:`Q_ASSIST_ANGLE <Q_ASSIST_ANGLE>` and :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>`, but triggers if the aircraft drops below the given altitude while the VTOL motors are not running. A value of zero disables this feature. The altitude is calculated as being above ground level. The height above ground is given from a Lidar used if available and :ref:`RNGFND_LANDING<RNGFND_LANDING>` =1 or from terrain data if :ref:`TERRAIN_FOLLOW<TERRAIN_FOLLOW>` =1, or comes from the height above home otherwise.

Assistance will be activated :ref:`Q_ASSIST_DELAY<Q_ASSIST_DELAY>` after any of the above enabling thresholds are reached.

Assistance can also be enabled, disabled, or forced by setting an RC switch to ``RCx_OPTION`` = 82. If that channel is below  1200us (LOW), then assistance is unconditionally disabled, if above 1800us, (HIGH) then assistance is always enabled. For other RC values, assistance will be enabled as explained above.

Assistance can also be forced active all the time by setting :ref:`Q_OPTIONS<Q_OPTIONS>` bit 7 to "1". For Tailsitters, assistance for tailsitters can be limited only to VTOL motors by by setting :ref:`Q_OPTIONS<Q_OPTIONS>` bit 8 to "1". This can increase stability during assistance by not using the copter style pid gains on the flying surfaces as well as the VTOL motors, or for use with copter tailsitters without servo-controlled flying surfaces.

.. note:: Assistance is available for all QuadPlane frame types except the single motor and non-tilt dual motor tailsitter frames.

What assistance the quad motors provides depends on the fixed wing
flight mode. If you are flying in an autonomous or semi-autonomous
mode then the quad motors will try to assist with whatever climb rate
and turn rate the autonomous flight mode wants when assistance is
enabled (ie. airspeed is below :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>` or attitude error is
above :ref:`Q_ASSIST_ANGLE <Q_ASSIST_ANGLE>`, or altitude is below :ref:`Q_ASSIST_ALT<Q_ASSIST_ALT>` ). In a manually navigated mode the quad will try
to provide assistance that fits with the pilot inputs.

The specific handling is:

-  In :ref:`AUTO <auto-mode>` mode the quad will provide lift to get to
   the altitude of the next waypoint, and will help turn the aircraft at the
   rate the navigation controller is demanding.
-  In fixed wing :ref:`LOITER <loiter-mode>`, :ref:`RTL <rtl-mode>` or GUIDED
   modes the quad motors will try to assist with whatever climb rate and
   turn rate the navigation controller is asking for.
-  In :ref:`CRUISE <cruise-mode>` or :ref:`FBWB <fbwb-mode>` mode the quad
   will provide lift according to the pilot's demanded climb rate
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
