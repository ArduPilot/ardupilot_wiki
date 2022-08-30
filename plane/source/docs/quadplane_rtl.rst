.. _quadplane_rtl:

======================
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

Return to launch behavior is determined by the :ref:`Q_RTL_MODE<Q_RTL_MODE>` parameter.

Fixed Wing RTL
==============

The default behaviour (:ref:`Q_RTL_MODE<Q_RTL_MODE>` = 0)of the RTL mode is the same as for fixed
wing. It will fly to the nearest rally point (or home if no rally
point is defined) and circle as a fixed wing aircraft about that
point. The VTOL motors will not be used unless the aircraft drops below
the airspeed defined in :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>`. The altitude the aircraft
will circle at will be the altitude in the rally point, or the
:ref:`ALT_HOLD_RTL<ALT_HOLD_RTL>` altitude if a rally point is not being used.

.. _hybrid_rtl:

Hybrid RTL
==========

The another option for RTL in a QuadPlane is to fly as a fixed wing
aircraft until it is close to the return point at which time it
switches to a VTOL RTL as described above. To enable this type of
hybrid RTL mode you need to set the :ref:`Q_RTL_MODE <Q_RTL_MODE>` parameter to 1, 2, or 3.

:ref:`Q_RTL_MODE <Q_RTL_MODE>` = 1
----------------------------------

The initial altitude that will be aimed for in the fixed wing portion
of the hybrid RTL is the same as for a fixed wing RTL. You should set
your rally point altitude and :ref:`ALT_HOLD_RTL<ALT_HOLD_RTL>` parameters appropriately to
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

:ref:`Q_RTL_MODE <Q_RTL_MODE>` = 2
----------------------------------

Setting :ref:`Q_RTL_MODE<Q_RTL_MODE>` to 2 results in behavior similar to above, but with the vehicle returning like normal fixed wing RTL until it reaches :ref:`Q_FW_LND_APR_RAD<Q_FW_LND_APR_RAD>`, then loitering in fixed wing mode down to :ref:`Q_RTL_ALT<Q_RTL_ALT>` altitude, and then exiting facing the wind and executing a QRTL to the home position. Be sure the loiter portion is set up to clear any obstacles.

:ref:`Q_RTL_MODE <Q_RTL_MODE>` = 3
----------------------------------

Setting :ref:`Q_RTL_MODE<Q_RTL_MODE>` to 3 results in behavior similar to a normal :ref:`QRTL <qrtl-mode>`. The vehicle will enter an "APPROACH" phase, and will return at :ref:`ALT_HOLD_RTL<ALT_HOLD_RTL>` and at a calculated distance, start a descent towards :ref:`Q_RTL_ALT<Q_RTL_ALT>`. As it approaches the landing position, ArduPilot starts an "airbraking" phase in non-tailistters to slow the vehicle and once slowed enters full VTOL mode and proceeds to execute a VTOL landing. This behavior is also used by default for the :ref:`QRTL<qrtl-mode>` mode unless :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16 is set to prevent the Hybrid operation above.

In effect, this enables the QRTL mode for any RTL actuation: failsafe actions, mode change to QRTL, or completion of a mission (unless the last mission item prevents RTL).

.. note:: This mode is also used by default in all mission VTOL_LANDINGs unless the :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16 is set to disable it.

This fixed wing "approach" allows VTOL landings to be used without needing to setting up approach waypoints to reduce altitude and get close enough to proceed in VTOL mode toward the landing point. If disabled by bit 16, the vehicle will instantly transition to VTOL mode upon that mission items execution, or upons mode changes to :ref:`QRTL <qrtl-mode>`, and navigate to its landing point in VTOL before doing a QLAND. This means that you should be very close to the landing site if the FW approach mode is disabled in a mission since it will proceed in VTOL flight to the land point.

The image below details the phases of the approach and landing with the default setting of bit 16 (ie, not enabled):

.. image:: ../../../images/approach.jpg
    :target: ../_images/appraoch.jpg

The phases of the approach are:

- further than 2 times the greater of either :ref:`RTL_RADIUS<RTL_RADIUS>` or :ref:`WP_LOITER_RADS<WP_LOITER_RAD>` (MAXRAD) plus a calculated distance needed to descend from :ref:`ALT_HOLD_RTL<ALT_HOLD_RTL>` to :ref:`Q_RTL_ALT<Q_RTL_ALT>`, the plane will attempt to climb or descend to :ref:`ALT_HOLD_RTL<ALT_HOLD_RTL>`. If within that range, it will attempt to climb/descend to a linear descent slope, meet it, and continue to descend, as shown above.
- if started further than 2X "MAXRAD" but closer than above, at 2x MAXRAD it will continue in fixed wing mode at :ref:`Q_RTL_ALT<Q_RTL_ALT>`.
- when it reaches a point that is within the VTOL stopping distance of the landing point (at the VTOL deceleration parameter limits and current speed), it will transition to VTOL mode and send a message that it is in "VTOL Position1" and continue moving to the land point. If the vehicle is NOT a tailsitter, an "AIRBRAKING" phase may occur before the VTOL transition, spinning up the VTOL motors to create additional braking.
- once the QuadPlane is within 5 m of the land point and moving less than 2 m/s, it will semnd a GCS message declaring that it is in "VTOL Position2), and final position over the land point and begin its landing descent, which will also be indicated by GCS messages
- if the approach is entered less than 1.5X MAXRAD, it will immediately move to VTOL Position1 state, and move toward the landing site attempting to obtain :ref:`Q_RTL_ALT<Q_RTL_ALT>` as it does so.
- if in VTOL mode at greater than 1.5X MAXRAD, the plane will transition to fixed wing, and attempt to navigate to home, executing the approach. The climb and turn toward the landing point will occur at even low altitudes, just like normal non-QuadPlane RTLs, so the :ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>` bit 4 for "Climb before turn in RTL" and/or :ref:`Q_OPTIONS<Q_OPTIONS>` bit 0 for "Level Transitions" might be worth considering.

VTOL RTL (QRTL)
===============

If you prefer to do return to launch as a pure VTOL aircraft (like a
multirotor would do) then you can use the :ref:`QRTL<qrtl-mode>` flight mode, but with :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16 set in order to disable its default behavior, which is like the hybrid RTL described above with :ref:`Q_RTL_MODE<Q_RTL_MODE>` = 3. The vehicle will transition to VTOL flight and then fly at the
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
