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

Setting :ref:`Q_RTL_MODE<Q_RTL_MODE>` to 3 results in behavior similar to a normal :ref:`QRTL <qrtl-mode>`. The vehicle returns like normal fixed wing RTL until it reaches a point about 5X the :ref:`RTL_RADIUS<RTL_RADIUS>`
distance, then starting a descent towards :ref:`Q_RTL_ALT<Q_RTL_ALT>`. As it approaches the landing position, ArduPilot starts an "airbraking" phase to slow the vehicle and once slowed enters full VTOL mode and proceeds to execute a QRTL. This behavior is also used by default for the :ref:`QRTL<qrtl-mode>` mode unless :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16 is set to prevent the Hybrid operation above.

In effect, this enables the QRTL mode for any RTL actuation: failsafe actions, mode change to QRTL, completion of a mission (unless the last mission item prevents RTL).

.. note:: This mode is also used by default in all mission VTOL_LANDINGs unless the :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16 is set to disable it. This allows the item to be used without needing to setting up approach waypoints to reduce alitutde and get close enough to proceed in VTOL mode toward the landing point. If disabled by bit 16, the vehicle will instantly transition to VTOL mode upon that mission items execution, and navigate to its landing point before doing a QLAND. This means that you should be very close to the landing site if the FW approach mode is disabled in a mission since it will proceed in VTOL flight to the land point.

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
