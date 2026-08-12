.. _guided-mode:

===========
GUIDED Mode
===========

The GUIDED mode is used when you want the aircraft to fly to a specific
point on the map without setting up a mission. Most ground control
stations support a "click to fly to" feature where you can click a point
on the map and the aircraft will fly to that location then loiter.

The other major use for GUIDED mode is in :ref:`geo-fencing <geofencing>`.
When the geo-fence is breached the aircraft will enter GUIDED mode, and
head to the predefined geo-fence return point, where it will loiter
until the operator takes over.

Normal "fly to" navigation in GUIDED mode is flown by the same L1
navigation controller (:ref:`NAVL1_PERIOD<NAVL1_PERIOD>`, etc) used in
AUTO mode.

A separate ``GUIDED_*`` PID set is only used if a ground
station or companion computer sends the
`MAV_CMD_GUIDED_CHANGE_HEADING <https://mavlink.io/en/messages/common.html#MAV_CMD_GUIDED_CHANGE_HEADING>`__
MAVLink command to slew to a new heading.

.. _plane-commands-in-guided-mode_tuning_guided_change_heading:

Tuning GUIDED_CHANGE_HEADING
============================

The ``GUIDED_*`` PID parameters are a guidance-layer gain, not an attitude/rate-controller gain, and they apply *only* while ``MAV_CMD_GUIDED_CHANGE_HEADING`` has an active target: they convert the heading/course error into a demanded bank angle, which the normal roll controller then flies. They play no part in normal Guided "fly to this location" navigation, AUTO waypoint navigation (which uses :ref:`NAVL1_\* <plane:NAVL1_PERIOD>`/:ref:`TECS_\* <plane:TECS_TIME_CONST>` instead), or the other GUIDED_CHANGE_* commands.

Tune the aircraft's normal roll controller first. If roll already tracks its demand well but heading changes feel too weak or too aggressive, adjust :ref:`GUIDED_P<GUIDED_P>` — higher commands more bank for a given heading error, lower softens it. :ref:`GUIDED_I<GUIDED_I>` and :ref:`GUIDED_D<GUIDED_D>` default to zero and rarely need changing.