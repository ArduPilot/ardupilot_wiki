.. _glide-mode:

==========
GLIDE Mode
==========

GLIDE mode (mode number 27) is a failsafe mode that holds the wings level with the throttle at zero, letting the plane glide down. There is no pilot input. Throttle is held at zero regardless of :ref:`THR_MIN<THR_MIN>`.

The pitch target is nose down by the full :ref:`STAB_PITCH_DOWN<STAB_PITCH_DOWN>` (2 degrees by default) to keep up airspeed, limited by :ref:`PTCH_LIM_MIN_DEG<PTCH_LIM_MIN_DEG>`. It is not applied while a QuadPlane is in VTOL flight (for example, with Q_ASSIST active). The flare switch (:ref:`RCx_OPTION<RC1_OPTION>` = 89) has no effect in GLIDE mode.

.. note:: GLIDE mode is available in firmware 4.8 and later. In earlier versions, the "Glide" failsafe actions switched to FBWA mode with zero throttle and level attitude instead, so the reported mode did not reflect that the plane was in a failsafe glide.

GLIDE mode is entered when the following occur, subject to the per-mode failsafe rules in :ref:`apms-failsafe-function` (for example, no short failsafe action is taken in CIRCLE, TAKEOFF or RTL, and a long failsafe in RTL does not switch to GLIDE):

- an RC short failsafe occurs with :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` = 2 (Glide)
- a long failsafe (RC or GCS) occurs with :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` = 2 (Glide)
- an RC or GCS failsafe occurs while an :ref:`RC Auxiliary switch <common-auxiliary-functions>` set to "157" (Force-enable Glide failsafe action) is active. This overrides the normal failsafe action, allowing an emergency landing beyond RC range without the plane turning back toward home. The switch only takes effect in MANUAL, STABILIZE, ACRO, FBWA, FBWB, CRUISE, AUTOTUNE and TRAINING, and for a long failsafe also in CIRCLE, LOITER, THERMAL and TAKEOFF. In AUTO and GUIDED it is ignored and the normal failsafe action is taken.

It may also be selected as a :ref:`flight mode switch <flight-modes>` position or from a GCS, for example to test the failsafe behaviour.

When a short failsafe clears, the plane returns to the mode it was in before the failsafe. After a long failsafe, GLIDE mode continues until the pilot changes mode once the RC link is restored. See :ref:`apms-failsafe-function` for details.
