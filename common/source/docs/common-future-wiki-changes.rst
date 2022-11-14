.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp"]

[site wiki="plane"]

on :ref:`acro-mode` page, in section "Acro Locking", add:
=========================================================

It is recommended that it be set to "2", instead of "1", in order to use a quarternion based control system with much better performance than the older system. In order for this to be effective, yaw rate control (:ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>`) must be "1" and the yaw rate controller tuned using :ref:`Autotune <automatic-tuning-with-autotune>` for best performance.

on the :ref:`automatic-tuning-with-autotune` page, add in the setup section:
============================================================================

The :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>` bitmask selects which axes will be tuned while in Autotune. Default is roll, pitch and yaw.

[/site]