.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


Plane
=====

:ref:`soaring-4_1` page:
-------------------------

Add content from :ref:`soaring-speed-to-fly`

:ref:`guide-tilt-rotor` page:
-----------------------------

Under Setting Up a Tilt Rotor replace first sentence with:

The first thing you need to do is enable QuadPlane support by setting
:ref:`Q_ENABLE<Q_ENABLE>` to 1 and Tilt Rotor support by setting :ref:`Q_TILT_ENABLE<Q_TILT_ENABLE>` = "1", and then choose the right quadplane frame class and
frame type.

:ref:`apms-failsafe-function` page:
-----------------------------------

add to Battery Failsafes section:

Battery Failsafe Actions
------------------------

The following is a description of the actions that can be taken for battery failsafes:

+-----+------------------+-----------------------------------------------------------------------------+
+Value| Action           |     Description                                                             +
+=====+==================+=============================================================================+
+ 0   | None             | Do nothing except warn                                                      +
+-----+------------------+-----------------------------------------------------------------------------+
+ 1   | RTL              | Switch to :ref:`RTL<rtl-mode>` mode                                         +
+-----+------------------+-----------------------------------------------------------------------------+
+ 2   | Land             | Switch to AUTO mode and execute nearest DO_LAND sequence, if in mission     +
+-----+------------------+-----------------------------------------------------------------------------+
+ 3   | Terminate        |  Disarm                                                                     +
+-----+------------------+-----------------------------------------------------------------------------+
+ 4   | QLAND            | If QuadPlane, switch to :ref:`qland-mode`, otherwise do nothing             +
+-----+------------------+-----------------------------------------------------------------------------+
+ 5   | Parachute        |  Trigger Parachute (Critical action only)                                   +
+-----+------------------+-----------------------------------------------------------------------------+
+ 6   | LOITER_TO_QLAND  | If QuadPlane, switch to LOITER_TO_QLAND mode,                               +
+     |                  | otherwise do nothing                                                        +
+-----+------------------+-----------------------------------------------------------------------------+



[copywiki destination="plane,copter,rover,dev"]