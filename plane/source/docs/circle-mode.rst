.. _circle-mode:

===========
CIRCLE Mode
===========

Circle mode is similar to :ref:`LOITER <loiter-mode>`, but doesn't attempt
to hold position. This is primarily meant as a failsafe mode and is the
mode that the aircraft will enter by default for 20 seconds when a
failsafe event occurs, before switching to RTL.

Circle mode is deliberately a very conservative mode, and doesn't rely
on GPS positioning as it is used when GPS fails. It will do a large
circle, The bank angle is set to the ``LIM_ROLL_CD`` divided by 3, to
try to ensure the plane remains stable even without GPS velocity data
for accelerometer correction. That is why the circle radius is so large.

Circle mode uses throttle and pitch control to maintain altitude at the
altitude where it started circling.
