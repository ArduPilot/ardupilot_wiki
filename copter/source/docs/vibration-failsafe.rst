.. _vibration-failsafe:

==================
Vibration Failsafe
==================

This feature is available only in Copter 4.0 and later firmware.

Vibration Failsafe is a little different than most other failsafes in ArduPilot, in that it does not initiate a flight mode change. Rather, it changes the algorithm which controls climb rate and acceleration in altitude control modes. It is enabled by default, but can be disabled by setting :ref:`FS_VIBE_ENABLE<FS_VIBE_ENABLE>` = 0.

Impact of High Vibrations
=========================

Multicopters can suffer from very high vibration levels (more 60m/s/s) which can lead to the accelerometers saturating (i.e. going beyond the range that the sensor can measure). This leads to “clipping” and means the EKF cannot accurately calculate it’s climb rate or vertical acceleration. This can lead to the vehicle becoming unable to control it’s climb rate and, in severe situations, can lead to the vehicle climbing rapidly at full throttle.

Activation
==========

When the EKF’s vertical velocity and position innovations becomes negative AND the velocity variance is less than 1, and this situation persists for 1 second or longer.

.. note:: An ``Innovation`` is the difference between the predicted value and the latest (non-IMU) value. A ``Variance`` is the EKF’s reported confidence in its estimate. 0 is very good, >1 is bad.

Algorithm Change
================

- **“Vibration compensation ON”** will appear on the Ground Stations HUD. EKF’s climb rate will be calculated using a 3rd order complementary filter which is tuned to be more resistant to vibration (but less accurate) than the normal method.
- Altitude controller switches to a more vibration resistant two-stage controller (position->velocity) instead of the regular three-stage controller (i.e. position->velocity->acceleration)
- The vehicle will not change mode but it’s altitude hold will be less accurate than normal. The vehicle may overshoot it’s altitude targets and/or respond more slowly to pilot input.

De-Activation
=============

- The vibration compensation will stay on for at least 15 seconds.
- When/if the EKF recovers, **“Vibration compensation OFF”** will be displayed on the HUD and the climb rate and altitude controllers will return to the regular method.
