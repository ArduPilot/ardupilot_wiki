.. _vibration-failsafe:

==================
Vibration Failsafe
==================

This feature is available only in Copter 4.0 and later firmware.

Vibration Failsafe is a little different than most other failsafes in ArduPilot, in that it does not initiate a flight mode change. Instead it changes the algorithm which controls altitude and climb rate in altitude control modes. It is enabled by default, but can be disabled by setting :ref:`FS_VIBE_ENABLE<FS_VIBE_ENABLE>` = 0.

Impact of High Vibrations
=========================

Multicopters can suffer from very high vibration levels (more than 60m/s/s) which can lead to the accelerometers saturating (i.e. going beyond the range that the sensor can measure). This leads to “clipping” and means the EKF cannot accurately calculate its climb rate or vertical acceleration. This can lead to the vehicle becoming unable to control its climb rate and, in severe situations, can lead to the vehicle climbing rapidly at full throttle.

When the failsafe will trigger
==============================

The vibration failsafe will trigger if all of the following are true for at least one second:

- EKF’s vertical velocity innovations are positive (see onboard log's NKF4.IVD value)
- EKF's vertical position innovations are positive (see NKF4.IPD)
- EKF's velocity variance is 1 or higher (see NKF4.SV)

.. note:: An ``Innovation`` is the difference between the predicted value and the latest (non-IMU) value. A ``Variance`` is the EKF’s reported confidence in its estimate. 0 is very good, >1 is bad.

What will happen
================

If the vibration failsafe triggers the following will happen:

- “Vibration compensation ON” will appear on the Ground Stations HUD. EKF’s climb rate will be calculated using a 3rd order complementary filter which is tuned to be more resistant to vibration (but less accurate) than the normal method.
- Altitude controller switches to a more vibration resistant two-stage controller (position->velocity) instead of the regular three-stage controller (i.e. position->velocity->acceleration)
- The vehicle will not change mode but its altitude hold will be less accurate than normal. The vehicle may overshoot it’s altitude targets and/or respond more slowly to pilot input.

Recovery from the failsafe
==========================

- Vibration failsafe will deactivate 15 seconds after the EKF returns to normal
- “Vibration compensation OFF” will be displayed on the HUD
- Altitude and climb rate controllers will return to their normal methods
