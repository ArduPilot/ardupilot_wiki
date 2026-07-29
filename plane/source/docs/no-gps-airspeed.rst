.. _no-gps-or-airspeed:

==============================
Flying without GPS or Airspeed
==============================

Generally, most features in ArduPlane rely on two assumptions:

* The plane can determine where it is in the world, generally with the use of a GPS
* The plane can determine its airspeed, either through a synthetic estimate or through a dedicated airspeed sensor.

The default parameters for ArduPlane assume that the above statements are true.
Much of the wiki is written assuming you have a GPS.
In the special case that you want to fly without either of these,
such as using just MANUAL and FBWA for initial flights, then follow this guide.

Recommended Parameters
======================

* Disable stall prevention with :ref:`STALL_PREVENTION<STALL_PREVENTION>` set to 0
* Disable GPS's with :ref:`GPS1_TYPE<GPS1_TYPE>` set to 0
* Disable airspeed use with :ref:`ARSPD_USE<ARSPD_USE>` set to 0
* Disable the first airspeed sensor with :ref:`ARSPD_TYPE<ARSPD_TYPE>` set to 0
* Force DCM with :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` set to 0
* Change the failsafe long action to ``Glide`` with :ref:`FS_LONG_ACTN<FS_LONG_ACTN>`

.. warning:: Using the :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` default of ``Continue``, or ``ReturnToLaunch``, will cause a fly-away.
             RTL requires GPS and should NOT be used.


Limitations
===========

:ref:`SERVO_AUTO_TRIM<SERVO_AUTO_TRIM>` won't work - this requires a velocity estimate. Although it's disabled by default, you will have to rely on manual trimming.

Most navigation modes won't be allowed, including AUTO, FBWB, and RTL.

:ref:`AUTOTUNE <automatic-tuning-with-autotune>` is not possible because it requires the vehicle to be flying faster than :ref:`AIRSPEED_MIN<AIRSPEED_MIN>`.
To tune your aircraft without GPS and airspeed sensor, you must do it manually.

