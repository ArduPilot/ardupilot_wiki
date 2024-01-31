.. _airspeed-parameters-setup:

=========================
Airspeed Parameters Setup
=========================

.. note: ArduPlane does not require an airspeed sensor. However, some airspeed related parameters are used even if no airspeed sensor is present or being used, notably for the scaling of tuning parameters with speed. If you are not using an airspeed sensor set ARSPD_TYPE to 0. 

Speed Scaling
=============

Since the effect of flying surface control deflection increases and decreases with airspeed, the stabilization gains in the PID loops are automatically scaled with airspeed by ArduPilot to prevent oscillations at higher airspeeds and preserve stabilization effectiveness at low airspeeds, even if no airspeed sensor is used.

This is accomplished by ArduPilot estimating airspeed from other sensor inputs like GPS velocity, position changes, and IMU accelerations. This estimate is also used in case an airspeed sensor is used and becomes unhealthy. In systems without an airspeed sensor this estimate is used for speed scaling of the PID loops.

So, even if an airspeed sensor is not present, some airspeed parameters are used. 

.. note:: Normally, the default values are acceptable, but very fast or very slow vehicles may need them adjusted for best performance.

Parameters
==========

Key parameters are:

- :ref:`AIRSPEED_MAX<AIRSPEED_MAX>`: This is the fastest normal flying speed for the vehicle. Normally, 2 times the nominal cruising speed is a good target.
- :ref:`AIRSPEED_MIN<AIRSPEED_MIN>`: This is the slowest flying speed for the vehicle. It should be set at least 25% above the stall speed of the vehicle.
- :ref:`SCALING_SPEED<SCALING_SPEED>`: This is the center of the speed scaling range and should be set close to the normal cruising speed of the vehicle. PID tuning (see :ref:`tuning-quickstart`) should be done at this flying speed. The loop gains are scaled above and below this value with a multiplier determined base on the above min and max flying speeds.

The above parameters determine the speed scaling characteristics.

.. warning:: You must set the :ref:`SCALING_SPEED<SCALING_SPEED>` before doing any tuning, since changing this after tuning directly scales the effect of the tuned PID values and will corrupt the tune, possibly leading to instability. If you need to change this after tuning to match your vehicle's nominal cruise speed for some reason, revert the PID values back to their default values before flying and retune again at the normal cruising speed.


And, if you use an airspeed sensor:

- :ref:`ARSPD_TYPE<ARSPD_TYPE>`: This determines the type of sensor used. Set to "0" if no sensor is present. (see :ref:`airspeed`). If non-zero and the sensor is healthy, the system will use the sensor's measurements instead of the estimated speed calculation for speed scaling and speed controlled modes.
- :ref:`AIRSPEED_CRUISE<AIRSPEED_CRUISE>`: This determines the nominal cruising speed in AUTO/GUIDED speed controlled modes when using an airspeed sensor. (see :ref:`tuning-cruise` ). Also see :ref:`tuning-cruise` section if not using an airspeed sensor for the impact of the parameter - :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` on cruising speed in speed controlled modes like CRUISE.

ARSPD_USE
---------

:ref:`ARSPD_USE<ARSPD_USE>` determines if the airspeed estimate is used in other ArduPlane functions besides speed scaling. If no airspeed sensor is present and :ref:`ARSPD_USE<ARSPD_USE>` =1, then the estimated airspeed will be used as if was obtained via an accurate airspeed sensor.

.. warning:: using the estimated airspeed for speed controlled modes by enabling :ref:`ARSPD_USE<ARSPD_USE>` without an airspeed sensor can result in erratic, or even dangerous, operation since the estimate can be wrong in situations like automatic takeoffs into a strong headwind, or when flying into the wind for an extended period without making any turns (which updates the wind speed estimate and therefore the estimated airspeed).
