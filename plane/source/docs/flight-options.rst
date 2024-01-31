.. _flight-options:

==============
Flight Options
==============


:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>` is a bitmask that allows configuring several alterations to Plane's behavior.


=====================================   ======================
:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>`   Function
=====================================   ======================
0                                       Allows rudder input to be used in only MANUAL,STABILIZE,and ACRO modes. In other modes, rudder input mixing is controlled by the :ref:`STICK_MIXING<STICK_MIXING>` parameter.
1                                       Forces center throttle stick to be :ref:`AIRSPEED_CRUISE<AIRSPEED_CRUISE>` instead of midway between :ref:`AIRSPEED_MAX<AIRSPEED_MAX>` and  :ref:`AIRSPEED_MIN<AIRSPEED_MIN>` when using an airspeed sensor in FBWB and CRUISE modes.
2                                       Disable the attitude checks for AUTOTAKEOFF and TAKEOFF mode start which must be less than +/-30 degrees roll and 45 degrees pitch.
3                                       Forces target airspeed to :ref:`AIRSPEED_CRUISE<AIRSPEED_CRUISE>` in CRUISE and FBWB modes when airspeed sensor is being used, ignoring throttle stick position.
4                                       Climb to :ref:`RTL_ALTITUDE<RTL_ALTITUDE>` altitude before turning toward home in RTL
5                                       Enable :ref:`Yaw Damping Controller<yaw-controller-tuning>` to be active in ACRO mode
6                                       Suppress speed scaling during auto takeoffs to be 1 or less to prevent oscillations when not using an airspeed sensor.
7                                       Enable default airspeed EKF fusion for takeoff (Advanced users only)
8                                       Remove :ref:`PTCH_TRIM_DEG<PTCH_TRIM_DEG>` offset on the GCS horizon to show pitch relative to AHRS trim (ie the attitude at which the autopilot was calibrated,unless manually changed)
9                                       Remove :ref:`PTCH_TRIM_DEG<PTCH_TRIM_DEG>` on the OSD horizon to show pitch relative to AHRS trim (ie the attitude at which the autopilot was calibrated,unless manually changed)
10                                      Adjust mid-throttle to be :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` in non-auto throttle modes except MANUAL,instead of midway between MAX and MIN stick values (note that the RCx_TRIM value for the throttle channel (x) MUST BE set to center stick value)
11                                      Disable suppression of fixed wing rate gains in ground mode
12                                      Enable FBWB style loiter altitude control if STICK_MIXING is enabled
=====================================   ======================

Default is no options enabled ("0"). Setting the bit will enable that function. For example, if forcing target airspeed in FBWB and CRUISE modes is desired, a value of "8" (bit 3 =1) would be set.

.. note:: Normally, PTCH_TRIM_DEG is subtracted from the AHRS pitch so that the artificial horizon shows pitch as if the autopilot was calibrated with aircraft level position set at PTCH_TRIM_DEG instead of flat.  This normally results in the artificial horizon indicating 0 pitch when in cruise at desired cruise speed. PTCH_TRIM_DEG is the pitch trim that would be required in stabilized modes to maintain altitude at nominal cruise airspeed and throttle, and for most planes is 1-3 degrees positive, depending on the aircraft design (see :ref:`tuning-cruise`).