.. _flight-options:

==============
Flight Options
==============


:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>` is a bitmask that allows configuring several alterations to Plane's behavior.


=====================================   ======================
:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>`   Function
=====================================   ======================
0                                       Allows rudder input to be used in only MANUAL,STABILIZE,and ACRO modes. In other modes, rudder input mixing is controlled by the :ref:`STICK_MIXING<STICK_MIXING>` parameter.
1                                       Forces center throttle stick to be :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` instead of midway between :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` and  :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` when using an airspeed sensor in FBWB and CRUISE modes.
2                                       Disable the attitude checks for AUTOTAKEOFF and TAKEOFF mode start which must be less than +/-30 degrees roll and 45 degrees pitch.
3                                       Forces target airspeed to :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` in CRUISE and FBWB modes when airspeed sensor is being used, ignoring throttle stick position.
=====================================   ======================

Default is no options enabled ("0"). Setting the bit will enable that function. For example, if forcing target airspeed in FBWB and CRUISE modes is desired, a value of "8" (bit 3 =1) would be set.