.. _ground-effect-compensation:

==========================
Ground Effect Compensation
==========================

Copter includes ground effect compensation which reduces the weighting of the barometer (in favour of the accelerometers) when the vehicle is likely taking off or landing.  This reduces the bounce sometimes seen when landing vehicles with short legs compared to the length of their propellers.

Ground effect compensation is enabled by default.

Setup
=====

- :ref:`GNDEFF_ALT<GNDEFF_ALT>` (default 0.5m): compensation during takeoff ends once the vehicle has climbed this far above the takeoff location, and compensation during landing is only applied below this altitude. Zero keeps compensation enabled but removes the altitude checks: any gentle descent is treated as a landing (as in 4.7 and earlier), and takeoff compensation ends once the vehicle has climbed at all after :ref:`GNDEFF_TMO<GNDEFF_TMO>` (or after 5 seconds). A negative value disables ground effect compensation.
- :ref:`GNDEFF_TMO<GNDEFF_TMO>` (default 2 seconds): minimum time after takeoff before compensation can end. Takeoff compensation always ends after 5 seconds regardless of altitude. Vehicles with strong barometer disturbance from propwash may benefit from values of 2 to 5 seconds. Zero means only the altitude check is used.

If the EKF has a height above ground estimate, from a downward facing :ref:`rangefinder <common-rangefinder-landingpage>` or optical flow, it is used for the altitude checks.  This height is measured relative to the reading while on the ground, so :ref:`GNDEFF_ALT<GNDEFF_ALT>` is the clearance above the vehicle's resting height.  Otherwise the height change since takeoff is used, assuming flat ground.  In that case, if a horizontal position (e.g. GPS) is available and the vehicle is more than 20m from the takeoff location, the landing altitude check is not used and any gentle descent is treated as a landing.

.. note:: Firmware 4.7 and earlier used ``GND_EFFECT_COMP`` (1 = enabled, the default) instead. A stored value of 0 is converted to :ref:`GNDEFF_ALT<GNDEFF_ALT>` = -1 when upgrading.

Video
=====

..  youtube:: ljSV-36MOOU
    :width: 100%
