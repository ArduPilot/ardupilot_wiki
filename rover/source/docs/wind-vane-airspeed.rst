.. _wind-vane-airspeed:

=========================
Airspeed Sensor
=========================

Setting :ref:`WNDVN_SPEED_TYPE <WNDVN_SPEED_TYPE>` to 1 allows reading the wind speed from the :ref:`airspeed library <airspeed>`. This allows any pitot tube type
airspeed sensor to be used. It is important the :ref:`ARSPD_USE <ARSPD_USE>` and :ref:`ARSPD_AUTOCAL <ARSPD_AUTOCAL>` parameters are left at zero, they enable features
designed for aircraft that will not work with Rover. It may also be desirable to disable start up calibration with :ref:`ARSPD_SKIP_CAL <ARSPD_SKIP_CAL>`. If left
enabled the airspeed sensor will be zeroed at boot. This recalibration requires the sensor is sheltered from the wind, this may be hard on a sailing craft. Pitot tube
airspeed sensors must be pointed directly into the wind, in this case that would require mounting the sensor to the wind vane. Due to this mechanical complexity
other methods of sensing wind speed may be more convenient.
