.. _soaring-speed-to-fly:

Speed to fly
============

Airspeed Management (Optional)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is also possible to let soaring control the airspeed in certain situations.

In THERMAL mode, the parameter :ref:`SOAR_THML_ARSPD<SOAR_THML_ARSPD>` sets the target airspeed in metres per second.

In AUTO, FBWB and CRUISE modes, the parameter :ref:`SOAR_CRSE_ARSPD<SOAR_CRSE_ARSPD>` controls the target airspeed when the aircraft is gliding.

 - Set :ref:`SOAR_CRSE_ARSPD<SOAR_CRSE_ARSPD>` to a positive number in metres per second to use this as the constant target airspeed.

 - Set :ref:`SOAR_CRSE_ARSPD<SOAR_CRSE_ARSPD>` to -1 to automatically calculate the best airspeed based on speed-to-fly theory. The best airspeed is based on current estimated lift/sink, headwind and the setting of :ref:`SOAR_VSPEED<SOAR_VSPEED>`. See the below talk for more information.

If one or the other of the above parameters are zero, or when the aircraft is using power (throttle > 0), :ref:`AIRSPEED_CRUISE<AIRSPEED_CRUISE>` is used in the corresponding flight modes as usual.

..  youtube:: Z-CZkG0lshc
    :width: 100%
