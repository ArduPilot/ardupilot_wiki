.. _sprayer:

============
Crop Sprayer
============

Copter 3.4 (and higher) include support for a crop sprayer.  This feature allows a autopilot connected to a PWM operated pump and (optionally) spinner to control the rate of flow of liquid fertilizer based on the vehicle speed.

..  youtube:: O8ZnxkXMv6A
    :width: 100%

Slightly out-of-date video showing a copter using the Sprayer feature (jump to 2:25 to see the sprayer in action).

Required Hardware
=================

   .. image:: ../images/sprayer_EnRoute_AC940D.jpg
       :target: https://enroute.co.jp/products/ac940d/

A multicopter such as the `EnRoute AC 940-D <https://enroute.co.jp/products/ac940d/>`_ with a PWM controlled pump and optionally a PWM controlled spinning mechanism (EnRoute vehicle does not require this secondary spinner control).

The pump controls the rate of flow of the fertilizer.

The optional spinner should be attached to the end of the spraying nozzles and distributes the fertilizer to a wider area.

Enabling the Sprayer
====================

-  Connect your autopilot to the ground station (i.e. Mission Planner)
-  Set the :ref:`SPRAY_ENABLE <SPRAY_ENABLE>` parameter to 1 and refresh parameters (the sprayer is not a commonly used feature so its other parameters are initially hidden)
-  Connect the pump to one of the autopilot's auxiliary PWM outputs (like Pixhawk AUX1) and set the appropriate SERVO*_FUNCTION  or RC*_FUNCTION to 22 (where "*" is the RC output number, i.e. if the pump is connected to a Pixhawk's AUX1, set :ref:`SERVO9_FUNCTION <SERVO9_FUNCTION>` to 22).
-  Connect the optional spinner to another auxiliary output port and set SERVO*_FUNCTION or RC*_FUNCTION to 23 (i.e. if using a Pixhawk's AUX2, set :ref:`SERVO10_FUNCTION <SERVO10_FUNCTION>` to 23)
-  To allow the pilot to turn on/off the sprayer set an auxiliary switch (i.e. CH7_OPT in versions prior to Copter-4.0 or an ``RCx_OPTIONS`` in Copter-4.0 or higher) to "15"

.. note::

   in Copter 3.5 (and higher) the RCx_FUNCTION parameters were renamed to SERVOx_FUNCTION to better separate their meaning from the RC input.

Configuring the pump
====================
-  PWM range used to control the pump and spinner can be configured by setting the SERVO*_MIN/RC*_MIN, SERVO*_MAX/RC*_MAX parameters corresponding to the pwm output channels the pump and spinner are connected to.
-  :ref:`SPRAY_PUMP_MIN <SPRAY_PUMP_MIN>` controls the minimum pump rate (expressed as a percentage).  By default this is 0% meaning the pump will completely stop if the vehicle stops.
-  :ref:`SPRAY_PUMP_RATE <SPRAY_PUMP_RATE>` controls the pump rate (expressed as a percentage) when the vehicle is travelling at 1m/s.  By default this is 10%.  The pump rate increases linearly with the vehicle speed meaning by default the pump will reach 100% at 10m/s.
-  :ref:`SPRAY_SPINNER <SPRAY_SPINNER>` sets the pwm value sent to the spinner when the pump is on.
-  :ref:`SPRAY_SPEED_MIN <SPRAY_SPEED_MIN>` sets the minimum vehicle speed (in cm/s) at which the pump will operate.  Default is 100 meaning the pump will begin when the vehicle is travelling at or above 1m/s.

