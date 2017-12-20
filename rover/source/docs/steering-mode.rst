.. _steering-mode:

=============
Steering Mode
=============

In Steering mode the user's steering stick controls the vehicle's lateral acceleration and the throttle stick controls the vehicle's speed.

- the :ref:`TURN_RADIUS <TURN_RADIUS>` parameter controls the maximum aggressiveness of the turns.  A smaller number results in more aggressive turns.
- when not moving, skid-steering vehicles will pivot in response to steering input but regular steering-throttle rovers will show almost no steering response
- the top speed is interpolated from the :ref:`CRUISE_THROTTLE <CRUISE_THROTTLE>` and :ref:`CRUISE_SPEED <CRUISE_SPEED>` parameters.  These parameters are described on the :ref:`Tuning Speed and Throttle <rover-tuning-throttle-and-speed>` page
- :ref:`object avoidance <rover-object-avoidance>` is active in this mode (if configured)
- the :ref:`SPEED_TURN_GAIN <SPEED_TURN_GAIN>` parameter will slow the vehicle as it does in :ref:`Auto <auto-mode>`.  This is described briefly on the :ref:`Tuning Navigation <rover-tuning-navigation>` page
- this mode can be used to tune the :ref:`steering rate <rover-tuning-steering-rate>` and :ref:`speed controllers <rover-tuning-throttle-and-speed>` ahead of attempting to tune :ref:`navigation controls <rover-tuning-navigation>`
