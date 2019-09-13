.. _acro-mode:

=========
Acro Mode
=========

In Acro mode the user's steering stick controls the vehicle's turn rate and the throttle stick controls the vehicle's speed. 

-  the :ref:`ACRO_TURN_RATE <ACRO_TURN_RATE>` parameter controls the maximum turn rate the user's steering stick can request. The turn rate varies linearly from zero to :ref:`ACRO_TURN_RATE <ACRO_TURN_RATE>` as the RC input varies from neutral to full deflection. Once the input returns to neutral, the vehicle will attempt to hold heading, compensating for external influences, ie. "heading hold".
-  the top speed is interpolated from the :ref:`CRUISE_THROTTLE <CRUISE_THROTTLE>` and :ref:`CRUISE_SPEED <CRUISE_SPEED>` parameters.  These parameters are described on the :ref:`Tuning Speed and Throttle <rover-tuning-throttle-and-speed>` page.
- :ref:`Object Avoidance <rover-object-avoidance>` is active in this mode (if configured).

This mode is perhaps the most useful mode for tuning the vehicle's :ref:`Steering Rate <rover-tuning-steering-rate>` controller.
