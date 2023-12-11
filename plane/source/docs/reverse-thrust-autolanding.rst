.. _reverse-thrust-autolanding:

====================
Reverse Thrust Setup
====================

Some ESC's allow for reversing motor direction. When using reverse on the propeller it will generate a negative thrust which can be used to reduce your airspeed. During a steep landing approach this method can be used to maintain a stable and low airspeed allowing you to land much more softly and precisely. Reverse thrust can also be automatically used during automatic landings (see below) or during other throttle controlled modes to steepen descents. 
See  :ref:`Reverse Thrust Setup<reverse-thrust-setup>` for details on setting up reverse thrust.

.. warning:: enabling reverse thrust automatically sets the SERVOx_TRIM value to LOW throttle stick for the throttle output servo. SERVOx_TRIM is normally ignored, and low stick is SERVOx_MIN! Arming in this condition with an ESC not properly setup will spin-up the motor, possibly dramatically!

Reverse-Thrust in AutoLanding and Throttle Controlled Modes
===========================================================

.. note:: To use this feature in automatic landings, it is highly recommend to use an airspeed sensor and a rangefinder (see above) for an accurate altitude.

The below video is an example of a Skywalker X8 performing an auto-landing with a 15 degree slope. The target is the hat on the ground showing it is possible to get repeatable high precision landings where the final position error was dictated by the GPS position error. This particular aircraft has been landed at 20deg and 25deg slopes too. You may have different results depending on weight of aircraft and available thrust from motor/propeller. Typically a Skywalker X8 would need a shallow slope such at 6 to 10deg.

..  youtube:: kdw8vjbttNo
    :width: 100%

.. _reverse-thrust-key-parameters:

Key Parameters
--------------

The key parameters that control reverse thrust landing in addition to the ones :ref:`listed in section 1.1 <automatic-landing_key_parameters>`  and :ref:` Reverse Thrust setup <reverse-thrust-setup>` are:

-  :ref:`LAND_PF_ALT <LAND_PF_ALT>`
-  :ref:`LAND_PF_SEC <LAND_PF_SEC>`
-  :ref:`LAND_PF_ARSPD <LAND_PF_ARSPD>`
-  :ref:`USE_REV_THRUST <USE_REV_THRUST>`
-  :ref:`TECS_APPR_SMAX <TECS_APPR_SMAX>`
-  ``SERVOx_TRIM`` where x is the throttle output channel
-  :ref:`THR_MIN <THR_MIN>`

The maximum amount of reverse thrust used in autopilot throttle controlled modes is set by :ref:`THR_MIN <THR_MIN>`. A value of -100 provides the maximum, while -20 would provide 20% of  maximum, and so on.

The :ref:`USE_REV_THRUST <USE_REV_THRUST>` parameter is a bit mask that allows the setting of when reverse thrust can be activated, as when autolanding, or in other throttle controlled modes.

If an RC channel's ``RCx_OPTION`` auxiliary function has been set to "64", then activating the switch will also provide access to the reverse thrust capability in non-throttle controlled modes, as shown in the :ref:`Reverse Thrust Setup<reverse-thrust-setup>` section.

.. note:: Airbrakes can also be automatically deployed during reverse thrust operation. See :ref:`Airbrakes<airbrakes-on-plane>`.


ESC (Electronic Speed Controller)
---------------------------------

Most important is to set the :ref:`SERVO3_TRIM <SERVO3_TRIM>` (assuming the esc/motor is attached to output 3) to the point that the ESC is idle, usually around mid-range (1500us) to create an output curve that has :ref:`SERVO3_MAX<SERVO3_MAX>` for full forward thrust, and :ref:`SERVO3_MIN<SERVO3_MIN>` for full reverse thrust. This should be done AFTER the RC Calibrations setup step. 

Hardware selection and programming
++++++++++++++++++++++++++++++++++

Many ESCs can operate in forwards and reverse, however that is usually not a stock feature and may need to be reprogrammed to do it. Any SimonK and BLHeli compatible ESC can be flashed to support reverse thrust.

`Here's info about BLHeli compatible ones <https://blhelisuite.wordpress.com/>`__.


Hardware configuration
++++++++++++++++++++++

.. note::

   Remove propeller while configuring ESCs and thrust parameters

Configure your ESC for reverse thrust by changing its neutral point.
Many ESC require custom firmware to accomplish this. Search Google or your ESC manufacturer for instructions on how to configure your particular ESC.

Set  Minimum PWM to 1000, mid to 1500, and maximum to 2000, corresponding to maximum reverse thrust,idle, and maximum positive thrust from the ESC/Motor.