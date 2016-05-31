.. _quadplane-parameters:

QuadPlane Parameter setup
=========================

All QuadPlane specific parameters start with a "Q\_" prefix. The
parameters are very similar to the equivalent Copter parameters so if
you are familiar with those you should find setting up a QuadPlane is
easy.

Key parameters are:

-  To enable QuadPlane functionality you need to set the Q_ENABLE
   parameter to 1 and then refresh the parameter list
-  The Q_THR_MIN_PWM and Q_THR_MAX_PWM parameters used to set the
   PWM range of the quad motors (this allows them to be different from
   the range for the forward motor). These need to be set to the range
   your ESCs expect.
-  The most critical tuning parameters are Q_A_RAT_RLL_P and
   Q_A_RAT_PIT_P. These default to 0.25 but you may
   find significantly higher values are needed for a QuadPlane.
-  The Q_M\_SPIN_ARMED parameter is important for getting the right
   level of motor output when armed in a quad mode
-  It is recommended that you set ARMING_RUDDER to 2 to allow for
   rudder disarm. Alternatively you could have :ref:`MANUAL <manual-mode>`
   as one of your available flight modes (as that will shut down the
   quad motors). Please be careful not to use hard left rudder and zero
   throttle while flying or you risk disarming your motors.
-  The Q_THR_MID parameter is important for smooth transitions. It
   defaults to 500 which means 50% throttle for hover. If your aircraft
   needs more or less than 50% throttle to hover then please adjust
   this. That will prevent a throttle surge during transition as the
   altitude controller learns the right throttle level

.. note::

   The QuadPlane code requires GPS lock for proper operation. This is
   inherited from the plane code, which disables intertial estimation of
   attitude and position if GPS lock is not available. Do not try to fly a
   QuadPlane indoors. It will not fly well

