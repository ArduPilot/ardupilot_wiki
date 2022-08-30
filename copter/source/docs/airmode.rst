.. _airmode:

=======
AirMode
=======

This is not an actual flight mode, but rather, an important feature of ACRO and STABILIZE modes. Other flight control software refer to it as a flight mode, so for consistency, its included in the Copter flight modes documentation

What it is
==========

Normally, in Copter, if arming is setup via the rudder stick, if throttle is lowered to idle in ACRO or STABILIZE modes, the stabilization is removed. While this prevents movement due to vibration while on the ground at idle throttle, it prevents stabilization in flight for aerobatic movements or rapid descents.

If an ``RCx_OPTION`` is enabled for switch ARM/DISARM (41) and used to arm, then stabilization at idle throttle is still fully active, which is often referred to as AIRMODE.

AIRMODE can also be set to be active without using the ``RCx_OPTION`` ARM/DISARM switch to arm. Setting an RC channel to ``RCx_OPTION`` = 84, allows enabling or disabling AIRMODE in ACRO and STABILIZE modes directly. In addition, setting bit 0 of :ref:`ACRO_OPTIONS<ACRO_OPTIONS>` will activate AIRMODE in those modes all the time.

.. note:: AIRMODE has no effect in Traditional Helicopter

.. note:: It isn't sufficient to just enable arming on an ``RCx_OPTION`` switch, you *also* have to actually use it for arming. So if you have it on a switch but then use rudder-arming then you won't get "airmode". 
