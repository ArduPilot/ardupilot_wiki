.. _airmode:

=======
AirMode
=======

This is not an actual flight mode, but rather, an important feature of ACRO and STABILIZE mode. Other flight control software refer to it as a flight mode, so for consistency, its included in the Copter flight modes documentation

What it is
==========

Normally, in Copter, if arming is setup via the rudder stick, if throttle is lowered to idle in ACRO mode, the stabilization is removed. While this prevents movement due to vibration while on the ground at idle throttle, it prevents stabilization in flight for aerobatic movements or rapid descents.

If a transmitter switch is used with  ``RCx_OPTION`` ARM/DISARM with Airmode (option 154) and used to arm, then stabilization at idle throttle is still fully active, which is often referred to as AIRMODE.

.. note :: to avoid enabling AIRMODE but still use an ARM/DISARM switch, use :ref:`Auxiliary function <common-auxiliary-functions>` option "153" for the switch.

AIRMODE can also be set to be active without using the ``RCx_OPTION`` switch to arm. Setting an RC channel to ``RCx_OPTION`` = 84, allows enabling or disabling AIRMODE in ACRO and STABILIZE modes directly. In addition, setting bit 0 of :ref:`ACRO_OPTIONS<ACRO_OPTIONS>` will activate AIRMODE in ACRO mode (only) all the time, except if overridden by the above switch, if used.

.. note:: AIRMODE has no effect in Traditional Helicopter

.. note:: It isn't sufficient to just enable arming on an ``RCx_OPTION`` switch, you *also* have to actually use it for arming. So if you have it on a switch but then use rudder-arming then you won't get "airmode". 
