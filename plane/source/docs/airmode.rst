.. _airmode:

=======
AirMode
=======

This is not an actual flight mode, but rather, an important feature of QACRO and QSTABILIZE modes. Other flight control software refer to it as a flight mode, so for consistency, its included in the QuadPlane flight modes documentation.

What it is
==========

Normally, in QuadPlane, if throttle is lowered to idle in QACRO or QSTABILIZE modes, the stabilization is removed. While this prevents movement due to vibration while on the ground at idle throttle, or when using rudder arming, it prevents stabilization in flight for aerobatic movements or rapid descents.

Instead, stabilization even at zero throttle can be enabled/disabled for these modes using an RC channel switch via ``RCx_OPTION`` = 84 (Airmode). 

.. note:: if :ref:`ARMING_RUDDER<ARMING_RUDDER>` is set to 2 (arm and disarm using rudder stick), while in Airmode, you cannot disarm using rudder. This allows full rudder stick throws without the chance of a disarm occurring.

.. warning:: If Airmode is enabled by this switch while disarmed, and you arm via rudder, the aircraft will immediately attempt to yaw to the right while still on the ground.

If you arm using an RC switch (``RCx_OPTION`` = 154), AirMode will be active but can be disabled with the above switch. If you arm using an RC switch using the (``RCx_OPTION`` = 153) function, then AirMode can only be enabled or disabled using the ``RCx_OPTION`` = 84 switch function.
