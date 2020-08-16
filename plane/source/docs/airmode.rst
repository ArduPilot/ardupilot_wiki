.. _airmode:

=======
AirMode
=======

.. note:: this feature will be available in Plane 4.1 firmware and later.

This is not an actual flight mode, but rather, an important feature of QACRO and QSTABILIZE modes. Other flight control software refer to it as a flight mode, so for consistency, its included in the QuadPlane flight modes documentation.

What it is
==========

Normally, in QuadPlane, if throttle is lowered to idle in QACRO or QSTABILIZE modes, the stabilization is removed. While this prevents movement due to vibration while on the ground at idle throttle, or when using rudder arming, it prevents stabilization in flight for aerobatic movements or rapid descents.

Instead, stabilization even at zero throttle can be enabled/disabled for these modes using an RC channel switch via ``RCx_OPTION`` = 84 (Airmode). 

.. note:: if :ref:`ARMING_RUDDER<ARMING_RUDDER>` is set to 2 (arm and disarm using rudder stick), while in Airmode, you cannot disarm using rudder. This allows full rudder stick throws without the chance of a disarm occurring.

.. warning:: If Airmode is enabled by this switch while disarmed, and you arm via rudder, the aircraft will immediately attempt to yaw to the right while still on the ground.

Alternatively, you can set :ref:`Q_OPTIONS<Q_OPTIONS>` bit 9 (512) to allow Airmode to be activated if armed using an RC channel switch (``RCx_OPTION`` = 41). Arming via rudder, in this case, will not activate Airmode, as it does when using the Airmode RC option switch. This option bit is ignored if using an RC switch to enable/disable Airmode.

.. note:: You cannot activate Airmode without using either the switch option or the :ref:`Q_OPTIONS<Q_OPTIONS>` method.




