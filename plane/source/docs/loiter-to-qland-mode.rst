.. _loiter-to-qland-mode:

=================================
LOITER to Altitude and QLAND Mode
=================================

If the previous mode was a fixed wing mode, this mode will perform a descending fixed wing LOITER down to :ref:`Q_RTL_ALT<Q_RTL_ALT>` and then switch to :ref:`QLAND mode <qland-mode>`. If the aircraft was already performing a loiter when switching to this mode the LOITER center does not change, otherwise the LOITER center will become the same location it is at when entering the mode.

.. note:: the QLAND target will be directly below the point it switches to QLAND (ie on the loiter circle path, not its center)

If in a VTOL mode, or below  :ref:`Q_RTL_ALT<Q_RTL_ALT>`, when switching to this mode, it will change immediately to :ref:`QLAND mode <qland-mode>`.

This is useful as a Battery Failsafe action to VTOL land the vehicle with as little battery usage as possible if at higher altitudes than using :ref:`QLAND mode <qland-mode>` by itself.

The fixed wing loiter radius is set by :ref:`WP_LOITER_RAD<WP_LOITER_RAD>` and descent rate is :ref:`FBWB_CLIMB_RATE<FBWB_CLIMB_RATE>` until :ref:`Q_RTL_ALT<Q_RTL_ALT>` is reached, and when reached, it will switch to :ref:`QLAND mode <qland-mode>`.


