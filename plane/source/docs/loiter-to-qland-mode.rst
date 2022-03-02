.. _loiter-to-qland-mode:

=================================
LOITER to Altitude and QLAND Mode
=================================

If the previous mode was a fixed wing mode, this mode will perform a descending fixed wing LOITER down to :ref:`Q_RTL_ALT<Q_RTL_ALT>` and then switch to :ref:`QLAND mode <qland-mode>`. If in a VTOL mode, or below  :ref:`Q_RTL_ALT<Q_RTL_ALT>`, when switching to this mode, it will change immediately to :ref:`QLAND mode <qland-mode>`.

This is useful as a Battery Failsafe action to VTOL land the vehicle with as little battery usage as possible if at higher altitudes than using :ref:`QLAND mode <qland-mode>` by itself.

The fixed wing loiter radius is set by :ref:`WP_LOITER_RAD<WP_LOITER_RAD>` and descent rate is :ref:`FBWB_CLIMB_RATE<FBWB_CLIMB_RATE>` to :ref:`Q_RTL_ALT<Q_RTL_ALT>` and when reached will switch to :ref:`QLAND mode <qland-mode>`.


