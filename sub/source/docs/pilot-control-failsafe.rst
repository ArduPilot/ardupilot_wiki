.. _pilot-control-failsafe:

======================
Pilot Control Failsafe
======================

Sub provides a failsafe mechanism in case pilot manual control is lost. Pilot control in Sub is provided by either or both of two ways: via GCS MAVLink RC override messages( stimulated by GCS Joystick) or by the optional RC control link.

When the failsafe will trigger
==============================
If :ref:`FS_PILOT_INPUT<FS_PILOT_INPUT>` is non-zero, then lack of both valid RC and MAVLink RC overrides from the GCS for greater than :ref:`FS_PILOT_TIMEOUT<FS_PILOT_TIMEOUT>` will initiate the Pilot Input failsafe.

.. note:: if the RC failsafe is enabled with :ref:`FS_THR_ENABLE<FS_THR_ENABLE>` nonzero, then Pilot Input failsafe will never be activated, only loss of RC will activate a failsafe even if the MAVLink overrides are lost from the GCS.

What will happen
================
When a Pilot Control failsafe is triggered, the sub can be configured via parameters to do nothing, send the GCS a warning message, or Disarm.

If the failsafe clears (i.e. RC control or MAVLink overrides from GCS is restored) the sub will remain in its failsafe mode. If it is disarmed, and the pilot wished to re-take control he/she would need to rearm the sub . This can be done via RC or GCS.

Setting the failsafe
====================

In Mission Planner's  full parameter list or full parameter tree, set the :ref:`FS_PILOT_INPUT <FS_PILOT_INPUT>` parameter to:

-  **Disabled** (Value 0) will disable the GCS failsafe entirely.
-  **Warn Only** (Value 1) will send a GCS warning message.
-  **Disarm** (Value 2)