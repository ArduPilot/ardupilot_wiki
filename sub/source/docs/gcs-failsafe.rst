.. _gcs-failsafe:

============
GCS Failsafe
============
The Ground Station Control (GCS) failsafe controls how Sub will behave if contact with the GCS is lost.  The GCS failsafe monitors the time since the last MAVLink heartbeat from the GCS.  If no heartbeat is received :ref:`FS_GCS_TIMEOUT<FS_GCS_TIMEOUT>` seconds (Default is 5 seconds), the GCS failsafe event will trigger based on your parameter settings. Note that if no GCS is ever connected, the GCS failsafe will remain inactive regardless of parameter settings.

.. note::

   Sub also supports other failsafes, see the :ref:`failsafe-landing-page`.

When the failsafe will trigger
==============================
The following situations can cause a loss of GCS MAVLink heartbeat, triggering a GCS failsafe event after :ref:`FS_GCS_TIMEOUT<FS_GCS_TIMEOUT>` seconds:

-  The operator turns off or otherwise disconnects the GCS
-  The vehicle travels beyond the range of the GCS telemetry radios
-  The GCS telemetry mechanism on either end loses power
-  Wires connecting the GCS telemetry to the autopilot or ground equipment become disconnected

What will happen
================
When a GCS failsafe is triggered, the vehicle can be configured via parameters to do nothing, send the GCS a warning message, disarm, or change to ALTHOLD or SURFACE modes.


If the failsafe clears (i.e. GCS reconnects and MAVLink heartbeat is restored) the vehicle will remain in its failsafe mode. It will **not** automatically return to the flight mode that was active before the failsafe was triggered, if it was changed.

Setting the failsafe
====================

In Mission Planner's  full parameter list or full parameter tree, set the :ref:`FS_GCS_ENABLE <FS_GCS_ENABLE>` parameter to:

-  **Disabled** (Value 0) will disable the GCS failsafe entirely.
-  **Warn Only** (Value 1) will send a GCS warning message.
-  **Disarm** (Value 2)
-  **Enter ALTHOLD mode** (Value 3), if armed.
-  **Enter SURFACE mode** (Value 4), if armed.

