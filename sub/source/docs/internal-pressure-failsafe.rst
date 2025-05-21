.. _internal-pressure-failsafe:

==========================
Internal Pressure Failsafe
==========================
Sub can use the barometer normally included with an autopilot to determine if the pressure inside the vehicle has increased too much. This could occur if the enclosure is bowing in and near implosion, or if something is burning/off-gassing, or if there's a big enough leak that water is forcing its way in. It can also be from the internal electronics heat.

The amount of the pressure in pascals is set by :ref:`FS_PRESS_MAX<FS_PRESS_MAX>`. If it is exceeded, then the following actions is taken as set by the value of the :ref:`FS_PRESS_ENABLE<FS_PRESS_ENABLE>` parameter:

-  **Disabled** (Value 0) will disable the failsafe entirely.
-  **Warn Only** (Value 1) will send a GCS warning message.

