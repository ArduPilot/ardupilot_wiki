.. _internal-temperature-failsafe:

=============================
Internal Temperature Failsafe
=============================
Sub can use the barometer normally included with an autopilot to determine if the temperature inside the vehicle has increased too much, which can cause vehicle electronics to malfunction, or may indicate a fire. The trigger temperature is set by :ref:`FS_TEMP_MAX<FS_TEMP_MAX>`. If it is exceeded, then the following actions is taken as set by the value of the :ref:`FS_TEMP_ENABLE<FS_PRESS_ENABLE>` parameter:

-  **Disabled** (Value 0) will disable the failsafe entirely.
-  **Warn Only** (Value 1) will send a GCS warning message.

