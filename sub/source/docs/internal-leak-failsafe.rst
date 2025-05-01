.. _internal-leak-failsafe:

======================
Internal Leak Failsafe
======================

If sub is equipped with a leak detector internally, then a failsafe can be enabled when a leak is detected. Sub provides the ability to have up to three detectors. Each detector can be either analog or digital. When a leak from any detector is detected, a failsafe can be enabled to occur.

For the remainder of this article, the first leak detector's parameters will used for example.

When the failsafe will trigger
==============================
If :ref:`FS_LEAK_ENABLE<FS_LEAK_ENABLE>` is non-zero, then if the detector attached to a logical pin on the autopilot designated by :ref:`LEAK1_PIN<LEAK1_PIN>` is analog (:ref:`LEAK1_TYPE<LEAK1_TYPE>` = "0"), its considered active ("1") if its voltage is > 2.0V. Digital sensors (:ref:`LEAK1_TYPE<LEAK1_TYPE>` = "1") present a logic level. Whether the active level of an analog sensor or the logic level from a digital sensor is taken as "dry" is determined by the :ref:`LEAK1_LOGIC<LEAK1_LOGIC>` parameter.

What will happen
================
When the failsafe occurs, the action taken is determined by the value of :ref:`FS_LEAK_ENABLE<FS_LEAK_ENABLE>`:

-  **Disabled** (Value 0) will disable the failsafe entirely.
-  **Warn Only** (Value 1) will send a GCS warning message.
-  **Enter SURFACE Mode** (Value 2).