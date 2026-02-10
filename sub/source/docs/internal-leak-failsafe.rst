.. _internal-leak-failsafe:

======================
Internal Leak Failsafe
======================

If sub is equipped with a leak detector internally, then a failsafe can be enabled when a leak is detected. Sub provides the ability to have up to three detectors. Each detector can be either analog or digital. When a leak from any detector is detected, a failsafe can be enabled to occur.

For the remainder of this article, the first leak detector's parameters will used for example.

When the failsafe will trigger
==============================
If :ref:`FS_LEAK_ENABLE<FS_LEAK_ENABLE>` is non-zero, and a detector is attached to either an analog input or GPIO pin on the autopilot, set by the :ref:`LEAK1_PIN<LEAK1_PIN>`. Then if the detector is analog, it's considered active ("1") if its voltage is > 2.0V. Digital sensors present a normal digital logic level.

Whether the active level of an analog sensor or the logic level from a digital sensor is taken as "dry" is determined by the :ref:`LEAK1_LOGIC<LEAK1_LOGIC>` parameter.

.. note:: In ArduSub firmware prior or equal to 4.5, the input pin must be declared to be analog or digital with the ``LEAKx_TYPE`` parameter.

.. note:: Digital leak detectors should be connected to digital input pins. Some flight controllers support that on :ref:`servo channel ports <main-aux-out>`, in which case the relevant channel(s) must be configured as :ref:`common-gpios`.

What will happen
================
When the failsafe occurs, the action taken is determined by the value of :ref:`FS_LEAK_ENABLE<FS_LEAK_ENABLE>`:

-  **Disabled** (Value 0) will disable the failsafe entirely.
-  **Warn Only** (Value 1) will send a "Leak Detected" GCS warning message.
    - On a flight controller with notifying hardware, status LEDs will flash yellow and white, and the buzzer will play a cycle of falling tones
-  **Enter SURFACE Mode** (Value 2) will send a warning and return the vehicle to the surface.
