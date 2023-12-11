.. _tkoff-rpm-min:

=============================
Dead Motor Takeoff Prevention
=============================

Ardupilot provides mechanism to prevent flips on takeoff if a motor, or motors, is not operating at a minimum rpm due to damage. It requires that ESC RPM telemetry is being used. See the ``ESC Telemetry - Average Motor RPM`` section of :ref:`common-rpm` for setup.

Then, by setting the :ref:`TKOFF_RPM_MIN<TKOFF_RPM_MIN>` slightly below the :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>`, if the average rpm is lower than this, throttle increases will not be passed on, preventing a takeoff and an immediate flip.