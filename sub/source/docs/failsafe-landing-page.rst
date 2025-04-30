.. _failsafe-landing-page:

========
Failsafe
========

Sub has a number of failsafe mechanisms to ease vehicle recovery/prevent wandering in the event that vehicle control is lost. The main failsafe topics are listed below.

.. note:: Once a Radio Failsafe, Battery Failsafe, GCS Failsafe or Terrain Data Loss Failsafe is entered and the failsafe action taken involves a mode change to the vehicle, it will remain in that mode until the pilot changes the mode directly.

.. toctree::
    :maxdepth: 1

    Radio Failsafe <radio-failsafe>

Battery Failsafe <failsafe-battery>
GCS Failsafe <gcs-failsafe>
EKF Failsafe <ekf-inav-failsafe>
Vibration Failsafe <vibration-failsafe>
Terrain Data Loss Failsafe <terrain-following>
Crash Check <crash_check>
Internal Leak <internal-leak>
Internal Pressure <internal-pressure>
Internal Temp <internal-temp>
Independent Watchdog <common-watchdog>
