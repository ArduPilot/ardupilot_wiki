.. _failsafe-landing-page:

========
Failsafe
========

Copter has a number of failsafe mechanisms to ease vehicle
recovery/prevent wandering in the event that vehicle control is lost.
The main failsafe topics are listed below.

.. note:: Once a Radio Failsafe, Battery Failsafe, GCS Failsafe or Terrain Data Loss Failsafe is entered and the failsafe action taken involves a mode change to the vehicle, it remain in that mode until the pilot changes the mode directly.

.. toctree::
    :maxdepth: 1

    Radio Failsafe <radio-failsafe>
    Battery Failsafe <failsafe-battery>
    GCS Failsafe <gcs-failsafe>
    EKF Failsafe <ekf-inav-failsafe>
    Vibration Failsafe <vibration-failsafe>
    Terrain Data Loss Failsafe (see section on loss of terrain data induced RTL) <terrain-following>
    Crash Check <crash_check>
    Parachute <parachute>
    Independent Watchdog <common-watchdog>
