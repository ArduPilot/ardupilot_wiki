.. _failsafe-landing-page:

=========
Failsafes
=========

Sub has a number of failsafe mechanisms to ease vehicle recovery/prevent wandering in the event that vehicle control is lost. The main failsafe topics are listed below.

.. note:: Once a Radio Failsafe, Battery Failsafe, GCS Failsafe or Pilot Control Loss Failsafe is entered and the failsafe action taken involves a mode change to the vehicle, it will remain in that mode until the pilot changes the mode directly.

.. toctree::
    :maxdepth: 1

    Radio Failsafe <radio-failsafe>
    Battery Failsafe <failsafe-battery>
    GCS Failsafe <gcs-failsafe>
    Pilot Control Failsafe <pilot-control-failsafe>
    Internal Leak Failsafe <internal-leak-failsafe>
    Internal Pressure Failsafe <internal-pressure-failsafe>
    Internal Temperature Failsafe <internal-temperature-failsafe>
    EKF Failsafe <ekf-inav-failsafe>
    Crash Check <crash_check>
    Independent Watchdog <common-watchdog>

