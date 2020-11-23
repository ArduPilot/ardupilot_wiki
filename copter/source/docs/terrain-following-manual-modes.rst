.. _terrain-following-manual-modes:

================
Surface Tracking
================

Copter supports low altitude Surface Tracking in nearly all modes including pilot controlled modes like :ref:`AltHold <altholdmode>`, :ref:`Loiter <loiter-mode>` and :ref:`PosHold <poshold-mode>` and also fully autonomous modes including :ref:`AUTO <auto-mode>` and :ref:`Guided <ac2_guidedmode>`.

Additionally, Terrain Following is supported in autonomous modes using a terrain height data base stored on the autopilot's SD card and/or obtained real-time from the Ground Control Station.

This page describes the Surface Following setup .  For autonomous modes using downloaded data for Terrain Following please see the :ref:`terrain following for autonomous modes <terrain-following>` wiki page.

..  youtube:: 3I06AOwIQVY
    :width: 100%

Setup and Configuration
-----------------------

- Connect a downward facing :ref:`lidar or sonar <common-rangefinder-landingpage>` to the vehicle
- If necessary adjust the :ref:`RNGFND_GAIN <RNGFND_GAIN>` parameter to increase or decrease the response to changes in reported altitude from the range finder
- An :ref:`auxiliary switch <common-auxiliary-functions>` (Copter 4.0 and later), or  :ref:`channel-7-and-8-options`, can be configured to turn on/off use of the rangefinder.

.. warning::

    Do not set the :ref:`EK2_ALT_SOURCE <EK2_ALT_SOURCE>` or ``EK3_ALT_SOURCE`` parameters.  These parameters should be left at "0" (barometer).

    Do not set the :ref:`EK2_RNG_USE_HGT <EK2_RNG_USE_HGT>`  or :ref:`EK3_RNG_USE_HGT <EK3_RNG_USE_HGT>` parameters.  These parameters should be left at "-1".

How does it work?
-----------------

- When the rangefinder can "see" the ground, the pilot's throttle stick adjusts the target altitude above the ground (i.e. the attitude corrected distance from the range finder)
- When the vehicle climbs out of the rangefinder's range the pilot's stick returns to directly controlling the vehicle's target climb rate (i.e. EKF estimated climb rate based on barometer and accelerometer).  When the vehicle comes back within the range of the ground, the target altitude above the terrain is reset to the current altitude above terrain. For more information, see :ref:`Understanding Altitude in ArduPilot <common-understanding-altitude>`.

