.. _terrain-following-manual-modes:

================
Surface Tracking
================

Copter supports low altitude Surface Tracking, either a floor/ground or a ceiling, in nearly all modes including pilot controlled modes like :ref:`AltHold <altholdmode>`, :ref:`Loiter <loiter-mode>` and :ref:`PosHold <poshold-mode>`.

Additionally, :ref:`Terrain Following<terrain-following>` is supported in autonomous modes using a terrain height data base stored on the autopilot's SD card and/or obtained real-time from the Ground Control Station.

This page describes the Surface Following setup .  For autonomous modes using downloaded data for Terrain Following please see the :ref:`terrain following for autonomous modes <terrain-following>` wiki page.

..  youtube:: 3I06AOwIQVY
    :width: 100%

Setup and Configuration
-----------------------

- Connect a downward facing rangefinder(for floor/ground tracking) and/or an upward facing rangefinder (for ceiling tracking) :ref:`lidar or sonar <common-rangefinder-landingpage>` to the vehicle. 

- Which is being used is configured by the :ref:`SURFTRAK_MODE<SURFTRAK_MODE>` parameter or by: 
- An :ref:`auxiliary switch <common-auxiliary-functions>` (function "75"), or  :ref:`channel-7-and-8-options`, can be configured to turn on/off use of the rangefinder.
- the :ref:`SURFTRAK_TC<SURFTRAK_TC>` parameter controls the smoothing of the surface data. Increase if you are moving fast and getting perturbations in the flight path. Conversely, it can be lower to make the vehicle more responsive to the rangefinder data.

.. warning::

    Do not set the :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` to Rangefinder.  This parameter should be left at the default.

    Do not set :ref:`EK3_RNG_USE_HGT <EK3_RNG_USE_HGT>` parameter.  This parameter should be left at "-1".

How does it work?
-----------------

- When the rangefinder can "see" the ground, the pilot's throttle stick adjusts the target altitude above the ground (i.e. the attitude corrected distance from the range finder)
- When the vehicle climbs out of the rangefinder's range the pilot's stick returns to directly controlling the vehicle's target climb rate (i.e. EKF estimated climb rate based on barometer and accelerometer).  When the vehicle comes back within the range of the ground, the target altitude above the terrain is reset to the current altitude above terrain. For more information, see :ref:`Understanding Altitude in ArduPilot <common-understanding-altitude>`.

