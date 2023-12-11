.. _deadreckoning-failsafe:

=======================
Dead Reckoning Failsafe
=======================

Copter includes the Dead Reckoning Failsafe that allows a vehicle to return home (or partially return home) if it loses GPS (or more accurately if it loses its position estimate which could be based on GPS, OpticalFlow, etc).

.. note::

    This failsafe is available in Copter-4.3 (and higher)

.. note::

    Similar functionality is available using the `copter-deadreckon-home.lua <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets/copter-deadreckon-home.lua>`__ script (`video1 <https://www.youtube.com/watch?v=KKShYheW4J0>`__, `video2 <https://www.youtube.com/watch?v=esM0EqMH_BE>`__)

Setup
=====

- Setup :ref:`wind speed estimation <airspeed-estimation>` including setting the :ref:`EK3_DRAG_BCOEF_X <EK3_DRAG_BCOEF_X>`, :ref:`EK3_DRAG_BCOEF_Y <EK3_DRAG_BCOEF_Y>` and :ref:`EK3_DRAG_MCOEF <EK3_DRAG_MCOEF>` parameters
- Set :ref:`FS_DR_ENABLE <FS_DR_ENABLE>` to "2" (RTL) or one of the other available options
- Set :ref:`FS_DR_TIMEOUT <FS_DR_TIMEOUT>` to the number of seconds that the vehicle can maintain control after losing GPS.  On most vehicles this is only 10 to 30 seconds

When will it trigger?
=====================

The deadreckoning failsafe will trigger under the same conditions as the :ref:`EKF failsafe <ekf-inav-failsafe>`.  Namely when any two of the EKF "variances" for compass, position or velocity are higher than the :ref:`FS_EKF_THRESH <FS_EKF_THRESH>` parameter for 1 second.  In practice it is expected that normally it will be the loss of GPS that will trigger the failsafe.

What will happen when the failsafe triggers?
============================================

- The vehicle will continue to fly normally for 7 to 10 seconds after the loss of GPS
- "Dead Reckoning started" will be displayed on the GCS and the vehicle will switch to :ref:`RTL <rtl-mode>` mode and begin flying home
- If the vehicle does not reach home within :ref:`FS_DR_TIMEOUT <FS_DR_TIMEOUT>` seconds, the :ref:`EKF failsafe <ekf-inav-failsafe>` will trigger and the vehicle will switch to :ref:`Land <land-mode>` mode
- Even if the GPS recovers while the vehicle is returning home, the vehicle will remain in RTL

Testing
=======

Follow the instructions below to test in the simulator or on a real vehicle.  If testing on a real vehicle be ready to re-take control in a manul mode (e.g. :ref:`Stabilize <stabilize-mode>` or :ref:`AltHold <altholdmode>`)

- Set :ref:`RC9_OPTION <RC9_OPTION>` to 65 to allow disabling the GPS from an :ref:`auxiliary switch <common-auxiliary-functions>` (any RC input channel may be used)
- Arm and fly the vehicle in any mode that requires GPS (e.g. :ref:`Loiter <loiter-mode>`, :ref:`Guided <ac2_guidedmode>`, :ref:`Auto <auto-mode>`, etc)
- Ensure the EKF has calculated a wind speed estimate by checking “wind_dir” and/or “wind_vel” in the ground station (see "Viewing Windspeed and Direction in Real-Time" on :ref:`this page <airspeed-estimation>`)
- Raise the auxiliary switch high to disable the GPS
- Within 7 to 10 seconds "Dead Reckoning started" should be displayed on the GCS and the vehicle should switch to :ref:`RTL <rtl-mode>` mode and begin flying home
- After :ref:`FS_DR_TIMEOUT <FS_DR_TIMEOUT>` seconds the :ref:`EKF failsafe <ekf-inav-failsafe>` will trigger and the vehicle will switch to :ref:`Land mode <land-mode>`
- If the vehicle's position drifts too much, switch to :ref:`AltHold <altholdmode>` mode to recover control of the vehicle
- Lower the auxiliary switch at any time to re-enable the GPS
- Post testing, lower or raise the :ref:`FS_DR_TIMEOUT <FS_DR_TIMEOUT>` parameter to reflect the maximum number of seconds the vehicle can maintain control without GPS

Videos
======

..  youtube:: G-vYP_IQZeM
    :width: 100%

..  youtube:: Xq-ecwgFKzA
    :width: 100%
