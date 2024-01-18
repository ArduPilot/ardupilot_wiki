.. _common-ac2_simple_geofence:


=================
Cylindrical Fence
=================

Overview
========

ArduPilot includes a simple "tin can" shaped fence centered on home that will attempt to stop your copter/rover from flying/driving too far away by stopping at the fence in some Copter/Rover modes if :ref:`common-object-avoidance-landing-page` is setup.

The maximum circular distance and altitude (altitude used by Plane and Copter only) and the vehicle behavior when the fence is reached can be configured using a ground station like Mission Planner or QGC.

See :ref:`common-geofencing-landing-page` for fence setup parameters common to all fences. Setup specific to the Cylindrical fence is below.

Enabling the Cylindrical Fence in Mission Planner
=================================================

To enable Fences, go to the Mission Planner full parameter list (CONFIG->Full Parameter Tree), search for items with ``FENCE_``:

.. image:: ../../../images/fence_enable.png
    :target: ../_images/fence_enable.png

Parameter associated with Cylindrical Fence
===========================================
-  Set :ref:`FENCE_TYPE<FENCE_TYPE>` bit 1 to "1"(+ 2 to value)
-  Set :ref:`FENCE_RADIUS<FENCE_RADIUS>` to the maximum distance from HOME you want (in meters). This should normally be at least 50m. This value must be larger than :ref:`FENCE_MARGIN<FENCE_MARGIN>` and greater than 30m.

.. note:: Polygon :ref:`FENCE_TYPE<FENCE_TYPE>` fences includes circular fences, as well as polygonal shapes, as specified in the Inclusion/Exclusion fence list. This simple home centered "TINCAN" fence is a separate fence. Rover ignores altitudes, if set.

..  youtube:: gP5LYPEOLZY
    :width: 100%

Copter GeoFence Tab
-------------------

Copter and Traditional Heli , when connected to Mission Planner, present a CONFIG tab called GeoFence which provides another way to access these parameters in those vehicles.

.. image:: ../../../images/Fence_MPSetup.png
    :target: ../_images/Fence_MPSetup.png

Warnings:
=========

-  The minimum recommended cylindrical fence radius is 30m
-  The fence requires the GPS to be functioning well so do not disable the :ref:`GPS arming check <common-prearm-safety-checks>` nor the :ref:`EKF failsafe <ekf-inav-failsafe>` while the fence is enabled. Conversely, if you disable either of these checks, disable the Fence.
-  For the best results, ensure RTL is working on your vehicle.
-  With the Fence enabled in Copter and Rover, the pre-arm checks will require you have GPS
   lock before arming the vehicle.
-  In Copter, if EKF failsafe occurs and the Fence is enabled and you lose
   GPS lock while flying the fence will be disabled.
-  In Copter, if EKF failsafe occurs and the Fence is enabled and in an autonomous mode, the vehicle will switch to LAND (HOLD for Rover) because we no
   longer know the vehicle position and we want to ensure the vehicle
   never travels far outside the fence. If this is not desired,
   the pilot can retake control by moving the flight mode switch to a manual mode.
-  The Copter and Rover backup fences are created 20m out from the previous breached
   fence not 20m out from the vehicle's position.  This means if you
   choose to override the fence you may have less than 20m to regain
   vehicle control before the fence switches the vehicle to the :ref:`FENCE_ACTION<FENCE_ACTION>`
   again.  If you really want to override the fence, you should be ready
   to switch the flight mode twice or alternatively set-up the
   enable/disable fence switch.

Video overview of the Fence Setup and Operation
===============================================

..  youtube:: HDnGdo54o-4
    :width: 100%
