.. _common-lib-fence:

=================
Fence
=================

Overview
========

Rover and Copter will attempt to stop your vehicle from flying/driving too far away
by stopping at the fence. Plane will execute a configurable :ref:`FENCE_ACTION<FENCE_ACTION>`
after breaching this fence.

Fence Types
===========

Simple
------

Simple Circular (Tin Can)
^^^^^^^^^^^^^^^^^^^^^^^^^

The simple "tin can" fence is a circular fence centered on home, with a radius defined by
:ref:`FENCE_RADIUS<FENCE_RADIUS>` .

Maximum Altitude
^^^^^^^^^^^^^^^^

The `Maximum Altitude` fence defines the maximum altitude above home that the vehicle can
operate at before executing the configurable :ref:`FENCE_ACTION<FENCE_ACTION>` .

Minimum Altitude
^^^^^^^^^^^^^^^^

The `Minimum Altitude` fence defines the minimum altitude above home that the vehicle can
operate at before executing the configurable :ref:`FENCE_ACTION<FENCE_ACTION>` .

Polygon Fences
--------------

Copter, Rover and Plane include support for polygon fences with up to 70 points and circular fences.
Either may be selected to be inclusion or exclusion type.

The purpose of these fences are to attempt to stop your vehicle from flying into (exclusion), or out of (inclusion),
the fences by initiating a failsafe action like RTL or, if flying in Loiter mode, the vehicle will normally stop before breaching the fence

These fences are an extension of the simpler Tin Can fence, and can be combined with it.

These fences are created and treated in a similar way to mission command lists and rally point lists and loaded into the autopilot.


..  youtube:: U3Z8bO3KbyM
    :width: 100%

Fence Actions
=============

The :ref:`FENCE_ACTION<FENCE_ACTION>` parameter defines the action to take when the fence has been breached.

At the moment the fence is breached a backup fence is erected 20m
further out (or up). If the vehicle breaches this backup fence (for
example if the the vehicle is not set up correctly or the operator takes
control but is unable to bring the vehicle back towards home), the vehicle
will execute the :ref:`FENCE_ACTION<FENCE_ACTION>` again (and another backup fence an additional
20m further out will be created if this is breached again).

Copter/Rover
------------

If the vehicle eventually proceeds 100m outside the configured fence
distance, despite the backup fences, the vehicle will switch into LAND mode (HOLD for Rover).  The idea being that it's clearly impossible to get the vehicle home so best to just bring it
down/stop it.  The pilot can still retake control of course with the flight mode
switches.  Like with the earlier fences, another fence is erected 20m
out which will again switch the vehicle to LAND (HOLD for Rover), if it continues away from
home.

Plane
-----

If the vehicle proceeds outside the configured fence distance, the vehicle will reinitiate the Fence Action.
The pilot can retake control of course with the flight mode switches.  Like with the earlier fences, another
fence is erected 20m out which will again change mode according to the fence action, if it continues away from
home.

Fence Enable
============


Fence Auto-Enable
=================

Enabling the fence with an RC Channel Auxiliary Switch
======================================================

It is not necessary to set-up a switch to enable or disable the fence
but if you wish to control the fence with a switch follow these
steps:

An ``RCx_OPTION`` can be set via the Config/Tuning > Full Parameter List screen:

-  Use an ``RCx_OPTION`` set to Fence
-  holding the switch high (i.e. PWM > 1800) will enable the fence, low
   (under 1800) will disable the fence.

Notes:
======

.. note:: You can define many inclusion and exclusion fences. However,multiple inclusions fences,
   including the simple circular fence must overlap, since the vehicle can operate only within the
   complete overlap area of all of the inclusion fences. Exclusion fences may be placed within or
   outside of inclusion fences.

.. note:: In order to upload or download these fences from Mission Planner the connected link must
   be using MAVLink2 protocol. Normally, since the USB conenction is used, this protocol is default.
   However, radio linked connections may use MAVLink1 by default and would need to be changed to MAVLink2
   in order to upload and download across them.

.. tip:: You can have both the simple circular fence and inclusion/exclusion fences and choose to use
   just the HOME centerd "tin-can" for a flight by selecting only the "Circle" or "Altitude and Circle"
   for :ref:`FENCE_TYPE<FENCE_TYPE>`. You can chose to enable the simple circular fence, these
   inclusion/exclusion fences, and/or altitude limit, in any combination, with this parameter.


Warnings:
=========
-  The minimum recommended fence radius is 30m
-  The fence requires the GPS to be functioning well so do not disable
   the :ref:`GPS arming check <common-prearm-safety-checks>` nor the :ref:`EKF failsafe <ekf-inav-failsafe>` while the fence is enabled. 
   Conversely if you disable either of these checks, disable the Fence.
-  For the best results, ensure RTL is working on your vehicle.
-  With the Fence enabled, the pre-arm checks will require you have GPS
   lock before arming the vehicle.
-  If GPS failsafe is not enabled and the Fence is enabled and you lose
   GPS lock while flying the fence will be disabled.
-  If GPS failsafe is enabled and the Fence is enabled and you lose GPS
   lock while piloting, the vehicle will switch to LAND (HOLD for Rover) because we no
   longer know the vehicle position and we want to ensure the vehicle
   never travels far outside the fence.  This behavior will occur
   regardless of the flight mode.  If this is not desired,
   the pilot can retake control by moving the flight mode switch.
-  The backup fences are created 20m out from the previous breached
   fence not 20m out from the vehicle's position.  This means if you
   choose to override the fence you may have less than 20m to regain
   vehicle control before the fence switches the vehicle to the :ref:`FENCE_ACTION<FENCE_ACTION>`
   again.  If you really want to override the fence, you should be ready
   to switch the flight mode twice or alternatively set-up the
   enable/disable fence switch.

Video overview of the Fence setup and Operation
===============================================

..  youtube:: HDnGdo54o-4
    :width: 100%


Combining with the Cylindrical Fence
====================================

A polygon fence can be used in combination with the :ref:`cylindrical fences <common-ac2_simple_geofence>` and the failsafe behaviour (i.e. stop at the fence or RTL) will trigger at whichever barrier the vehicle reaches first (i.e. the green line shown below)

.. note::
   .. image:: ../../../images/copter_polygon_circular_fence..png
      :target: ../_images/copter_polygon_circular_fence..png