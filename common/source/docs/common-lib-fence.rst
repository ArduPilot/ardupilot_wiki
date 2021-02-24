.. _common-lib-fence:

=================
Fence
=================

Overview
========



Fence Types
===========

Simple
------

Tin Can
^^^^^^^

Maximum Altitude
^^^^^^^^^^^^^^^^

Minimum Altitude
^^^^^^^^^^^^^^^^

Advanced
--------

Inclusion Circle
^^^^^^^^^^^^^^^^

Inclusion Polygon
^^^^^^^^^^^^^^^^^

Exclusion Polygon
^^^^^^^^^^^^^^^^^

Exclusion Circle
^^^^^^^^^^^^^^^^

Fence Actions
=============

The :ref:`FENCE_ACTION<FENCE_ACTION>` parameter defines the action to take when the fence has been breached.

At the moment the fence is breached a backup fence is erected 20m
further out (or up).  If the vehicle breaches this backup fence (for
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
