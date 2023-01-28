.. _yaw-in-plane:

============================
Yaw and Yaw Control in Plane
============================

The control of yaw in plane is mainly via the RUDDER output function (``SERVOx_FUNCTION`` = 21). Of course, flying wings do not have that function and yaw occurs naturally as the wings are banked and elevator applied to eliminate altitude loss. The fact the most flying wings are swept provides an amount of natural turn coordination that rudder normally provides for turns.

ArduPilot, in fixed wing modes (except ACRO mode, if yaw rate controller is enabled), provides NO direct rate stabilization or heading angle hold for yaw, as it does for pitch and roll. In manual fixed wing modes (MANUAL,ACRO, and TRAINING), the pilot will need to control the rudder, if present, in order to make coordinated turns (turns where there is no lateral acceleration, ie slip or skid).

However, in all other modes, ArduPilot can provide reasonable turn coordination via the :ref:`KFF_RDDRMIX<KFF_RDDRMIX>` parameter which will feed rudder in as ailerons move. See the section below, :ref:`coordinated-turn-setup`. In all other modes, besides MANUAL, ACRO, or TRAINING, no rudder input from the pilot is needed.

.. note:: rudder demand can be applied by pilot input in all modes, including AUTO, which is usually undesirable.

In addition, ArduPilot produced yaw control can also occur:

#.  If :ref:`Ground Steering<tuning-ground-steering-for-a-plane>` is enabled, and no GroundSteering output(``SERVOx_FUNCTION`` =26) is being used to control a nose or tail wheel, then the steering controller will command the rudder deflection while taxiing instead of there being direct control from the rudder stick. 
#.  If differential spoilers are used and the  :ref:`DSPOILR_RUD_RATE<DSPOILR_RUD_RATE>` parameter is non-zero. See :ref:`differential-spoilers`.
#.  If the YAW Damper or Side-Slip Controllers are enabled and active. (See :ref:`Yaw Damper<yaw-controller-tuning>`). These controllers do not directly take pilot input, but rather help stabilize yaw and control side-slip in turns. The Yaw Damper can be used to provide yaw stability in fin-less flying wings using split spoilers/elevons.
#.  In ACRO mode, if enabled, a yaw rate controller can be used to stabilize body frame yaw. See :ref:`acro-mode` page for details and tuning.

.. _differential-thrust-yaw:

Other YAW mechanisms
====================

If twin engines are used (THROTTLE LEFT and THROTTLE RIGHT output functions), then the pilot and ArduPilot can use differential thrust to produce yaw in Plane, or in QuadPlanes when in fixed wing modes, if using twin/quad forward motors for fixed wing flight modes. This is enabled by setting :ref:`RUDD_DT_GAIN<RUDD_DT_GAIN>` to a non-zero value.

Yaw Trim
========

The easiest way to adjust yaw trim is to monitor heading, either thru an FPV camera, or Ground Control Station, and adjust the yaw servo's ``SERVOx_TRIM`` to stop heading creep when flying with wings level.

.. _coordinated-turn-setup:

Setting up :ref:`KFF_RDDRMIX<KFF_RDDRMIX>` for Coordinated Turns
================================================================

Rapidly roll the model from maximum bank angle in one direction to maximum bank angle in the opposite direction in FBWA. Do this several times going in each direction and observe the yawing motion of the model. If as the wings pass through level the nose is yawed in the opposite direction to the roll (for example when rolling from left to right bank, the nose points left) then increase the value of :ref:`KFF_RDDRMIX<KFF_RDDRMIX>` gain until the yaw goes away. Do not use a value larger than 1. Conversely, lower it if the nose is yawing into the turn. The default value for :ref:`KFF_RDDRMIX<KFF_RDDRMIX>` is usually close