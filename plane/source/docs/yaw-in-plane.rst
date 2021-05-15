.. _yaw-in-plane:

============================
Yaw and Yaw Control in Plane
============================

The control of yaw in plane is mainly via the RUDDER output function (``SERVOx_FUNCTION`` = 21). Of course, flying wings do not have that function and yaw occurs naturally as the wings are banked and elevator applied to eliminate altitude loss. The fact the most flying wings are swept provides an amount of natural turn coordination that rudder normally provides for turns.

Ardupilot, in fixed wing modes, provides NO direct rate stabilization or heading angle hold for yaw, as it does for pitch and roll. Rudder control surface movement is primarily a direct manual pilot control in all modes.

There are a few exceptions:

#.  If :ref:`KFF_RDDRMIX<KFF_RDDRMIX>` is non-zero, then rudder will be added as the ailerons move to provide improved turn coordination.
#.  If :ref:`Ground Steering<tuning-ground-steering-for-a-plane>` is enabled, and no GroundSteering output(``SERVOx_FUNCTION`` =26) is being used to control a nose or tail wheel, then the steering controller will command the rudder deflection while taxiing instead of there being direct control from the rudder stick. 
#.  If differential spoilers are used and the  :ref:`DSPOILR_RUD_RATE<DSPOILR_RUD_RATE>` parameter is non-zero. See :ref:`differential-spoilers`.
#.  If the YAW Damper or Side-Slip Controllers are enabled and active. (See :ref:`Yaw Damper<yaw-controller-tuning>`). These controllers do not directly take pilot input, but rather help stabilize yaw and control side-slip in turns. The Yaw Damper can be used to provide yaw stability in fin-less flying wings using split spoilers/elevons.

Other YAW mechanisms
====================

If twin engines are used (THROTTLE LEFT and THROTTLE RIGHT output functions), then the pilot and ArudPilot can use differential thrust to produce yaw in Plane, or in Quadplanes when in fixed wing modes, if using twin/quad forward motors for fixed wing flight modes. This is enabled by setting :ref:`RUDD_DT_GAIN<RUDD_DT_GAIN>` to a non-zero value.