.. _soaring:

=======
Soaring
=======

The Autonomous Soaring functionality in ardupilot allows the plane to respond to 
rising air current (thermals) in order to extend endurance and gain altitude with 
minimal use of the motor (soaring).

.. image:: ../../../images/thermalling.jpg

This picture shows the different phases of flight when using the soaring
functionality:

#. When modes AUTO, FBWB or CRUISE are entered, throttle is set to zero provided
   the aircraft is above SOAR_ALT_MIN altitude. The aircraft begins gliding.
#. If the aircraft reaches SOAR_ALT_MIN altitude, throttle is re-enabled and the
   aircraft will begin to climb to the altitude of the next waypoint.
#. When the aircraft reaches SOAR_ALT_CUTOFF altitude, throttle is set to zero
   again.
#. If, during gliding flight, the air is estimated to be rising at more than
   SOAR_VSPEED the aircraft will enter LOITER mode. It will adjust the
   loiter position to better centre the thermal.
#. LOITER mode is exited under the following conditions:

   - SOAR_ALT_MAX is reached.
   - SOAR_ALT_MIN is reached.
   - Flight mode is manually changed.
   - The estimate of achievable climb rate falls below SOAR_VSPEED, and 
     thermalling has lasted at least SOAR_MIN_THML_S seconds.

   The flight mode will be returned to whatever it was before LOITER was 
   triggered, with the following exception. If the previous mode was FBWB or 
   CRUISE, and thermalling ended due to reaching SOAR_ALT_MIN, RTL will be
   triggered instead.
   
Technical details are available `here <https://arxiv.org/abs/1802.08215/>`_.


Setting up soaring
==================

To use your plane for soaring, it should ideally be a glider type aircraft with 
a good lift to drag ratio and be equipped with an airspeed sensor. There are a 
few steps involved in setting a plane up for soaring:

#. Set up a suitable mission.
#. Tune the TECS.
#. Estimate aircraft drag.
#. Set up the soaring parameters.

Mission Setup
=============

The main requirement for a mission is that it take the aircraft above SOAR_ALT_CUTOFF
so that gliding flight is initiated. To achieve this, set the waypoint altitude 
above SOAR_ALT_CUTOFF. 

.. warning::
 
   The autopilot can move the loiter waypoint quite a long way as it tracks a 
   thermal, particularly if there is wind. You should set up a 
   :ref:`geofence <geofencing>`
   to keep it within a safe distance.


Tune the TECS
=============

The TECS needs to be set up to fly the aircraft at a consistent airspeed when 
gliding. To achieve this, set TECS_SPDWEIGHT to 2.0, set SOAR_ENABLE to 1 and set
SOAR_VSPEED to a large number, say 50.0. This means that the aircraft will 
glide but will never begin thermalling. Set SOAR_ALT_CUTOFF to an altitude you
feel comfortable with. It should be high enough to allow a good length of time to
be spent gliding. 
Launch the aircraft and put it in AUTO mode. It should climb to SOAR_ALT_CUTOFF 
and then begin a gliding descent.
Watch the telemetry graphs or look at the Dataflash logs. Is the aircraft maintaining
the demanded airspeed? The demanded airspeed can be seen in the Dataflash log as 
TECS.spdem, and via telemetry you can use NAV_CONTROLLER_OUTPUT.aspd_error. You will 
probably need to increase PTCH2SRV_IMAX and TECS_INTEG_GAIN to achieve good airspeed
tracking in gliding flight.

Estimate Aircraft Drag
======================

To work out how fast the air is rising or sinking the autopilot needs to know the
aircraft's sink rate for a given airspeed in still air. This is related to the 
drag polar of the plane.
Estimating the polar can be a little involved. If you have an airframe reasonably
similar to a Parkzone Radian, it is reasonable to leave the SOAR_POLAR_B and
SOAR_POLAR_CDO unchanged. You should adjust SOAR_POLAR_K for your plane using the
following formula:

SOAR_POLAR_K = 16*Weight/Area
(weight in kg, area in metres squared).

Set up the Soaring Parameters
=============================

Change the SOAR_VSPEED parameter back to a sensible value. Remember, 
this parameter controls when the mode will be changed to LOITER and thermalling 
starts. Change SOAR_ALT_MAX to the altitude you want the autopilot to stop 
thermalling.

.. warning::
 
   Although thermalling will stop at SOAR_ALT_MAX, it is possible that for strong
   thermals to take the plane higher than this before it exits the thermal.






