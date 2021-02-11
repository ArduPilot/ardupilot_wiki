.. _soaring:

=======
Soaring
=======

.. image:: ../../../images/soar-cover.jpg


The autonomous soaring functionality in ArduPilot allows the plane to respond to 
rising air current (thermals) in order to extend endurance and gain altitude with 
minimal use of the motor (soaring). Its full technical description is available in

*S. Tabor, I. Guilliard, A. Kolobov.* `ArduSoar: an Open-Source Thermalling Controller for Resource-Constrained Autopilots <https://arxiv.org/abs/1802.08215/>`_. *International Conference on Intelligent Robots and Systems (IROS), 2018.*


.. image:: ../../../images/thermalling.jpg

This picture shows the different phases of flight when using the soaring
functionality:

#. If modes AUTO, FBWB or CRUISE are entered, and Soaring is enabled, the throttle is set to zero provided the aircraft is above :ref:`SOAR_ALT_MIN<SOAR_ALT_MIN>` altitude and the aircraft then begins gliding.
#. In AUTO, if the aircraft descends to :ref:`SOAR_ALT_MIN<SOAR_ALT_MIN>` altitude, throttle is re-enabled and the aircraft will begin to climb to the altitude of the next waypoint. If that waypoint altitude is less than :ref:`SOAR_ALT_CUTOFF<SOAR_ALT_CUTOFF>` altitude, then Soaring can not begin before reaching the waypoint. If it is above :ref:`SOAR_ALT_CUTOFF<SOAR_ALT_CUTOFF>` altitude, then Soaring can occur once that altitude has been reached. In FBWB or CRUISE modes, if  :ref:`SOAR_ALT_MIN<SOAR_ALT_MIN>` altitude is reached, an RTL will be initiated, so the pilot must disable Soaring, or change to a mode other than FBWB or CRUISE to climb back above :ref:`SOAR_ALT_CUTOFF<SOAR_ALT_CUTOFF>` to begin gliding again and prevent an RTL beginning.
#. When the aircraft reaches :ref:`SOAR_ALT_CUTOFF<SOAR_ALT_CUTOFF>` altitude, throttle is set to zero again.
#. If, during gliding flight, the air is estimated to be rising at more than
   :ref:`SOAR_VSPEED<SOAR_VSPEED>` and the :ref:`RC switch position<soaring_rc-switch>` allows it, the aircraft will automatically enter LOITER mode. While in LOITER mode the aircraft will adjust the loiter position to better centre the thermal.
#. LOITER mode is exited under the following conditions:

   - :ref:`SOAR_ALT_MAX<SOAR_ALT_MAX>` is reached.
   - :ref:`SOAR_ALT_MIN<SOAR_ALT_MIN>` is reached.
   - Flight mode is changed by the pilot.
   - The estimate of achievable climb rate falls below :ref:`SOAR_VSPEED<SOAR_VSPEED>` , and 
     thermalling has lasted at least :ref:`SOAR_MIN_THML_S<SOAR_MIN_THML_S>` seconds.
   - The aircraft drifts more than :ref:`SOAR_MAX_DRIFT<SOAR_MAX_DRIFT>` - see :ref:`Limit maximum distance from home<soaring_maximum-distance-from-home>`

   The flight mode will be returned to whatever it was before LOITER was 
   triggered, with the following exception: If the previous mode was FBWB or 
   CRUISE, and thermalling ended due to reaching :ref:`SOAR_ALT_MIN<SOAR_ALT_MIN>`, RTL will be
   triggered instead.


Non Supported Boards
====================

Generally all boards support soaring, *except* those with firmware limitations referred to on :ref:`this page <common-limited_firmware>`. As of June 2020,
non-supported boards include:

 - KakuteF7Mini
 - KakuteF7
 - sparky2
 - Pixhawk1-1M
 - OMNIBUSF7V2

Setting up soaring
==================

To use your plane for soaring, it should ideally be a glider type aircraft with 
a good lift to drag ratio and be equipped with an airspeed sensor. There are a 
few steps involved in setting a plane up for soaring:

#. Set up a suitable mission.
#. Tune the TECS.
#. Estimate aircraft drag.
#. Set up the soaring parameters.
#. Set loiter radius and bank angle limit.

Mission Setup
=============

The main requirement for a mission is that it take the aircraft above :ref:`SOAR_ALT_CUTOFF<SOAR_ALT_CUTOFF>`
so that gliding flight is initiated. To achieve this, set the waypoints' altitude(s)
above :ref:`SOAR_ALT_CUTOFF<SOAR_ALT_CUTOFF>` . 

Tune the TECS
=============
 
For best results the TECS needs to be set up to fly the aircraft at a consistent airspeed when 
gliding. To achieve this, :ref:`TECS_SPDWEIGHT<TECS_SPDWEIGHT>` to 2.0, :ref:`SOAR_ENABLE<SOAR_ENABLE>` to 1 and set
:ref:`SOAR_VSPEED<SOAR_VSPEED>` to a large number, say 50.0, or use the :ref:`RC switch<soaring_rc-switch>`
to inhibit mode changes. This means that the aircraft will
glide but will never begin thermalling. Set :ref:`SOAR_ALT_CUTOFF<SOAR_ALT_CUTOFF>` to an altitude you
feel comfortable with. It should be high enough to allow a good length of time to
be spent gliding. 
Launch the aircraft and put it in AUTO mode. It should climb to :ref:`SOAR_ALT_CUTOFF<SOAR_ALT_CUTOFF>` 
and then begin a gliding descent.
Watch the telemetry graphs or look at the Dataflash logs. Is the aircraft maintaining
the demanded airspeed? The actual and demanded airspeed can be seen in the onboard log as 
``TECS.sp`` and ``TECS.spdem``, and via telemetry you can use ``NAV_CONTROLLER_OUTPUT.aspd_error``. You will 
probably need to increase :ref:`PTCH2SRV_IMAX<PTCH2SRV_IMAX>` and :ref:`TECS_INTEG_GAIN<TECS_INTEG_GAIN>` to achieve good airspeed
tracking in gliding flight.

Estimate Aircraft Drag
======================

To work out how fast the air is rising or sinking the autopilot needs to know the
aircraft's sink rate for a given airspeed in still air. This is related to the 
drag polar of the plane.
Estimating the polar can be a little involved. If you have an airframe reasonably
similar to a Parkzone Radian, it is reasonable to leave the :ref:`SOAR_POLAR_B<SOAR_POLAR_B>` and
:ref:`SOAR_POLAR_CD0<SOAR_POLAR_CD0>` unchanged. You should adjust :ref:`SOAR_POLAR_K<SOAR_POLAR_K>` for your plane using the
following formula:

:ref:`SOAR_POLAR_K<SOAR_POLAR_K>` = 16*Weight/Area
(weight in kg, area in metres squared).

:ref:`SOAR_POLAR_K<SOAR_POLAR_K>` = 703*Weight/Area
(weight in oz, area in inches squared).


Set up the Soaring Parameters
=============================

Change the :ref:`SOAR_VSPEED<SOAR_VSPEED>` parameter back to a sensible value. Remember, 
this parameter controls when the mode will be changed to LOITER and thermalling 
starts. Change :ref:`SOAR_ALT_MAX<SOAR_ALT_MAX>` to the altitude you want the autopilot to stop 
thermalling.

Set loiter radius and bank angle limit
======================================

The parameter :ref:`WP_LOITER_RAD<WP_LOITER_RAD>` sets how tight the loiter circle is. For thermalling it is usually
best to have the aircraft fly at a 30 - 45 degree bank angle. The corresponding loiter radius can be calculated as 
about airspeed squared over ~10 (for 45 degrees) or ~6 (for 30 degrees), from the equation

.. raw:: html

   <a href="https://www.codecogs.com/eqnedit.php?latex=r&space;=&space;\frac{v^2}{g&space;\tan&space;\phi}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?r&space;=&space;\frac{v^2}{g&space;\tan&space;\phi}" title="r = \frac{v^2}{g \tan \phi}" /></a>

The tanget is for the desired bank angle, the result will be in meters for the radius,with g  = 9.81 m/s/s, velocity (v) is in m/s. For example, if the airspeed in loiter is 20m/s, then the :ref:`WP_LOITER_RAD<WP_LOITER_RAD>` should be 40 for a 45 degree bank, assuming that :ref:`LIM_ROLL_CD<LIM_ROLL_CD>` is set at 4500 or higher.

You should make sure that the limiting bank angle :ref:`LIM_ROLL_CD<LIM_ROLL_CD>` is set a bit larger to give some room for manoeuvring.

.. _soaring_rc-switch:

Set up RC switch (Optional)
===========================

You can use a 2-position RC switch to control when the autopilot can use soaring. Set the parameter SOAR_ENABLE_CH to the corresponding channel number. The 3 positions have the following effect.

 - Below 1700us. Soaring is disabled (equivalent to setting SOAR_ENABLE = 0). Throttle will be used as normal. Switching to this from either of the positions below, will disable Soaring and maintain the current flight mode.
 
 - Above 1700us. Fully automatic mode changes to LOITER from AUTO, FBWB or CRUISE modes in response to detected rising air, and following of rising air currents.

+----------------+---------------+-------------------+------------------+-------------------+
| PWM Value      | Auto throttle |  Tracking thermal | Automatic change | Automatic change  | 
|                | cutoff        |  updrafts         | back from LOITER | to LOITER         |
+----------------+---------------+-------------------+------------------+-------------------+
| < 1700 us      |       N       |       N           |       N          |       N           | 
+----------------+---------------+-------------------+------------------+-------------------+
| > 1700 us      |       Y       |       Y           |       Y          |       Y           |
+----------------+---------------+-------------------+------------------+-------------------+


Set limits
===========

Because the soaring feature can follow rising air as required to gain altitude, it is important to set limits to avoid it leaving the original flight area completetly. This is especially important in windy conditions as the autopilot will try to follow thermals downwind. There are three ways to set limits.


Time limits
-----------

:ref:`SOAR_MIN_THML_S<SOAR_MIN_THML_S>` : Minimum time to remain in LOITER once entered for a thermal before exiting due to low lift or altitude limits.

:ref:`SOAR_MIN_CRSE_S<SOAR_MIN_CRSE_S>` : Minimum time to remain in glide after exiting LOITER due to low lift or altitude limits before entering LOITER mode again, or when entering Soaring initially.


Spatial limits
--------------

:ref:`Geofence <geofencing>` can be used to constrain the physical flight area used. Set it up in the usual way.


Use of TECS synthetic airspeed
==============================

If your plane can't accommodate an airspeed sensor, it is possible to use the TECS synthetic airspeed estimate :ref:`TECS_SYNAIRSPEED<TECS_SYNAIRSPEED>`.
Make sure you read the warning regarding this feature before deciding to use it. To use this feature, set the parameter :ref:`TECS_SYNAIRSPEED<TECS_SYNAIRSPEED>` to 1.


MAVLINK Telemetry
=================

Currently, the only effect on telemetry is that when soaring is active the climb rate item (VFR_HUD.climb) is altered. Rather that the estimated vertical speed of the aircraft, the estimated vertical speed of the air mass is sent. This field is used by Mission Planner and OpenTX radios to produce vario audio output.

