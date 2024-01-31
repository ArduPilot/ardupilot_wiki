.. _precision-autolanding:

=====================
Precision AutoLanding
=====================

In the case where the landing touch down point must be tightly controlled (ie narrow and/or short landing area), the precision of the flare height and approach speed (and therefore flare duration) must greater. This requires either an accurate GPS (ie, :ref:`common-rtk-correction`), or ground facing rangefinder or LIDAR, to obtain more precise altitude readings than a barometer usually can provide due to impacts of air pressure changes in the fuselage due to speed and angle of attack. Using an airspeed sensor is also recommended to control approach speeds. All the same basic considerations, previously discussed in the Basic Autolanding section also apply.

Precision Altitude Source
=========================

A normal GPS will only provide vertical accuracy to within a couple of meters. However, if an RTK corrected GPS is used, tens of centimeter accuracy is possible. However, this requires a base station or laptop providing correction data. See :ref:`common-rtk-correction` for details.

To use the GPS as the altitude source instead of BARO set:

:ref:`EK3_SRC1_POSZ<EK3_SRC1_POSZ>` = 3 to use GPS instead of barometer for altitude

Or a downward facing Lidar/Rangefinder can be used. See :ref:`rangefinder-autolanding` page.

Approach Airspeed
=================

Automatic landing is greatly assisted by the use of an airspeed sensor.
When using an airspeed sensor the landing approach speed (the speed
coming down the glide slope) is controlled by the
:ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>`
parameter, in meters/second.

You need to choose a value for :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>` that is above the
stall speed of your aircraft, but low enough that the aircraft is able
to lose altitude and land in a reasonable distance. Note that as the
stall speed is dependent on the weight of your aircraft you will need to
adjust the landing speed if you change the aircraft's weight
significantly (such as by adding batteries or a camera).

The :ref:`LAND_WIND_COMP<LAND_WIND_COMP>` parameter controls how much headwind compensation is used when landing. Headwind speed component multiplied by this parameter is added to :ref:`TECS_LAND_ARSPD<TECS_LAND_ARSPD>` value. Set to 0 to disable this. 

.. note:: The target landing airspeed value is still limited to being lower than :ref:`AIRSPEED_MAX<AIRSPEED_MAX>`.

To further improve landing you can use a Pre-Flare to reduce airspeed
just before the flare. This is enabled by setting either
:ref:`LAND_PF_ALT <LAND_PF_ALT>` or :ref:`LAND_PF_SEC <LAND_PF_SEC>`
to either enter a pre-flare state at a fixed altitude or at an estimated
seconds to ground (given your current decent rate). Once the Pre-Flare
is triggered the desired airspeed becomes :ref:`LAND_PF_ARSPD <LAND_PF_ARSPD>`.
This value should be lower than :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>` but greater than the
stall speed. This is particularly useful where reverse thrust is
available. However, some aircraft can handle a stall landing so setting
this to a very low number (1) will tell the aircraft to bleed off as
much airspeed as possible before the flare.

Controlling the approach
------------------------

During the landing approach the autopilot needs to balance the requested
airspeed (set by :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>`) and the requested glide slope and
landing position (set by the final approach waypoint and final landing point).
The default configuration tries to balance these two demands equally,
but for some aircraft you may want to prioritize one over the other.

The priority of airspeed control versus height control is set using the
:ref:`TECS_LAND_SPDWGT <TECS_LAND_SPDWGT>`
parameter. A value of 1 (the default) means a balance between the two. A
value closer to two gives a higher priority to airspeed and a value
closer to zero gives a higher priority to height control. For example,
if you are landing at a speed close to the stall speed you may wish to
place a high priority on the airspeed control. To do that you should set
:ref:`TECS_LAND_SPDWGT <TECS_LAND_SPDWGT>` to a value close to 2, such as 1.9.

If what you want in a landing is precision in the position where it
lands then you should set :ref:`TECS_LAND_SPDWGT <TECS_LAND_SPDWGT>` to a low number, such as
0.2 or even 0.0. In that case the plane will still try to achieve the
target landing airspeed by using the throttle, but it will not try to
control airspeed with pitch.

If you are landing a glider (or any aircraft without a motor) then you
should set :ref:`TECS_LAND_SPDWGT <TECS_LAND_SPDWGT>` to 2.0, so that airspeed is the priority
and pitch will be used to control airspeed.

In most cases a value of -1 gives the best result. This special value
will auto-adjust the value during the landing, scaling it from your
normal :ref:`TECS_SPDWEIGHT <TECS_SPDWEIGHT>`
value down to zero at the point of landing. So up in the sky during
approach you maintain good airspeed but by the time you land the
emphasis is on a more accurate landing.

Determining your max glide slope angle
--------------------------------------

For a steep landing approach, the limitation is how well you can maintain your desired airspeed. 
This is determined by your aircraft's ability to create reverse thrust (motor+prop thrust or airbrake drag ability),if used,  and its resistance to slowing down (aircraft mass). 
In many cases extreme steepness is unnecessary, but possible. 
With an over-sized motor and lightweight aircraft you can come in as steep as 60 degrees.

To determine your steepest approach angle, set :ref:`TECS_APPR_SMAX <TECS_APPR_SMAX>` very high as to not limit you (e.g. 99). 
Next, plan a mission with a steeper than normal approach (try 15 degrees and go up from there).
Watch your airspeed on the approach - the aircraft should be able to maintain :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>` without exceeding 75% of the available reverse throttle range. 
If not, you're coming in too steep for the negative-thrust-to-mass ratio of your aircraft.

.. tip::

   Keep in mind that whatever value you determine as your maximum may
   not be acceptable in all wind conditions. It is best to be a little
   conservative to maintain repeatability.


:ref:`automatic-flaps` can also be used for steeper approaches.

Setting up a Pre-Flare
======================

With a rangefinder or accurate GPS, and airspeed sensors installed, a pre-flare point can be set since we will have an accurate airspeed and altitude reading. 

This gives us a good idea of our momentum and stable "initial conditions" to the final flare. Set :ref:`LAND_PF_ALT<LAND_PF_ALT>` (and/or :ref:`LAND_PF_SEC<LAND_PF_SEC>`) to a fairly high point (for example 10m) and adjust from there. Next set :ref:`LAND_PF_ARSPD<LAND_PF_ARSPD>` to a value just above your stall speed.

When :ref:`LAND_PF_ALT<LAND_PF_ALT>` is reached the airspeed demand will instantly go from :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>` to :ref:`LAND_PF_ARSPD<LAND_PF_ARSPD>`.

This will cause it to slam on the brakes if reverse thrust is being used (see :ref:`reverse-thrust-autolanding`) so that the airspeed reduces to the desired airspeed quickly, otherwise plan for some time/distance for the airspeed to reduce.

The trick is to set :ref:`LAND_PF_ALT<LAND_PF_ALT>` to an altitude where it
achieves :ref:`LAND_PF_ARSPD<LAND_PF_ARSPD>` before killing the throttle at
:ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>` (which is set at a lower altitude - around 1
or 2m).

Example, :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>` = 15, :ref:`LAND_PF_ARSPD<LAND_PF_ARSPD>` = 12, :ref:`LAND_PF_ALT<LAND_PF_ALT>` = 12, :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>` = 2. Depending on your slope, mass of aircraft and motor+propeller thrust ability, you're expecting the aircraft to decelerate from 15m/s to 12m/s airspeed while dropping 10m to 2m. These are the critical params to adjust to ensure a smooth and slow flare below 2m altitude.

Flare
-----

Now that you are starting the flare with a stable and predictable airspeed, it's much easier to :ref:`control the flare <automatic-landing_controlling_the_flare>`. 
If you've already tuned your flare for an auto-land without reverse thrust you'll want to retune it. 
You'll notice you're coming in much slower ad tuning will be easier. 
The tweaks and compromises you had to do before are much easier to deal with.


Determining actual stall speed of your aircraft
+++++++++++++++++++++++++++++++++++++++++++++++

Unless you really know what you're doing, stall speed can be hard to estimate. 
Traditionally, to determine this true value you would need to slowly decrease your airspeed until you stall but that comes with the pesky problem that now you have a stalled aircraft falling out of the sky.

With :ref:`LAND_PF_ALT<LAND_PF_ALT>` and :ref:`LAND_PF_ARSPD<LAND_PF_ARSPD>` you can check your stall speed much lower to the ground. 
To know the airspeed at the exact moment it stalls, check your dataflash logs (``*.bin`` on SD card) for the airspeed (ARSP.Airspeed) when your wing loses lift and drops by comparing actual roll (CTUN.Roll) and desired roll (CTUN.NavPitch) diverge.
