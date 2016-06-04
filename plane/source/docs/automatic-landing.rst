.. _automatic-landing:

=================
Automatic Landing
=================

This article explains how to land Plane as part of a mission plan and
includes information about how a landing can be safely aborted.

Configuring for Automatic Landing
=================================

Plane can automatically land an aircraft, as part of a mission plan.

To land the plane you need to add a
:ref:`NAV_LAND <mav_cmd_nav_land>`
command to the end of your mission indicating the latitude, longitude
and altitude of your desired touchdown point. In most cases, the
altitude should be set to 0. During landing, the autopilot will shut
down the throttle and hold the current heading when the plane reaches
the flare point, controlled by the parameters described below.

.. _automatic-landing_key_parameters:

Key Parameters
--------------

The key parameters that control automatic landing are:

-  :ref:`LAND_FLARE_ALT <LAND_FLARE_ALT>`
-  :ref:`LAND_FLARE_SEC <LAND_FLARE_SEC>`
-  :ref:`LAND_PITCH_CD <LAND_PITCH_CD>`
-  :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>`
-  :ref:`TECS_LAND_SPDWGT <TECS_LAND_SPDWGT>`

The meaning and recommended value of each of these parameters is
described below.

Setting the Flare Point
-----------------------

The "flare" is the final stage of the landing when the autopilot cuts
the throttle and raises the pitch, increasing drag and slowing the
aircraft to sink onto the ground. The appropriate time to flare depends
on the type of aircraft, and is controlled by the
:ref:`LAND_FLARE_ALT <LAND_FLARE_ALT>` and :ref:`LAND_FLARE_SEC <LAND_FLARE_SEC>`
parameters.

The primary control of the flare is the ``LAND_FLARE_SEC`` parameter.
This is the time in seconds before the aircraft would hit the ground if
it continued with its current descent rate. So if the plane is
descending at 2 meters/second and you set the ``LAND_FLARE_SEC`` to 3
then the aircraft would flare at an altitude of 6 meters above the
ground. By using a time to impact to control the flare the aircraft is
able to flare at a higher altitude if it is descending quickly, and at a
lower altitude if it is descending slowly. That helps ensure the flare
is able to produce a smooth touchdown.

The second control is ``LAND_FLARE_ALT``. That is an altitude above the
ground in meters at which the aircraft will flare, regardless of its
descent rate.

The appropriate values for these two parameters depends on how the
autopilot is estimating its altitude above the ground. If the autopilot
has a good rangefinder (:ref:`such as LIDAR <common-rangefinder-lidarlite>`) then you can safely choose
quite small numbers, and flare close to the ground. That will generally
produce a better landing. A value for ``LAND_FLARE_SEC`` of 1.5 and
``LAND_FLARE_ALT`` of 2 is a good place to start with a LiDAR. If you
are relying solely on a barometer for landing altitude then you will
probably need higher values, to account for barometric error.

Controlling the glide slope
---------------------------

Another important factor in setting up the flare point is the glide
slope. The glide slope is the ratio of the distance from the last
waypoint to the landing point, and the height difference between the
last waypoint and the landing point. For example, if the landing point
is 300 meters from the last waypoint, and the last waypoint is 30 meters
above the ground then the glide slope is 10%.

If the glide slope is too steep then the aircraft will not be able to
flare in time to avoid crashing, plus the autopilot may not be able to
keep the plane on the approach slope accurately. It is recommended that
you start with a glide slope of at most 10%. What glide slope your plane
can handle will depend on how well your pitch controller tuning is, how
good your TECS tuning is, and the landing speed you ask for.

If you find your aircraft is not following the desired glide slope
accurately then you should first check your pitch tuning in your logs,
and ensure that the demanded and achieved pitch match within a couple of
degrees during landing. If they don't then look at the documentation on
pitch tuning (or possibly re-run AUTOTUNE). If the demanded and achieved
pitch do match then you should check your TECS logs to ensure that the
demanded and achieved airspeed are matching during landing. Have a look
at the TECS tuning patch for more information.

You should also be aware that many model aircraft can glide for long
distances, and it may be that your requested glide slope and airspeed
combination just isn't achievable.

Landing Airspeed
----------------

Automatic landing is greatly assisted by the use of an airspeed sensor.
When using an airspeed sensor the landing approach speed (the speed
coming down the glide slope) is controlled by the
:ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>`
parameter, in meters/second.

You need to choose a value for ``TECS_LAND_ARSPD`` that is above the
stall speed of your aircraft, but low enough that the aircraft is able
to lose altitude and land in a reasonable distance. Note that as the
stall speed is dependent on the weight of your aircraft you will need to
adjust the landing speed if you change the aircraft's weight
significantly (such as by adding batteries or a camera).

To further improve landing you can use a Pre-Flare to reduce airspeed
just before the flare. This is enabled by setting either
:ref:`LAND_PF_ALT <LAND_PF_ALT>` or :ref:`LAND_PF_SEC <LAND_PF_SEC>`
to either enter a pre-flare state at a fixed altitude or at an estimated
seconds to ground (given your current decent rate). Once the Pre-Flare
is triggered the desired airspeed becomes :ref:`LAND_PF_ARSPD <LAND_PF_ARSPD>`.
This value should be lower than ``TECS_LAND_ARSPD`` but greater than the
stall speed. This is particularly useful where reverse thrust is
available. However, some aircraft can handle a stall landing so setting
this to a very low number (1) will tell the aircraft to bleed off as
much airspeed as possible before the flare.

Controlling the approach
------------------------

During the landing approach the autopilot needs to balance the requested
airspeed (set by ``TECS_LAND_ARSPD``) and the requested glide slope and
landing position (set by the previous waypoint and final landing point).
The default configuration tries to balance these two demands equally,
but for some aircraft you may want to prioritize one over the other.

The priority of airspeed control versus height control is set using the
:ref:`TECS_LAND_SPDWGT <TECS_LAND_SPDWGT>`
parameter. A value of 1 (the default) means a balance between the two. A
value closer to two gives a higher priority to airspeed and a value
closer to zero gives a higher priority to height control. For example,
if you are landing at a speed close to the stall speed you may wish to
place a high priority on the airspeed control. To do that you should set
``TECS_LAND_SPDWGT`` to a value close to 2, such as 1.9.

If what you want in a landing is precision in the position where it
lands then you should set ``TECS_LAND_SPDWGT`` to a low number, such as
0.2 or even 0.0. In that case the plane will still try to achieve the
target landing airspeed by using the throttle, but it will not try to
control airspeed with pitch.

If you are landing a glider (or any aircraft without a motor) then you
should set ``TECS_LAND_SPDWGT`` to 2.0, so that airspeed is the priority
and pitch will be used to control airspeed.

In most cases a value of -1 gives the best result. This special value
will auto-adjust the value during the landing, scaling it from your
normal :ref:`TECS_SPDWEIGHT <TECS_SPDWEIGHT>`
value down to zero at the point of landing. So up in the sky during
approach you maintain good airspeed but by the time you land the
emphasis is on a more accurate landing.

.. _automatic-landing_controlling_the_flare:

Controlling the flare
---------------------

The final stage of the landing is called the "flare". During the flare
the aircraft tries to retain a course along the line between the last
waypoint and the landing waypoint, and it controls it's height solely
using a target descent rate. Once the flare is started the throttle is
"disabled" - set to some value between :ref:`THR_MIN <THR_MIN>` and
zero.

The main job of the flight controller in the flare is to try to achieve
the descent rate specified in the
:ref:`TECS_LAND_SINK <TECS_LAND_SINK>` parameter. That defaults to 0.25 meters/second, which is a reasonable
touchdown vertical speed for most models. To achieve that speed the TECS
controller uses pitch control only as the motor has been forced to zero.

The primary parameters which affect the ability of the aircraft to
achieve the desired descent rate are
:ref:`LAND_PITCH_CD <LAND_PITCH_CD>`, 
:ref:`TECS_LAND_DAMP <TECS_LAND_DAMP>`
and the main pitch tuning parameters.

The ``LAND_PITCH_CD`` parameter sets the minimum pitch target in the
flare (in centi-degrees). This parameter is very airframe specific and
is designed to prevent the nose of the aircraft being too far down on
touchdown causing issues with damaging the landing gear or breaking a
propeller.  For most aircraft this should be a small positive number
(such as 300, meaning 3 degrees), but for some belly landing aircraft a
small negative number can be good, to allow the nose to be kept down a
small amount to reduce the chance of stall if the flare happens too far
off the ground.

Note that the actual pitch of the aircraft can be quite a bit above
``LAND_PITCH_CD`` as the TECS controller tries to control the descent
rate. The maximum pitch is controlled by the
:ref:`TECS_PITCH_MAX <TECS_PITCH_MAX>`
parameter if it is non-zero, otherwise by the
:ref:`LIM_PITCH_MAX <LIM_PITCH_MAX>` parameter.

The ``TECS_LAND_DAMP`` parameter is a damping constant for the pitch
control during. A larger number will cause the pitch demand to change
more slowly. This parameter can be used to reduce issues with sudden
pitch changes when the flare happens.

After the Flare
---------------

After the plane flares it continues to navigate, but with zero throttle.
The navigation direction is a line extrapolated forward through the
landing point from the last waypoint. Note that the navigation roll will
be limited to
:ref:`LEVEL_ROLL_LIMIT <LEVEL_ROLL_LIMIT>`
(which defaults to 5 degrees) to prevent wing strike, so if there is a
significant cross-wind then it is likely that the aircraft will not be
able to maintain the exact path.

If your aircraft is consistently landing long (which can happen for a
variety of reasons) then you can adjust
:ref:`TECS_LAND_SRC <TECS_LAND_SRC>` to
either force a stall (negative) or bring it down (positive). This value
will adjust your ``TECS_LAND_SINK`` proportional to the distance from
the LAND point. This helps ensure you land in a reasonable distance from
the LAND point.

.. note::

   Possible causes of landing long include ground effect giving the
   aircraft more lift as it is close to the ground or simply the aircraft
   traveling very fast.

When the plane has stopped moving for
:ref:`LAND_DISARMDELAY <LAND_DISARMDELAY>`
seconds (default 20 seconds) it will disarm the motor. Optionally, you
can disable servo movement once LAND_DISARMDELAY has triggered by
setting :ref:`LAND_THEN_NEUTRL <LAND_THEN_NEUTRL>`.

Using a rangefinder
-------------------

If you have :ref:`fitted a rangefinder <common-rangefinder-landingpage>`
to your aircraft then you can use it for much more accurate landing
control. To allow the rangefinder to be used for landing you need to set
the :ref:`RNGFND_LANDING <RNGFND_LANDING>` parameter to 1.

When using a rangefinder for landing the altitude given by the
rangefinder is used only in the landing approach and to determine the
flare point, and is designed to allow the aircraft to more accurately
follow the glide slope and to flare at the right time.

.. note::

   The effectiveness of a rangefinder can depend on the surface you
   are flying over, so it is a good idea to do some low passes in a flight
   mode such as FBWA first, then examine the logs to check that the
   rangefinder is working correctly.

Also note that if you have a longer range rangefinder then it is a very
good idea to set the minimum range of the rangerfinder well above zero.
For example, the PulsedLight Lidar has a typical range of over 40
meters, and when it gets false readings it tends to read ranges of less
than 1 meter. Setting :ref:`RNGFND_MIN_CM <RNGFND_MIN_CM>`
to 150 will discard any rangerfinder readings below 1.5 meters, and will
greatly improve the robustness of the Lidar for landing.

Improving the landing
---------------------

The key to a good landing is the autopilot knowing how far off the
ground it is. With the default setup the only sensor available to detect
altitude is the barometer. Unfortunately barometers suffer from three
main types of error:

-  barometric drift due to changes in atmospheric pressure
-  barometric drift due to changes in the temperature of the autopilot
   electronics
-  barometric error due to local pressure changes from airflow around
   the barometer

The ideal setup for good automatic landing is to have a
:ref:`Lidar <common-rangefinder-landingpage>`. A Lidar can measure
the distance to the ground very accurately, and doesn't suffer from
drift. If you have a Lidar installed you can enable its use for landing
with ``RNGFND_LANDING=1``.

If a Lidar isn't fitted then there are a few things you can do to
minimise barometric error problems with auto-land

-  perform a barometer calibration after the electronics have warmed up.
   The easiest way to do this with a Pixhawk is to disarm the plane with
   the safety switch. When the plane is disarmed it assumes it is on the
   ground and will zero the barometer to the current pressure.
-  try to prevent direct airflow over the autopilot that could cause
   speed related pressure changes
-  fly shorter flights, allowing for less time for airpressure changes.
   Check your logs and see if the landing is happening at zero altitude
   consistently

With planes that belly land it can also work well to setup the landing
with a shallow pitch (in ``LAND_PITCH_CD``) and set a slightly higher
altitude to flare at. That will only work if your stall speed is low
enough that gliding for a while will work reliably.

Using DO_LAND_START
=====================

Sometimes it is useful to trigger an automatic landing as part of an RTL
(return to launch). To do this you need to do two things:

-  add a :ref:`DO_LAND_START <mav_cmd_do_land_start>`
   mission item to your mission, just before the start of your landing
   sequence
-  set the :ref:`RTL_AUTOLAND <RTL_AUTOLAND>`
   parameter to 1 or 2

The way it works is that when the plane enters an RTL it checks to see
if the parameter RTL_AUTOLAND is set to 1 or 2. If it is then the
current mission is searched for a mission item of type DO_LAND_START.
If one is found then the plane will automatically enter AUTO mode and
land, starting at the part of the mission just after the
``DO_LAND_START`` marker.

The exact behaviour depends on the ``RTL_AUTOLAND`` value:

-  If ``RTL_AUTOLAND=1`` then the plane will first RTL as normal, then
   when it starts circling the return point (home or a rally point) it
   will then switch to the AUTO mission after the ``DO_LAND_START`` and
   land
-  If ``RTL_AUTOLAND=2`` then the plane will bypass the RTL completely
   and go straight to the landing sequence.

You can optionally include more than one ``DO_LAND_START`` mission item
in your mission. If that is done then the latitude/longitude of the
``DO_LAND_START`` mission items is used to choose which landing sequence
to use. The ``DO_LAND_START`` closest to the current location is used.
This can be useful if you have multiple landing sequences for different
wind conditions or different areas.

How to abort an auto-landing
=====================================
A landing-abort mechanism is provided to allow you to abort a landing sequence in a safe, controlled, and expected way. Custom abort behaviour can be pre-programmed as part of the mission or you can use the default abort mechanism. To enable this feature set param LAND_ABORT_THR=1.
 
There are three steps to this feature:
#. Trigger an abort
#. The behavior during the abort
#. The mission state after the abort completes.

.. note::

   This section describes the abort behavior introduced in Plane
   3.4.

   
Step 1) Abort land triggers
--------------
The are three ways to trigger an auto-landing abort. All of them will only work while in AUTO mode and currently executing a ``LAND`` waypoint mission item:

-  *Send the ``MAV_CMD_DO_GO_AROUND`` command using a GCS.* Mission Planner has a button labeled "Abort Landing" on the FlightData Actions tab.
-  *RC input Throttle > 90%*. This will trigger an abort while staying in AUTO mode. The throttle only needs to be high briefly to trigger it. Don't forget to lower it!
-  *Mode change*. For human piloted landing abort you can switch out of AUTO mode into, for example MANUAL/STABILIZE/FBWA, and navigate the aircraft safely however you'd like. Using this method will skip abort behavior step 2 because it is being done manually. When switching back to AUTO the mission will resume as described in step 3 below.


Step 2) Abort land flight behavior
----------------------------------
The abort behaviour has a default configuration and does not require a pre-planned mission. The default abort behavior is to simulate an auto-takeoff: pitch up at least 10 degrees and set throttle to TKOFF_THR_MAX and hold the heading until it reaches a target altitude of 30m. It is possible to override the pitch and altitude to allow for a customized behavior.

- Pitch minimum. If there was a NAV_TAKEOFF ever executed on this mission then the same pitch will be re-used here.
- Target altitude. If NAV_LAND param1 is >0 then it is used as a target altitude in meters. Else If a NAV_TAKEOFF was ever executed on this mission then the same altitude will be re-used here.
  
This step is skipped if the abort trigger is via mode change because it is assumed the pilot manually took over and flew the aircraft to a safe altitude at the pitch and throttle of their choosing.


Step 3) Mission state after an aborted landing completes
----------------------------------
Once an abort land has completed, by either reaching the target altitude or switching back to AUTO, the mission index will have changed and you will no longer be executing a NAV_LAND command. The mission index will change to be one of these three options and checked for in this order:

- If the NAV_LAND mission item is followed by mission item :ref:`CONTINUE_AND_CHANGE_ALT <mav_cmd_nav_continue_and_change_alt>` with param1 = 0 or 1 then the mission index will increment once to that command and execute it like normal. This can be followed by further post-abort mission planning for any custom planned mission behavior.
- Else If there is a :ref:`DO_LAND_START <mav_cmd_do_land_start>` in the mission then it jumps to that index.
- Else the mission index decrements once to be the index before the NAV_LAND. This will ensure the same landing approach is repeated.


.. _reverse-thrust:

Reverse-Thrust Landing
======================

Some ESC's allow for reverse direction. When using reverse on the
propeller it will generate a negative thrust which can be used to slow
you down. During a steep landing approach this method can be used to
maintain a stable airspeed allowing you to land much more precisely even
with a LiDAR Baro bump on the approach. To use this feature it is highly
recommend to use an airspeed sensor  and a rangefinder (see above) for
an accurate altitude.

.. note::

   Reverse-thrust landings are available starting from Plane
   v3.5.1.

Key Parameters
--------------

The key parameters that control reverse thrust landing in addition to
the ones :ref:`listed in section 1.1 <automatic-landing_key_parameters>` are:

-  :ref:`LAND_PF_ALT <LAND_PF_ALT>`
-  :ref:`LAND_PF_SEC <LAND_PF_SEC>`
-  :ref:`LAND_PF_ARSPD <LAND_PF_ARSPD>`
-  :ref:`USE_REV_THRUST <USE_REV_THRUST>`
-  :ref:`TECS_APPR_SMAX <TECS_APPR_SMAX>`
-  :ref:`RC3_TRIM <RC3_TRIM>`
-  :ref:`THR_MIN <THR_MIN>`



ESC (Electronic Speed Controller)
---------------------------------

Hardware selection and programming
++++++++++++++++++++++++++++++++++

Most ESCs can operate in forwards and reverse, however that is usually not a stock feature 
and may need to be reprogrammed to do it. Any SimonK and BLHeli compatible ESC can be 
flashed to support reverse thrust. 

`Here's info about BLHeli compatible ones <https://blhelisuite.wordpress.com/>`__.


Hardware configuration
++++++++++++++++++++++

.. note::

   Remove propeller while configuring ESCs.

Configure your ESC for reverse thrust by changing it's neutral point.
Many ESC require custom firmware to accomplish this. Search google or
your ESC's mfgr for instructions on how to configure your particular
ESC.

Set these:

#. Minimum PWM to 1000, mid to 1500, and maximum to 2000.
#. ``THR_MIN`` to a negative value such -100. Next set ``RC3_TRIM`` (or
   whatever ``RCx`` is mapped to throttle via ``RCMAP_THROTTLE``) to
   your ESC's mid value.

Determining your max glide slope angle
--------------------------------------

For a steep landing approach, the limitation is how well you can
maintain your desired airspeed. This is determined by your aircraft's
ability to create reverse thrust (motor+prop combo) and its resistance
to slowing down (aircraft mass). In most cases extreme steepness is
unnecessary, but possible. With an over-sized motor and lightweight
aircraft you can come in as steep as 60 degrees.

To determine your steepest approach angle, set :ref:`TECS_APPR_SMAX <TECS_APPR_SMAX>`
very high as to not limit you (e.g. 99). Next, plan a mission with a
steeper than normal approach (try 15 degrees and go up from there).
Watch your airspeed on the approach - the plane should be able to
maintain :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>` with
only 75% of the available reverse throttle range. If not, you're coming
in too steep for the negative-thrust-to-mass ratio of your aircraft.

.. tip::

   Keep in mind that whatever value you determine as your maximum may
   not be acceptable in all wind conditions. It is best to be a little
   conservative.

Setting up the Pre-Flare
------------------------

With a rangefinder and airspeed sensors installed, at the pre-flare
point we will have an accurate airspeed and altitude reading. This gives
us a good idea of our momentum and stable "initial conditions" to the
final flare. Set ``LAND_PF_ALT`` (or ``LAND_PF_SEC``) to a fairly high
point (for example 10m) and adjust from there. Next
set ``LAND_PF_ARSPD`` to a value just above your stall speed.

When LAND_PF_ALT is reached the airspeed demand will instantly go
from :ref:`TECS_LAND_ARSPD <TECS_LAND_ARSPD>` to LAND_PF_ARSPD.
This will cause it to slam on the brakes via increased reverse thrust to
reduce speed instead of just maintaining a given speed.

The trick is to set ``LAND_PF_ALT`` to an altitude where it
achieves ``LAND_PF_ARSPD`` before killing the throttle at
``LAND_FLARE_ALT`` (which occurs at a somewhat low altitude - around 1
or 2m).

Example, ``TECS_LAND_ARSPD = 15``, ``LAND_PF_ARSPD = 12``, ``LAND_PF_ALT=12``, ``LAND_FLARE_ALT=2``.
Depending on your slope, mass of aircraft and motor+propellor thrust
ability, you're expecting the aircraft to decelerate from 15 to 12m/s
airspeed while dropping 10m. These are the critical params to adjust to
ensure a smooth and slow flare.

Flare
-----

Now that you are starting the flare with a stable and predictable
airspeed, it's much easier to :ref:`control the flare <automatic-landing_controlling_the_flare>`. If you've already
tuned your flare for an auto-land without reverse thrust you'll want to
retune it. You'll notice you're coming in much slower.

Other benefits of reverse-thrust landings
-----------------------------------------

LiDAR baro bump is handled better
+++++++++++++++++++++++++++++++++

On a long duration flight the baro drift will cause an altitude offset
that is not detectable until the LiDAR detects the ground (at which
point the aircraft "snaps" to the glide-slope). This causes an increased
airspeed moments before your flare, causing a touch-down beyond the
intended land point. With reverse-thrust the "snap" still happens, but
the TECS controller automatically changes the throttle demand to
maintain the desired airspeed.

Determining actual stall speed of your aircraft
+++++++++++++++++++++++++++++++++++++++++++++++

Unless you really know what you're doing, stall speed can be hard to
estimate. To be sure of the value you normally need to slowly decrease
your airspeed until you stall - with the consequent problem that now you
have a stalled plane falling out of the sky.

With LAND_PF_ALT and LAND_PF_ARSPD you can check your stall speed
much lower to the ground. To know the exact moment it stalls, check your
logs for when roll and roll_desired diverge.
