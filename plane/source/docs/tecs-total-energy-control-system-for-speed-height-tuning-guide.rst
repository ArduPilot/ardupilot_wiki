.. _tecs-total-energy-control-system-for-speed-height-tuning-guide:

====================================================================
TECS (Total Energy Control System) for Speed and Height Tuning Guide
====================================================================

This article explains how to tune the TECs system for speed and height
control.

Calibrate the airspeed sensor
=============================

This only applies if you are using an airspeed sensor.

A properly installed and calibrated airspeed sensor enables your model
to fly with much better control over its airspeed, which in turn enables
lower speeds to be used safely, extending endurance. For this reason if
you are interested in maximising the climb performance and endurance of
your aircraft, using an airspeed sensor and setting it up properly is
worth the effort. Instructions on how to calibrate an airspeed sensor
:ref:`can be found here <calibrating-an-airspeed-sensor>`.

.. note::

   *Glider Pilots:* Set :ref:`TECS_SPDWEIGHT <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_spdweight>` to 2.0,
   and you can also use an airspeed sensor and take advantage of TECS
   ability to control your airspeed.

Tune the Pitch to Servo Loop
============================

The performance of the TECS speed and height controller is dependent on
having the pitch to servo control loop tuned correctly. Before tuning
TECS you need to have tuned the pitch loop following the instructions
in :ref:`Roll, Pitch and Yaw Controller Tuning <roll-pitch-controller-tuning>`.

Set initial parameters — throttle, pitch, airspeed and vertical speed limits
============================================================================

If you have a properly calibrated airspeed sensor and a well tuned pitch
to servo loop, then the throttle, pitch angle, airspeed and vertical
speed limits limits are the final keys to getting the TECS algorithm to
perform well.

The following instructions are based around taking a series of
measurements, whilst the plane is being flown at the speed set by :ref:`TRIM_ARSPD_CM <TRIM_ARSPD_CM>`
which represents the speed that you are most likely going to be flying
at. Flying these tests at :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` is impractical unless you
have an assistant calling out airspeed, so where these instructions ask
for the plane to be flown at :ref:`TRIM_ARSPD_CM <TRIM_ARSPD_CM>`, fly at a speed in the middle zone between your :ref:`ARSPD_FBW_MIN <tecs-total-energy-control-system-for-speed-height-tuning-guide_thr_min>` and
:ref:`ARSPD_FBW_MAX <tecs-total-energy-control-system-for-speed-height-tuning-guide_thr_max>` and that will be good enough for
most applications. Those users wanting to extract maximum performance
can dial the numbers in over a number of flights, using the log data.

#. Set the maximum throttle percentage :ref:`THR_MAX <tecs-total-energy-control-system-for-speed-height-tuning-guide_thr_max>`. The
   maximum throttle should be set to a value that enables the aircraft
   to climb at its maximum pitch angle (default value of 20 degrees) at
   :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` . The default value of 75% should be OK for
   moderately powered aircraft. Low powered aircraft will need to
   increase it to 100%, high powered models (capable of vertical climbs)
   will need to reduce it.
#. Set the throttle percentage required to fly level at
   :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` by adjusting the :ref:`TRIM_THROTTLE <tecs-total-energy-control-system-for-speed-height-tuning-guide_trim_throttle>` parameter. This can be determined
   initially by testing different throttle settings in FBWA mode.
#. Set the maximum and minimum airspeed limits (in metres/second) using
   the :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` and :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` parameters.
   :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` should be set to just slightly less than the maximum speed your aircraft is
   capable of in level flight with the throttle set to :ref:`THR_MAX<THR_MAX>` .
   :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` should be set to the slowest speed your aircraft
   can safely fly without stalling in level flight.
#. Set the maximum pitch angle :ref:`LIM_PITCH_MAX <tecs-total-energy-control-system-for-speed-height-tuning-guide_lim_pitch_max>` (in
   centi-degrees) your aircraft can fly with the throttle set to
   :ref:`THR_MAX<THR_MAX>` . This can be determined by performing maximum pitch angle
   climbs in FBWA with the throttle set to the maximum and checking the
   airspeed during climb. If the airspeed rises above :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` during
   the climb, then either increase :ref:`LIM_PITCH_MAX<LIM_PITCH_MAX>` or
   reduce :ref:`THR_MAX<THR_MAX>` . If it falls below :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` during
   climb then either reduce :ref:`LIM_PITCH_MAX<LIM_PITCH_MAX>` or
   increase :ref:`THR_MAX<THR_MAX>`  . Make sure you allow some margin for reduction
   in power due to reduced battery voltage or other effects. Remember
   that the amount of power from an electric power system at the end of
   flight will only be 80% of what you have at the start.
#. Set the minimum pitch angle :ref:`LIM_PITCH_MIN <tecs-total-energy-control-system-for-speed-height-tuning-guide_lim_pitch_min>`
   (in centi-degrees) your aircraft can fly with the throttle set to
   :ref:`THR_MIN <tecs-total-energy-control-system-for-speed-height-tuning-guide_thr_min>` that can be flown without over-speeding the
   aircraft.
#. Set the maximum climb rate :ref:`TECS_CLMB_MAX <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_clmb_max>` (in
   metres/second). This is the best climb rate that the aircraft can
   achieve with the throttle set to :ref:`THR_MAX<THR_MAX>` and flying at
   :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` . For electric aircraft make sure this number can be
   achieved towards the end of flight when the battery voltage has
   reduced. This can be measured in FBWA mode by performing climbs to
   height with the throttle set to :ref:`THR_MAX <THR_MAX>`.
#. Set the minimum sink rate :ref:`TECS_SINK_MIN <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_sink_min>` (in
   metres/second). This is the sink rate of the aircraft with the
   throttle set to :ref:`THR_MIN<THR_MIN>` and flown at :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` . This can
   be measured by closing the throttle in FBWA and gliding the aircraft
   down from height.
#. Set the maximum sink rate  :ref:`TECS_SINK_MAX <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_sink_max>` (in
   metres/second). If this value is too large, the aircraft can
   over-speed on descent. This should be set to a value that can be
   achieved without exceeding the lower pitch angle limit and without
   exceeding :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` .

Flight Testing
==============

#. Place the aircraft into a loiter about a waypoint either using auto,
   RTL or guided mode. Check that the aircraft maintains height without
   noticeable pitching or height changes greater than 10m. If the
   aircraft appears to be oscillating in height then try
   increasing :ref:`TECS_TIME_CONST <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_time_const>` in increments
   of 1 (do not increase to more than 10). If you need to increase to
   more than 10 to reduce the oscillation in height, then this normally
   indicates a problem with the pitch to servo loop tuning or the
   settings of the pitch angle and climb rate limits.

#. Verify that :ref:`THR_MAX<THR_MAX>` , :ref:`LIM_PITCH_MAX<LIM_PITCH_MAX>` and :ref:`TECS_CLMB_MAX<TECS_CLMB_MAX>`
   are set correctly. The setting of these parameters can be checked by
   commanding a positive altitude change of no less than 50m in loiter,
   RTL or guided mode. The objective is to set these parameters such
   that the throttle required to climb is about 80% of :ref:`THR_MAX<THR_MAX>` ,
   the aircraft is maintaining airspeed, and the demanded pitch angle is
   about 5 degrees below :ref:`LIM_PITCH_MAX<LIM_PITCH_MAX>` .

   #. If speed drops below the desired value, and the throttle increases
      to and stays on :ref:`THR_MAX<THR_MAX>` , then either :ref:`TECS_CLMB_MAX<TECS_CLMB_MAX>` should
      be reduced or :ref:`THR_MAX<THR_MAX>` increased.
   #. If the demanded pitch angle is constantly at the limit set
      by :ref:`LIM_PITCH_MAX<LIM_PITCH_MAX>` , then either the pitch angle
      limit :ref:`LIM_PITCH_MAX<LIM_PITCH_MAX>` needs to be increased or the maximum
      climb rate :ref:`TECS_CLMB_MAX<TECS_CLMB_MAX>` needs to be reduced.

#. Verify :ref:`LIM_PITCH_MIN<LIM_PITCH_MIN>` and :ref:`TECS_SINK_MAX<TECS_SINK_MAX>` are set correctly. The
   setting of these parameters can be checked by commanding a negative
   altitude change of no less than 50m in loiter, RTL or guided mode. The
   objective is to set these parameters such that the throttle is on
   :ref:`THR_MIN<THR_MIN>` , the airspeed is below :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>` (or visually
   confirm that model is not gaining too much speed if an airspeed sensor
   is not being used), and the demanded pitch angle is about 5 degrees
   above :ref:`LIM_PITCH_MIN<LIM_PITCH_MIN>` .

   #. If the speed is too high, then :ref:`TECS_SINK_MAX<TECS_SINK_MAX>` should be reduced.
   #. If the demanded pitch angle is constantly at the limit set
      by :ref:`LIM_PITCH_MIN<LIM_PITCH_MIN>` , then either the pitch angle
      limit :ref:`LIM_PITCH_MIN<LIM_PITCH_MIN>` needs to be reduced (become more negative)
      or the maximum sink rate :ref:`TECS_SINK_MAX<TECS_SINK_MAX>` needs to be reduced.

If the height response oscillates you can try increasing the value of
`TECS_PTCH_DAMP <#TECS_PTCH_DAMP>`__ in increments of 0.1 (don't go
above 0.5 unless you know how to check for excessive noise in the
nav_pitch signal using the mission planner tuning window) and then try
increasing the value of :ref:`TECS_TIME_CONST<TECS_TIME_CONST>` in increments of 1.0.

.. note::

   If you are not using an airspeed sensor and you have problems with
   pitch and height oscillation using the default parameters, then it
   usually indicates that your pitch to servo loop has not been tuned
   properly or your model could have a significant thrust line misalignment
   where throttle changes cause noticeable pitch angle changes. Ideally you
   should improve your pitch loop tuning first, before adjusting
   :ref:`TECS_PTCH_DAMP<TECS_PTCH_DAMP>` and :ref:`TECS_TIME_CONST<TECS_TIME_CONST>` as described here.

If using airspeed sensing, adjust the value of :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` so
that it matches the average amount of throttle required by the
controller during constant height loiter. If not using airspeed sensing,
adjust :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` to achieve a level flight speed you are happy
with.


Fine Tuning
===========

The following parameters can be adjusted to fine-tune the controller
response:

.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_thr_slewrate:

:ref:`THR_SLEWRATE <THR_SLEWRATE>`: This
is the maximum % change in throttle over one second . A setting of 100
means to not change the throttle by more than 100% of the full throttle
range in one second. Reducing this value will reduce the amount of
throttle 'surging' in windy conditions but will reduce controller
accuracy and will produce oscillation in throttle, speed and height if
reduced too much.


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_thr_damp:

:ref:`TECS_THR_DAMP <TECS_THR_DAMP>`:
This is the damping gain for the throttle demand loop. Increase to add
damping to correct for oscillations in speed and height. **This gain has
no effect if an airspeed sensor is not being used.**


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_integ_gain:

:ref:`TECS_INTEG_GAIN <TECS_INTEG_GAIN>`: This
is the integrator gain on the control loop. Increasing this gain
increases the speed at which speed and height offsets are trimmed out,
but reduces damping and increases overshoot.


:ref:`TECS_RLL2THR <TECS_RLL2THR>`:
Increasing this gain turn increases the amount of throttle that will be
used to compensate for the additional drag created by turning. Ideally
this should be set to approximately 10 x the extra sink rate in m/s
created by a 45 degree bank turn. Increase this gain if the aircraft
initially loses energy in turns and reduce if the aircraft initially
gains energy in turns. Efficient high aspect-ratio aircraft (eg powered
sailplanes) can use a lower value, whereas inefficient low aspect-ratio
models (eg delta wings) can use a higher value. **This gain has no
effect if an airspeed sensor is not being used.**


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_spdweight:

:ref:`TECS_SPDWEIGHT <TECS_SPDWEIGHT>`:
This parameter adjusts the amount of weighting that the pitch control
applies to speed vs height errors. Setting it to 0.0 will cause the
pitch control to control height and ignore speed errors. This will
normally improve height accuracy but give larger airspeed
errors. Setting it to 2.0 will cause the pitch control loop to control
speed and ignore height errors. This will normally reduce airspeed
errors, but give larger height errors. The default value of 1.0 allows
the pitch control to simultaneously control height and speed.

.. note::

   This parameter is has no effect if the TECS has no airspeed estimate, in which
   case a value of 0.0 will be used. To provide an airspeed estimate an airspeed
   sensor must be installed, or :ref:`TECS_SYNAIRSPEED <TECS_SYNAIRSPEED>` must be
   set to 1.

.. note::

   **Glider Pilots**: Set this parameter to 2.0 (The glider will
   adjust its pitch angle to maintain airspeed, ignoring changes in
   height).

.. note::
   When the :ref:`soaring<soaring>` feature is in use and is requesting the TECS shut off
   throttle to glide, a value of 2.0 will automatically be used providing an airspeed
   estimate is available.

:ref:`TECS_PTCH_FF_K <TECS_PTCH_FF_K>`:
This parameter can be used together with :ref:`TECS_PTCH_FF_V0<TECS_PTCH_FF_V0>` to provide a 
feedforward gain between demanded airspeed and pitch attitude. This is best
used with :ref:`TECS_SPDWEIGHT<TECS_SPDWEIGHT>` set to 2.0. As noted above, this is appropriate for
gliders, and setting :ref:`TECS_PTCH_FF_K <TECS_PTCH_FF_K>` can improve the responsiveness to changes
in speed demand.

.. note::

   The units of this parameter are radians of pitch per metre per second between
   current demanded airspeed and TECS_PTCH_FF_V0. Appropriate values are negative
   (pitch down with increasing speed demand). Sensible starting values are -0.04
   for gliders and -0.08 for draggy airframes.

To tune this parameter, either use FBWB to manually input speed demand changes,
or set up a mission involving DO_CHANGE_SPEED items. Set TECS_PTCH_FF_V0 to the
normal flight speed of your aircraft. This should also be the speed it glides at
with no pitch input in FBWA mode (i.e. when flying at a pitch attitude specified
by the STAB_PTCH_DOWN parameter). When reviewing the log from such a flight, look
at the TECS pitch integrator item (TECS.iph) in the onboard logs. Usually this 
reduces (becomes more negative) to trim the aircraft nose-down for a higher airspeed,
and vice versa. The goal is to use the feed-forward gain to reduce the required 
changes in this integrator value to trim the aircraft to a new airspeed. If the
TECS.iph value becomes more negative when the demanded airspeed increases, make the
:ref:`TECS_PTCH_FF_K <TECS_PTCH_FF_K>` more negative. If the TECS.iph value becomes 
more positive when the demanded aispeed increases, make the :ref:`TECS_PTCH_FF_K <TECS_PTCH_FF_K>`
value more positive. When this process in complete and the feed-forward gain is providing 
most of the pitch attitude change needed, the TECS.iph value doesn't need to change much.
This gives better tracking of changes in demanded airspeed.


Advanced Parameters
===================

.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_vert_acc:

:ref:`TECS_VERT_ACC <TECS_VERT_ACC>`:
This is the maximum vertical acceleration either up or down that the
controller will use to correct speed or height errors. The default value
of 7 m/s/s (equivalent to +- 0.7 g) allows for reasonably aggressive
pitch changes if required to recover from under-speed conditions.

.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_hgt_omega:

:ref:`TECS_HGT_OMEGA <TECS_HGT_OMEGA>`: 
This is the cross-over frequency (in radians/second) of the
complementary filter used to fuse vertical acceleration and barometric
height to obtain an estimate of height rate and height. Increasing this
frequency weights the solution more towards use of the barometer, whilst
reducing it weights the solution more towards use of the accelerometer data.


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_spd_omega:

:ref:`TECS_SPD_OMEGA <TECS_SPD_OMEGA>`: 
This is the cross-over frequency (in radians/second)of the
complementary filter used to fuse longitudinal acceleration and airspeed
to obtain an improved airspeed estimate. Increasing this frequency
weights the solution more towards use of the airspeed sensor, whilst
reducing it weights the solution more towards use of the accelerometer
data.

Complete Parameter List
=======================

.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_thr_max:

:ref:`THR_MAX <THR_MAX>`:
This is the maximum throttle % that can be used by the controller. For
overpowered aircraft, this should be reduced to a value that provides
sufficient thrust to climb at the maximum pitch angle :ref:`LIM_PITCH_MAX<LIM_PITCH_MAX>` .

.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_thr_min:

:ref:`THR_MIN <THR_MIN>`: This
is the minimum throttle % that can be used by the controller. For
electric aircraft this will normally be set to zero, but can be set to a
small non-zero value if a folding prop is fitted to prevent the prop
from folding and unfolding repeatedly in-flight or to provide some
aerodynamic drag from a turning prop to improve the descent rate.

:ref:`THR_SLEWRATE <tecs-total-energy-control-system-for-speed-height-tuning-guide_thr_slewrate>` (definition above)

.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_trim_throttle:

:ref:`TRIM_THROTTLE <TRIM_THROTTLE>`:
This is the throttle % required for level flight at the normal cruise
speed.


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_arspd_fbw_max:

:ref:`ARSPD_FBW_MAX <ARSPD_FBW_MAX>`:
This is the maximum airspeed (in metres/second) that the autopilot will
use in auto-throttle modes. It should be set to the highest speed that
the aircraft can achieve in level flight with the throttle set to
:ref:`THR_MAX<THR_MAX>` . It must be sufficiently above the :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` value
to allow the autopilot to accurately control altitude using airspeed (at
least 50% above :ref:`ARSPD_FBW_MIN<ARSPD_FBW_MIN>` is recommended). For electric
aircraft, make sure this number is achievable at the end of flight when
the battery voltage has reduced.

:ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>`: This
is the minimum indicated airspeed (in metres/second) that the speed
controller will attempt to control to. This should be set to a speed
that allows the aircraft to turn at the maximum bank angle without
stalling.


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_clmb_max:

:ref:`TECS_CLMB_MAX <TECS_CLMB_MAX>`: 
This is the best climb rate (in metres/second) that the aircraft can achieve
with the throttle set to :ref:`THR_MAX<THR_MAX>` and the airspeed set to the default
value. For electric aircraft make sure this number can be achieved
towards the end of flight when the battery voltage has reduced. The
setting of this parameter can be checked by commanding a positive
altitude change of 100m in loiter, RTL or guided mode. If the throttle
required to climb is close to :ref:`THR_MAX<THR_MAX>` and the aircraft is
maintaining airspeed, then this parameter is set correctly. If
the airspeed starts to reduce, then the parameter is set to high, and if
the throttle demand required to climb and maintain speed is noticeably
less than :ref:`THR_MAX<THR_MAX>` , then either :ref:`TECS_CLMB_MAX<TECS_CLMB_MAX>` should be increased or :ref:`THR_MAX<THR_MAX>` reduced. 



.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_sink_min:

:ref:`TECS_SINK_MIN <TECS_SINK_MIN>`: 
This is the sink rate of the aircraft (in metres/second) with the throttle
set to :ref:`THR_MIN<THR_MIN>` and flown at the same airspeed as used to measure
:ref:`TECS_CLMB_MAX`. 


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_time_const:

:ref:`TECS_TIME_CONST <TECS_TIME_CONST>`: 
This is the time constant of the TECS control algorithm (in seconds). Smaller
values make it faster to respond, larger values make it slower to respond.

:ref:`TECS_THR_DAMP <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_thr_damp>` (definition above)

:ref:`TECS_INTEG_GAIN <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_integ_gain>` (definition above)

:ref:`TECS_VERT_ACC <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_vert_acc>` (definition above)

:ref:`TECS_HGT_OMEGA <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_hgt_omega>` (definition above)

:ref:`TECS_SPD_OMEGA <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_spd_omega>` (definition above)


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_lim_pitch_max:

:ref:`LIM_PITCH_MAX <LIM_PITCH_MAX>`:
This is the maximum pitch angle (in centidegrees) that the controller
will demand. It should be set to a value that the aircraft can achieve
whilst maintaining airspeed with the throttle set to :ref:`THR_MAX`. 


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_lim_pitch_min:

:ref:`LIM_PITCH_MIN <LIM_PITCH_MIN>`:
This is the minimum pitch angle (in centidegrees) that the controller
will demand. It should be set to a value that the aircraft can achieve
without over-speeding with the throttle set to :ref:`THR_MIN<THR_MIN>` .

:ref:`TECS_RLL2THR <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_integ_gain>` (definition above)

:ref:`TECS_SPDWEIGHT <tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_spdweight>` (definition above)

:ref:`TECS_PTCH_DAMP <TECS_PTCH_DAMP>`:
This is the damping gain for the pitch demand loop. Increase to add
damping to correct for oscillations in height. The default value of 0.0
will work well provided the pitch to servo controller has been tuned
properly.


.. _tecs-total-energy-control-system-for-speed-height-tuning-guide_tecs_sink_max:

:ref:`TECS_SINK_MAX <TECS_SINK_MAX>`:
This sets the maximum descent rate (in metres/second) that the
controller will use. If this value is too large, the aircraft can
over-speed on descent. This should be set to a value that can be
achieved without exceeding the lower pitch angle limit and without
over-speeding the aircraft.

Algorithm Overview
==================

TECS stands for Total Energy Control System and for Plane refers to a
new control algorithm that coordinates throttle and pitch angle demands
to control the aircraft's height and airspeed. The underlying physics
behind the operation of TECS is simple, but to understand how it works
you need to understand the two types of mechanical energy that TECS
controls. These are:

::

    Gravitational Potential Energy = mass x gravity x height

and,

::

    Kinetic Energy = ½ x mass x speed²

Gravitational Potential Energy is the energy stored in an object due to
its height and is proportional to the height of the object. We all know
intuitively that to raise the height of an object requires energy and
that when an object falls energy is released. Similarly, to increase the
height of our aircraft requires more energy, which means more throttle
is required.

Kinetic energy is the energy stored in an object due to its velocity and
is proportional to velocity squared. An example is a rifle bullet, which
although it doesn't weigh very much, has a lot of energy due to its high
speed. Our aircraft don't travel at the speeds of rifle bullet, but they
do require energy to be supplied to increase their speed.

The total energy of the aircraft is the sum of the gravitational
potential energy and the kinetic energy. The drag acting on an aircraft
in flight is continually reducing its total energy, so the only way to
maintain height and speed is to supply thrust using a motor or utilise
energy from some other external source such as a rising air current.
TECS calculates the total energy required based on the demanded speed
and height and adjusts the throttle to maintain total energy at the
demanded value.

The other job of the TECS algorithm is to ensure that the balance
between gravitational potential and kinetic energy is correct. For
example if the aircraft is flying too slow and too high simultaneously,
its total energy might be correct, but there is too much potential
energy and not enough kinetic energy. TECS tries to maintain the correct
balance between potential and kinetic energy by adjusting the demanded
pitch angle. By lowering the nose energy is transferred from
gravitational potential to kinetic energy or vice-versa.

How much weighting is placed on kinetic energy or speed errors vs
potential energy or height errors is controlled by the
:ref:`TECS_SPDWEIGHT<TECS_SPDWEIGHT>` parameter. At the default setting of 1.0, even weight
is placed on speed and height errors. If  :ref:`TECS_SPDWEIGHT<TECS_SPDWEIGHT>` is set to
0.0 then the pitch angle demand will respond 100% to height errors and
ignore speed and if set to 2.0 will respond 100% to speed errors and
ignore height.

If the airspeed measurement is not used (as selected by
setting :ref:`ARSPD_USE<ARSPD_USE>` and :ref:`ARSPD2_USE<ARSPD2_USE>`  = 0), then the pitch angle will be used 100% to
control height and the throttle will be calculated from the demanded
pitch angle.
