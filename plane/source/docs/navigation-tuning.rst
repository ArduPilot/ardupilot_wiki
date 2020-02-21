.. _navigation-tuning:

=================
Navigation Tuning
=================

Navigation Tuning with the L1 Controller
========================================

Starting with Plane 2.72 a new navigation controller called the "L1
controller" was introduced. This controller produces much more accurate
flight paths both for waypoints and loiter than the previous crosstrack
and PID controller.

Tuning the navigation controller usually involves adjusting one key
parameter, called :ref:`NAVL1_PERIOD<NAVL1_PERIOD>` . The default is 20, which results in
reasonable turns for most R/C airframes. A smaller value for
:ref:`NAVL1_PERIOD<NAVL1_PERIOD>` will lead to more aggressive navigation (sharper corners).
A larger value will lead to gentler navigation. You can additionally
adjust the :ref:`NAVL1_DAMPING<NAVL1_DAMPING>` and :ref:`WP_RADIUS<WP_RADIUS>` for further tuning.

Steps to tuning the L1 controller
=================================

-  Make sure you have already tuned the roll and pitch controller,
   correctly trimmed your plane, and have ensured the plane does not
   gain or lose altitude in turns
-  Make sure you have setup :ref:`LIM_ROLL_CD<LIM_ROLL_CD>` to an appropriate value for
   the bank angle you are comfortable with your plane flying without
   stalling. For slow flying electric gliders a value of around 5000 (50
   degrees) is a good start. For fast flying aerobatic planes you may
   wish to use a larger value, such as 6500 (65 degrees).
-  Setup a rectangular mission for your plane to fly, with a loop so it
   repeats the mission continuously. Ensure that the size of the mission
   is small enough that you will have a good view of the aircraft at all
   times
-  Set :ref:`NAVL1_PERIOD<NAVL1_PERIOD>` to the default value of 20, and :ref:`NAVL1_DAMPING<NAVL1_DAMPING>` to
   the default value of 0.75. Set :ref:`WP_RADIUS<WP_RADIUS>` to the distance your plane
   would fly in 2 seconds at cruise speed.
-  Takeoff and put the plane in AUTO using your transmitter switch
-  Observe the behaviour of the plane in the turns. If it turns too
   slowly then reduce :ref:`NAVL1_PERIOD<NAVL1_PERIOD>` by 5. If it is "weaving" after a
   turn then increase :ref:`NAVL1_PERIOD<NAVL1_PERIOD>` by 1 or 2
-  If you are tuning for maximum performance, once you have completed
   the tuning of :ref:`NAVL1_PERIOD<NAVL1_PERIOD>` you can increment :ref:`NAVL1_PERIOD<NAVL1_PERIOD>` by 1 and
   then modify :ref:`NAVL1_DAMPING<NAVL1_DAMPING>` in steps of 0.05 to get the
   response you want. Do not decrease :ref:`NAVL1_DAMPING<NAVL1_DAMPING>` too much - it is
   unlikely you will need a value below 0.6.

Tuning waypoint transition behaviour
====================================

To control whether Plane flies through a waypoint, then turns afterwards
or turns before the waypoint so that it neatly lines up for the next leg
of the mission you should adjust the waypoint radius via the :ref:`WP_RADIUS<WP_RADIUS>`
parameter. If you want Plane to fly through the waypoint then set the
:ref:`WP_RADIUS<WP_RADIUS>` to a small number, perhaps 10 meters. If you want Plane to
turn before the waypoint so that it lines up with the next leg of the
mission then set :ref:`WP_RADIUS<WP_RADIUS>` to a larger number. A good starting value is
to set it equal to the turn radius your aircraft can easily achieve.
This will normally be the same as your loiter radius :ref:`WP_LOITER_RAD<WP_LOITER_RAD>` .

L1 Controller Background & Description
======================================

The L1 controller concept is based on the following technical paper:

::

    S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for
    Trajectory Tracking," Proceedings of the AIAA Guidance, Navigation and
    Control Conference, Aug 2004. AIAA-2004-4900.

This was the basis for Brandon Jones’ original pull request:
https://github.com/ArduPilot/ardupilot/pull/101

These algorithms were subsequently modified by Paul Riseborough with the
following changes:

-  The L1 length was calculated dynamically to enable a constant period
   for the tracking loop to be specified by the user and to enable the
   navigation loop gain to automatically adjust for changes in aircraft
   ground speed. Achieving a constant period for the guidance loop gives
   a consistent response across a range of airspeeds and enables the
   tuning parameter to be related to the time required to roll the
   aircraft and measure its response.
-  The guidance gain was changed from a fixed value of 2 to be
   calculated based on the :ref:`NAVL1_DAMPING<NAVL1_DAMPING>` value set by the user. This
   enabled additional damping to be specified to compensate for delays
   in the velocity measurement and aircraft roll response.
-  A complementary filter fusing GPS velocities, airspeed and aircraft
   heading was used to estimate the ground speed vector. This enabled
   the accuracy of the GPs velocity to be taken advantage of, without
   the limitations imposed by its inherent latency.
-  The track capture algorithm was modified to enable explicit control
   over the track capture angle.
-  The waypoint circle tracking algorithm used during RTL, GUIDED and
   LOITER modes was modified to use a modified PD control law rather
   than the L1 control law. This was necessary to enable small loiter
   radius’ to be flown in combination with larger values of
   :ref:`NAVL1_PERIOD<NAVL1_PERIOD>`.
-  The distance from the next waypoint to start the turn onto the next
   track segment was modified to use the L1 length dynamically
   calculated by the algorithm, but constrained to be no greater than
   :ref:`WP_RADIUS<WP_RADIUS>` . This enabled the user to select whether they would rather
   fly through the waypoint and then turn, or turn early and smoothly
   intercept the next track.

.. image:: ../images/L1_loiter.png
    :target: ../_images/L1_loiter.png

.. image:: ../images/L1_WP_following.png
    :target: ../_images/L1_WP_following.png
