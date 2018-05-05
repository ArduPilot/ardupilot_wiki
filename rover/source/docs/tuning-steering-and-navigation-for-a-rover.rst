.. _tuning-steering-and-navigation-for-a-rover:

==============================================================
Tuning Steering and navigation for a Rover (ver 3.1 and older)
==============================================================

.. note::

   These instructions are relevant for Rover-3.1 and older.
   For newer versions of the Rover firmware please refer to these :ref:`steering <rover-tuning-steering-rate>` and
   :ref:`navigation tuning guides <rover-tuning-navigation>`.

When setting up a Rover there are 6 key parameters you need to get
right. This guide provides a set of steps to get those parameters right,
so that your rover will accurately navigate in AUTO mode.

The key parameters
==================

-  ``STEER2SRV_P``: This tells the rover code what the turning circle
   (as a diameter in meters) is for your rover. It is critical that you
   get this parameter right, as it tells the code what steering angle to
   choose to achieve a desired turn rate.
-  ``TURN_MAX_G``: This tells the rover the maximum G force (in
   multiples of one gravity) that your rover can handle while remaining
   stable. If you set this too high then your rover may flip over in
   turns, or skid. If you set it too low then your rover will not be
   able to turn sharply enough.
-  ``NAVL1_PERIOD``: This controls the aggressiveness of the navigation
   algorithm. A smaller number means more aggressive turning in AUTO
   mode, a large number means larger, smoother turns.
-  ``SPEED_TURN_GAIN``: This controls how much the rover should slow
   down while turning, as a percentage of current target speed. To not
   slow down at all in turns set this to 100. When doing initial tuning
   it is recommended you use 100, and lower this later to improve
   handling of tight courses.
-  ``CRUISE_SPEED``: This controls the target speed in meters/second in
   AUTO.
-  ``CRUISE_THROTTLE``: This sets the initial guess at what throttle is
   needed to achieve CRUISE_SPEED when driving straight ahead. This
   needs to be **RIGHT** for the rover to achieve good speed control.

Step 1: Setting initial parameters
==================================

To start the tuning process set the following values:

-  STEER2SRV_P = 2
-  TURN_MAX_G = 1
-  NAVL1_PERIOD = 6
-  SPEED_TURN_GAIN = 100
-  CRUISE_SPEED = 2
-  CRUISE_THROTTLE = 20

These are conservative values that should give you slow, gentle
behaviour for most rovers.

Step 2: Setting CRUISE_THROTTLE
===============================

We need to get ``CRUISE_THROTTLE`` right so we know what throttle level
will give us a speed of 2 meters/second.

-  Switch to MANUAL mode
-  Slowly advance the throttle until the ground station shows a speed of
   2 meters/second
-  note the throttle percentage shown in the HUD
-  set CRUISE_THROTTLE to the percentage needed for straight driving at
   2 meters/second

Step 3: Setting the STEER2SRV_P
===============================

To set your STEER2SRV_P parameter you need to measure the diameter of
the turning circle of your rover.

Put your rover into MANUAL mode, and put the steering hard over to one
side. Then very slowly drive your rover in a circle. Use a tape measure
to measure the diameter of that circle and set STEER2SRV_P to that
value in meters.

Step 4: tuning TURN_MAX_G
=========================

The ``TURN_MAX_G`` parameter can now be tuned so that your rover can
drive more aggressively, without turning over.

-  put into STEERING mode
-  turn at maximum turn rate and maximum throttle
-  adjust TURN_MAX_G to highest level that rover remains stable,
   doesn't start to roll over and doesn't skid

Step 5: tuning NAVL1_PERIOD
===========================

Now you can finally start tuning your steering in AUTO mode. To tune in
AUTO you will need to create a mission for your rover to navigate. A
simple rectangular mission is the best to start with.

-  create a rectangular course for the rover
-  run the course in AUTO mode
-  if the rover weaves along the straights, then raise NAVL1_PERIOD in
   increments of 0.5
-  if the rover doesn't turn sharply enough then lower NAVL1_PERIOD in
   increments of 0.5
-  if the rover is turning too late in corners then raise WP_RADIUS in
   increments of 1.0

You may also find that you may need to raise NAVL1_DAMPING slightly to
improve navigation in tight courses.

Final Setup
===========

Now that you have the basic parameters tuned you may find you can go
back and raise CRUISE_SPEED for faster driving. If you do, then make
sure you adjust CRUISE_THROTTLE to suit.

You may also like to lower SPEED_TURN_GAIN to ask the rover to slow
down a bit when turning. That would allow for higher speeds when driving
straight ahead.

There are a number of other parameters that can be used to fine tune
your steering once you have completed the basic ones above. Please see
the :doc:`full parameter list <parameters>` for details.

Some parameters to pay particular attention to are STEER2SRV_TCONST,
STEER2SRV_I, STEER2SRV_D and STEER2SRV_IMAX.

Fixing problems with weaving
============================

A common issue with rovers is that the steering 'weaves', turning from
side to side rather than turning smoothly. There can be several possible
reasons for this happening.

The first thing you need to work out is if the problem is confined to
low speed driving or also affects higher speed driving. If the problem
only happens at very low speed (such as when first entering auto on a
mission) then the most likely problem is that STEER2SRV_MINSPD is too
low. The default is 1.0 m/s, which is quite low, and if your GPS heading
isn't very reliable at low speed then you may need to raise that number.
Try 2.0 and see if that helps with low speed weaving.

If the problem also happens at higher speeds then it is likely to either
be the L1 navigation tuning or the steering controller tuning. To fix it
you will need to understand how these parameters work.

The NAVL1_PERIOD controls how rapidly the L1 navigation controller
changes the demanded steering direction. Making this number larger will
reduce weaving in AUTO, but it will also mean that you can't steer as
accurately around tight corners. Try raising NAVL1_PERIOD in steps of
0.5 until the weaving stops. For most rovers a value between 6.0 and 8.0
is good, but some rovers may need higher values.

If raising the NAVL1_PERIOD fixes the weaving but leaves you unable to
handle sharp turns in missions then you will instead need to tune the
steering controller.

In the steering controller there are 3 key parameters that will control
weaving:

-  A smaller STEER2SRV_P will reduce weaving, try reducing it by 0.1 at
   a time
-  A larger STEER2SRV_D will "damp" the weaving, but if you make it too
   large then you will get high speed oscillation. For example you may
   find that a value of 0.1 reduces the damping, but a value of 0.2
   could cause a high speed oscillation in the steering servo. If you
   get fast oscillation then reduce the STEER2SRV_D value by 50%.
-  A larger STEER2SRV_TCONST will slow down the steering controller,
   which will reduce weaving. Try raising it in steps of 0.1.

With the current controller system you do need to experiment a bit with
these values to get the behaviour you want. We hope to introduce an
automatic tuning system in future, but for now manual tuning is needed.
