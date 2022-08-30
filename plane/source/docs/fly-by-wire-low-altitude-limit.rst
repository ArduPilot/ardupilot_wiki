.. _fly-by-wire-low-altitude-limit:

==============================
Fly-By-Wire Low Altitude Limit
==============================

This function of Plane allows you to set a minimum altitude for FBW-B
mode (even if you don't have airspeed sensor) which your airplane will
try to stay above.

When FBW Low Alt Limit is enabled, if your airplane goes lower than
defined alt it will level up and climb back to the safe altitude. After
it is reached you will regain full control.

Use for R/C training
====================

This function will help those of us who doesn't have that much flying
skills to fly safely. It is really useful for FPV flying as FBW-B mode
provides easy control and the Low Altitude Limit will help you avoid
crashing into the ground ;-)

`Here <http://youtu.be/9wysVRrOmcQ>`__ is a video demo. APM is in FBW-B
mode and I hold pitch all the way DOWN! until the very end of video.
Minimum alt was set 30 meters above home altitude, so you can see
ground.

How to setup Alt Limit
======================

Using your APM Mission Planner you will have to set parameter
:ref:`ALT_HOLD_FBWCM<ALT_HOLD_FBWCM>` to the desired alt **in centimeters!** If it
is **0** then it is turned off.

Choosing the right altitude
===========================

Desired altitude should be set very carefully. As we know our planes
can't react immediately, so we have to choose altitude with this in
mind. I recommend using 3000 (30 meters) for pretty flat terrain.

Landing
=======

In order to land your plane you will need to disable this feature by
changing your flightmode to anything other than FBW-B.
