.. _fly-by-wire-low-altitude-limit:

==============================
Fly-By-Wire Low Altitude Limit
==============================

This function of Plane allows you to set a minimum altitude for FBWB and CRUISE
modes (even if you don't have airspeed sensor) which your airplane will
try to stay above.

When :ref:`CRUISE_ALT_FLOOR<CRUISE_ALT_FLOOR>` is enabled, if your airplane goes lower than
defined altitude relative to HOME, it will level up and/or climb back to this altitude. After
it is reached you will regain altitude control.

.. note:: the minimum MSL altitude target will be the higher of: this altitude, or the current altitude target above terrain in FBWB or CRUISE modes if :ref:`common-terrain-following` is enabled, or the :ref:`FENCE_ALT_MIN<FENCE_ALT_MIN>` above HOME if that fence (See :ref:`common-geofencing-landing-page`) is enabled.

Use for R/C training
====================

This function will help those of us who doesn't have that much flying
skills to fly safely. It is really useful for FPV flying as FBWB or CRUISE modes
provide easy control and the :ref:`CRUISE_ALT_FLOOR<CRUISE_ALT_FLOOR>` limit will help you avoid
crashing into the ground ;-)

`Here <http://youtu.be/9wysVRrOmcQ>`__ is a video demo. APM is in FBW-B
mode and I hold pitch all the way DOWN! until the very end of video.
Minimum alt was set 30 meters above home altitude, so you can see
ground.

How to setup Alt Limit
======================

Using your APM Mission Planner you will have to set parameter
:ref:`CRUISE_ALT_FLOOR<CRUISE_ALT_FLOOR>` to the desired alt **in centimeters!** If it
is **0** then it is turned off.

Choosing the right altitude
===========================

Desired altitude should be set very carefully. As we know our planes
can't react immediately, so we have to choose altitude with this in
mind. Using 3000 (30 meters) is recommended for flights over pretty flat terrain.

Landing
=======

In order to land your plane you will need to disable this feature by
changing your flightmode to anything other than FBWB or CRUISE. FBWA, STABILIZE, or MANUAL modes are always the preferred modes for a manual landing, in any case.
