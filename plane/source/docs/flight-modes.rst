.. _flight-modes:

============
Flight Modes
============

This article provides links to Plane's flight modes.

Overview
========

Plane has a wide range of built in flight modes. Plane can act as a
simple flight stabilization system, a sophisticated autopilot, a
training system or a flight safety system depending on what flight mode
and options you choose.

Flight modes are controlled through a :ref:`radio transmitter switch <common-rc-transmitter-flight-mode-configuration>`, via mission
commands, or using commands from a ground station (GCS) or companion
computer.

Major Flight Modes
==================

All of the flight modes below have optional additional controls that may
be used to change the behaviour to suit particular flying needs. After
you have read the introductory material below it is highly recommended
that you look through the complete set of :doc:`Plane parameters <parameters>` so you can explore the full range
of functionality available.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Mode</th><th>Roll</th><th>Pitch</th><th>Throttle</th><th>GPS</th><th>Need TX</th><th>Summary</th></tr>
   <tr><td>MANUAL</td><td>-</td><td>-</td><td>-</td><td></td><td>Y</td><td>Manual control surface movement, passthrough</td></tr>
   <tr><td>FBWA</td><td>s</td><td>s</td><td>-</td><td></td><td>Y</td><td>Roll and pitch follow stick input, up to set limits</td></tr>
   <tr><td>FBWB</td><td>s</td><td>A</td><td>SPD</td><td>Y</td><td>Y</td><td>like FBWA, but with automatic height and speed control</td></tr>
   <tr><td>CRUISE</td><td>A</td><td>A</td><td>SPD</td><td>Y</td><td>Y</td><td>like FBWB, but with ground course tracking and terrain following</td></tr>
   <tr><td>STABILIZE</td><td>+</td><td>+</td><td>-</td><td></td><td>Y</td><td>Wing-leveling on stick release</td></tr>
   <tr><td>AUTOTUNE</td><td>s</td><td>s</td><td>-</td><td></td><td>Y</td><td>like FBWA, but learns attitude tuning while flying</td></tr>
   <tr><td>TRAINING</td><td>+</td><td>+</td><td>-</td><td></td><td>Y</td><td>Manual control up to roll and pitch limits</td></tr>
   <tr><td>ACRO</td><td>+</td><td>+</td><td>-</td><td></td><td>Y</td><td>rate controlled mode with no attitude limits</td></tr>
   <tr><td>Q(Copter Modes)</td><td>s</td><td>s</td><td>A</td><td>Y</td><td>Y</td><td>Varies depending on mode. See quadplane documentation</td></tr>
   <tr><td>AUTO</td><td>A</td><td>A</td><td>A</td><td>Y</td><td></td><td>Follows Mission</td></tr>
   <tr><td>LOITER</td><td>A</td><td>A</td><td>A</td><td>Y</td><td></td><td>Circles point where mode switched</td></tr>
   <tr><td>CIRCLE</td><td>A</td><td>A</td><td>A</td><td></td><td></td><td>Gently turns aircraft</td></tr>
   <tr><td>GUIDED</td><td>A</td><td>A</td><td>A</td><td>Y</td><td></td><td>Circles user defined point from GCS</td></tr>
   <tr><td>Return To Launch (RTL)</td><td>A</td><td>A</td><td>A</td><td>Y</td><td></td><td>Returns to and circles home or rally point</td></tr>
   <tr><td>LAND (AUTO)</td><td>A</td><td>A</td><td>A</td><td>Y</td><td></td><td>Final part of automatic mission for touchdown</td></tr>
   </table>
   
   
.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Symbol</th><th>Definition</th></tr>
   <tr><td>-</td><td>Full manual control of flight surfaces</td><tr>
   <tr><td>+</td><td>Manual control with stabilized limits or assistance</td><tr>
   <tr><td>s</td><td>Stabilized control with limits</td></tr>
   <tr><td>A</td><td>Automatic control</td></tr>
   <tr><td>SPD</td><td>Controls speed</td></tr>
   </table>

.. note:: Automatic throttle controlled modes can optionally have pilot-based speed adjustments using the throttle stick via the :ref:`THROTTLE_NUDGE<THROTTLE_NUDGE>` parameter. Otherwise, the autopilot will attempt to maintain :ref:`TRIM_ARSPD_CM<TRIM_ARSPD_CM>` airspeed if an airspeed sensor is being used, or :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` as a target throttle, as in FBWA and CRUISE modes.


.. warning::

   Any stabilized control mode requires tuning for optimal performance.
   The default PID and TECS values are very conservative to avoid uncontrollable 
   oscillations. Perform an :ref:`AUTOTUNE <automatic-tuning-with-autotune>` before using stabilized or automatic roll and pitch modes
   extensively. :ref:`Tune TECS <tecs-total-energy-control-system-for-speed-height-tuning-guide>` before using automatic throttle modes
   extensively.
   
   
Mission Specific Modes
======================

When flying an AUTO mission Plane has some sub-modes that are set using
mission items. The two main sub-modes are :ref:`TAKEOFF <takeoff-mode>` and :ref:`LAND <land-mode>`.

Flight Mode List
================

.. toctree::
    :maxdepth: 1

    ACRO Mode <acro-mode>
    AUTO Mode <auto-mode>
    AUTOTUNE Mode <autotune-mode>
    CRUISE Mode <cruise-mode>
    CIRCLE Mode <circle-mode>
    FBWA Mode (FLY BY WIRE_A) <fbwa-mode>
    FBWB Mode (FLY BY WIRE_B) <fbwb-mode>
    GUIDED Mode <guided-mode>
    LAND Mode <land-mode>
    LOITER Mode <loiter-mode>
    MANUAL Mode <manual-mode>
    RTL Mode (Return To Launch) <rtl-mode>
    STABILIZE Mode <stabilize-mode>
    TAKEOFF Mode <takeoff-mode>
    TRAINING Mode <training-mode>
