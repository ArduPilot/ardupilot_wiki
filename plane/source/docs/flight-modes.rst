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
   <tr><th>Mode</th><th>Roll</th><th>Pitch</th><th>Throttle</th><th>GPS</th><th>Summary</th></tr>
   <tr><td>:ref:`MANUAL <manual-mode>`</td><td>-</td><td>-</td><td>-</td><td></td><td>Manual control surface movement</td></tr>
   <tr><td>:ref:`FBWA <fbwa-mode>`</td><td>s</td><td>s</td><td>-</td><td></td><td>Roll and pitch follow stick input, up to set limits</td></tr>
   <tr><td>:ref:`FBWB <fbwb-mode>`</td><td>s</td><td>A</td><td>A</td><td>Y</td><td>like FBWA, but with automatic height and speed control</td></tr>
   <tr><td>:ref:`CRUISE <cruise-mode>`</td><td>A</td><td>A</td><td>A</td><td>Y</td><td>like FBWB, but with ground course tracking and terrain following</td></tr>
   <tr><td>:ref:`STABILIZE <stabilize-mode>`</td><td>+</td><td>+</td><td>-</td><td></td><td>Wing-leveling on stick release</td></tr>
   <tr><td>:ref:`AUTOTUNE <autotune-mode>`</td><td>s</td><td>s</td><td>-</td><td></td><td>like FBWA, but learns attitude tuning while flying</td></tr>
   <tr><td>:ref:`TRAINING <training-mode>`</td><td>+</td><td>+</td><td>-</td><td></td><td>Manual control up to roll and pitch limits</td></tr>
   <tr><td>:ref:`ACRO <acro-mode>`</td><td>+</td><td>+</td><td>-</td><td></td><td>rate controlled mode with no attitude limits</td></tr>
   <tr><td>:ref:`AUTO <auto-mode>`</td><td>A</td><td>A</td><td>A</td><td><Y/td><td>Follows Mission</td></tr>
   <tr><td>:ref:`LOITER <loiter-mode>`</td><td>A</td><td>A</td><td>A</td><td><Y/td><td>Circles point where mode switched</td></tr>
   <tr><td>:ref:`CIRCLE <circle-mode>`</td><td>A</td><td>A</td><td>A</td><td></td><td>Gently turns aircraft</td></tr>
   <tr><td>:ref:`GUIDED <guided-mode>`</td><td>A</td><td>A</td><td>A</td><td></td><td>Circles user defined point from GCS</td></tr>
   <tr><td>:ref:`Return To Launch (RTL) <rtl-mode>`</td><td>A</td><td>A</td><td>A</td><td>Y</td><td>Returns to and circles home or rally point</td></tr>
   </table>
   
   
.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Symbol</th><th>Definition</th></tr>
   <tr><td>-</td><td>Full manual control of flight surfaces</td><tr>
   <tr><td>+</td><td>Manual control with stabilized limits or assistance</td><tr>
   <tr><td>s</td><td>Stabilized control with limits</td></tr>
   <tr><td>A</td><td>Automatic control</td></tr>
   </table>

Mission Specific Modes
======================

When flying an AUTO mission Plane has some sub-modes that are set using
mission items. The two main sub-modes are TAKEOFF and LAND.

:ref:`TAKEOFF <takeoff-mode>`

:ref:`LAND <land-mode>`

.. toctree::
    :maxdepth: 1

    MANUAL Mode <manual-mode>
    STABILIZE Mode <stabilize-mode>
    FBWA Mode (FLY BY WIRE_A) <fbwa-mode>
    FBWB Mode (FLY BY WIRE_B) <fbwb-mode>
    AUTOTUNE Mode <autotune-mode>
    TRAINING Mode <training-mode>
    ACRO Mode <acro-mode>
    CRUISE Mode <cruise-mode>
    AUTO Mode <auto-mode>
    RTL Mode (Return To Launch) <rtl-mode>
    LOITER Mode <loiter-mode>
    CIRCLE Mode <circle-mode>
    GUIDED Mode <guided-mode>
    TAKEOFF Mode <takeoff-mode>
    LAND Mode <land-mode>
