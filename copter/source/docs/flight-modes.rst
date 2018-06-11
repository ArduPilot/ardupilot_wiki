.. _flight-modes:

============
Flight Modes
============

This article provides an overview and links to the available flight modes
for Copter.

Overview
========

Copter has 20 flight built-in flight modes, 10 of which are regularly
used. There are modes to support different levels/types of flight
stabilization, a sophisticated autopilot, a follow-me system etc.

Flight modes are controlled through the radio (via a :ref:`transmitter switch <common-rc-transmitter-flight-mode-configuration>`),
via mission commands, or using commands from a ground station (GCS) or
companion computer.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Mode</th><th>Alt Ctrl</th><th>Pos Ctrl</th><th>GPS</th><th>Summary</th></tr>
   <tr><td>XXXXX</td><td>XXXXX</td><td>XXXXX</td><td>XXXXX</td><td>XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX</td></tr>
   <tr><td>Stabilize</td><td>-</td><td>+</td><td></td><td>Self-levels the roll and pitch axis</td></tr>
   <tr><td>Alt Hold</td><td>s</td><td>+</td><td></td><td>Holds altitude and self-levels the roll & pitch</td></tr>
   <tr><td>Loiter</td><td>s</td><td>s</td><td>Y</td><td>Holds altitude and position, uses GPS for movements</td></tr>
   <tr><td>RTL</td><td>A</td><td>A</td><td>Y</td><td>Retruns above takeoff location, may aslo include landing</td></tr>
   <tr><td>AUTO</td><td>A</td><td>A</td><td>Y</td><td>Executes pre-defined mission</td></tr>
   <tr><td>Acro</td><td>-</td><td>-</td><td></td><td>Holds attitude, no self-level</td></tr>
   <tr><td>Autotune</td><td>s</td><td>A</td><td>Y</td><td>Automated pitch and bank procedure to improve control loops</td></tr>
   <tr><td>Brake</td><td>s</td><td>A</td><td>Y</td><td>Brings copter to an immediate stop</td></tr>
   <tr><td>Circle</td><td>s</td><td>A</td><td>Y</td><td>Automatically circles a point in front of the vehicle</td></tr>
   <tr><td>Drift</td><td>-</td><td>+</td><td>Y</td><td>Like stabilize, but coordinates yaw with roll like a plane</td></tr>
   <tr><td>Flip</td><td>A</td><td>A</td><td></td><td>Rises and completes an automated flip</td></tr>
   <tr><td>Guided</td><td>A</td><td>A</td><td>Y</td><td>Navigates to single points commanded by GCS</td></tr>
   <tr><td>Land</td><td>A</td><td>s</td><td>(Y)</td><td>Reduces altitude to ground level, attempts to go straight down</td></tr>
   <tr><td>PosHold</td><td>s</td><td>+</td><td>Y</td><td>Like loiter, but manual roll and pitch when sticks not centered</td></tr>
   <tr><td>Sport</td><td>s</td><td>s</td><td></td><td>Alt-hold, but holds pitch & roll when sticks centered</td></tr>
   <tr><td>Throw</td><td>A</td><td>A</td><td>Y</td><td>Holds position after a throwing takeoff</td></tr>
   <tr><td>Follow Me</td><td>s</td><td>A</td><td>Y</td><td>Follows another GPS on the ground</td></tr>
   <tr><td>Simple/Super Simple</td><td></td><td></td><td>Y</td><td>An add-on to flight modes to use pilot's view instead of yaw orientation</td></tr>
   <tr><td>RTL</td><td>A</td><td>A</td><td>Y</td><td>RTL, but traces path to get home</td></tr>
   </table>


.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Symbol</th><th>Definition</th></tr>
   <tr><td>-</td><td>Manual control</td><tr>
   <tr><td>+</td><td>Maunal control with limits & self-level</td><tr>
   <tr><td>s</td><td>Automated stabilized control</td></tr>
   <tr><td>A</td><td>Automatic control</td></tr>
   </table>


Recommended Flight Modes
========================

In general when first starting to use Copter you should progress through
the flight modes in the order listed below, being sure that you are
comfortable with each before progressing to the next (click the links
for more details):

-  :ref:`Stabilize <stabilize-mode>`
-  :ref:`Alt Hold <altholdmode>`
-  :ref:`Loiter <loiter-mode>`
-  :ref:`RTL (Return-to-Launch) <rtl-mode>`
-  :ref:`Auto <auto-mode>`

Additional flight modes:

-  :ref:`Acro <acro-mode>`
-  :ref:`AutoTune <autotune>`
-  :ref:`Brake <brake-mode>`
-  :ref:`Circle <circle-mode>`
-  :ref:`Drift <drift-mode>`
-  :ref:`Flip <flip-mode>`
-  :ref:`Guided <ac2_guidedmode>` (and :ref:`Guided_NoGPS <guided_nogps>`)
-  :ref:`Land <land-mode>`
-  :ref:`PosHold <poshold-mode>`
-  :ref:`Sport <sport-mode>`
-  :ref:`Throw <throw-mode>`
-  :ref:`Follow Me <ac2_followme>`
-  :ref:`Simple and Super Simple <simpleandsuper-simple-modes>`
-  :ref:`Smart RTL (Return-to-Launch) <smartrtl-mode>`
-  :ref:`Avoid_ADSB <common-ads-b-receiver>` for ADS-B based avoidance of manned aircraft.  Should not be set-up as a pilot selectable flight mode.

Most transmitters provide a 3 position switch but you can find
instructions :ref:`here for setting up a 6-position flight mode switch <common-rc-transmitter-flight-mode-configuration>`.

GPS Dependency
==============

Flight modes that use GPS-positioning data require an active GPS lock
prior to takeoff. To see if your autopilot has acquired GPS lock,
connect to a ground station or consult your autopilot's hardware
overview page to see the LED indication for GPS lock. Below is a summary
of GPS dependency for Copter flight modes.

Requires GPS lock prior to takeoff:

-  :ref:`Loiter <loiter-mode>`
-  :ref:`RTL (Return-to-Launch) <rtl-mode>`
-  :ref:`Auto <auto-mode>`
-  :ref:`Guided <ac2_guidedmode>`
-  :ref:`Drift <drift-mode>`
-  :ref:`PosHold <poshold-mode>`
-  :ref:`Follow Me <ac2_followme>`
-  :ref:`Circle <circle-mode>`
-  :ref:`Smart RTL (Return-to-Launch) <smartrtl-mode>`
-  :ref:`Throw <throw-mode>`

Do not require GPS lock:

-  :ref:`Stabilize <stabilize-mode>`
-  :ref:`Alt Hold <altholdmode>`
-  :ref:`Acro <acro-mode>`
-  :ref:`Sport <sport-mode>`
-  :ref:`Land <land-mode>`

Full list of flight modes
=========================

.. toctree::
    :maxdepth: 1

    Acro Mode <acro-mode>
    Altitude Hold Mode <altholdmode>
    Auto Mode <auto-mode>
    Brake Mode <brake-mode>
    Circle Mode <circle-mode>
    Drift Mode <drift-mode>
    Follow Me Mode (GSC Enabled) <ac2_followme>
    Flip Mode <flip-mode>
    Guided Mode <ac2_guidedmode>
    Land Mode <land-mode>
    Loiter Mode <loiter-mode>
    PosHold Mode <poshold-mode>
    Position Mode <ac2_positionmode>
    RTL Mode <rtl-mode>
    Simple and Super Simple Modes <simpleandsuper-simple-modes>
    Smart RTL (Return-to-Launch) <smartrtl-mode>
    Sport Mode <sport-mode>
    Stabilize Mode <stabilize-mode>
    Throw Mode <throw-mode>
