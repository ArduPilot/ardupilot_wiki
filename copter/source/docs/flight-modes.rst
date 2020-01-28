.. _flight-modes:

============
Flight Modes
============

This article provides an overview and links to the available flight modes
for Copter.

Overview
========

Copter has 23 flight built-in flight modes, 10 of which are regularly
used. There are modes to support different levels/types of flight
stabilization, a sophisticated autopilot, a follow-me system etc.

Flight modes are controlled through the radio (via a :ref:`transmitter switch <common-rc-transmitter-flight-mode-configuration>`),
via mission commands, or using commands from a ground station (GCS) or
companion computer.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Mode</th><th>Alt Ctrl</th><th>Pos Ctrl</th><th>GPS</th><th>Summary</th></tr>
   <tr><td>Acro</td><td>-</td><td>-</td><td></td><td>Holds attitude, no self-level</td></tr>
   <tr><td>Airmode</td><td>-</td><td>-/+</td><td></td><td>Actually not a mode, but a feature,see below</td></tr>
   <tr><td>Alt Hold</td><td>s</td><td>+</td><td></td><td>Holds altitude and self-levels the roll & pitch</td></tr>
   <tr><td>Auto</td><td>A</td><td>A</td><td>Y</td><td>Executes pre-defined mission</td></tr>
   <tr><td>AutoTune</td><td>s</td><td>A</td><td>Y</td><td>Automated pitch and bank procedure to improve control loops</td></tr>
   <tr><td>Brake</td><td>s</td><td>A</td><td>Y</td><td>Brings copter to an immediate stop</td></tr>
   <tr><td>Circle</td><td>s</td><td>A</td><td>Y</td><td>Automatically circles a point in front of the vehicle</td></tr>
   <tr><td>Drift</td><td>-</td><td>+</td><td>Y</td><td>Like stabilize, but coordinates yaw with roll like a plane</td></tr>
   <tr><td>Flip</td><td>A</td><td>A</td><td></td><td>Rises and completes an automated flip</td></tr>
   <tr><td>FlowHold</td><td>s</td><td>A</td><td></td><td>Position control using Optical Flow</td></tr>
   <tr><td>Follow</td><td>s</td><td>A</td><td>Y</td><td>Follows another vehicle</td></tr>
   <tr><td>Guided</td><td>A</td><td>A</td><td>Y</td><td>Navigates to single points commanded by GCS</td></tr>
    <tr><td>Heli_Autorotate</td><td>A</td><td>A</td><td>Y</td><td>Used for emergencies in traditional helicopters. Helicopter only.  Currently SITL only.</td></tr>
   <tr><td>Land</td><td>A</td><td>s</td><td>(Y)</td><td>Reduces altitude to ground level, attempts to go straight down</td></tr>
   <tr><td>Loiter</td><td>s</td><td>s</td><td>Y</td><td>Holds altitude and position, uses GPS for movements</td></tr>
   <tr><td>PosHold</td><td>s</td><td>+</td><td>Y</td><td>Like loiter, but manual roll and pitch when sticks not centered</td></tr>
   <tr><td>RTL</td><td>A</td><td>A</td><td>Y</td><td>Retruns above takeoff location, may aslo include landing</td></tr>
   <tr><td>Simple/Super Simple</td><td></td><td></td><td>Y</td><td>An add-on to flight modes to use pilot's view instead of yaw orientation</td></tr>
   <tr><td>SmartRTL</td><td>A</td><td>A</td><td>Y</td><td>RTL, but traces path to get home</td></tr>
   <tr><td>Sport</td><td>s</td><td>s</td><td></td><td>Alt-hold, but holds pitch & roll when sticks centered</td></tr>
   <tr><td>Stabilize</td><td>-</td><td>+</td><td></td><td>Self-levels the roll and pitch axis</td></tr>
   <tr><td>SysID</td><td>-</td><td>+</td><td></td><td>Special diagnostic/modeling mode</td></tr>
   <tr><td>Throw</td><td>A</td><td>A</td><td>Y</td><td>Holds position after a throwing takeoff</td></tr>
   <tr><td>ZigZag</td><td>A</td><td>A</td><td>Y</td><td>Useful for crop spraying</td></tr>
   </table>


.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Symbol</th><th>Definition</th></tr>
   <tr><td>-</td><td>Manual control</td><tr>
   <tr><td>+</td><td>Manual control with limits & self-level</td><tr>
   <tr><td>s</td><td>Pilot controls climb rate</td></tr>
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
-  :ref:`AirMode <airmode>`
-  :ref:`Heli_Autorotate <traditional-helicopter-autorotation-mode>` for traditional helicopters only.
-  :ref:`AutoTune <autotune>`
-  :ref:`Brake <brake-mode>`
-  :ref:`Circle <circle-mode>`
-  :ref:`Drift <drift-mode>`
-  :ref:`Flip <flip-mode>`
-  :ref:`FlowHold <flowhold-mode>`
-  :ref:`Follow <follow-mode>`
-  :ref:`Guided <ac2_guidedmode>` (and :ref:`Guided_NoGPS <guided_nogps>`)
-  :ref:`Land <land-mode>`
-  :ref:`PosHold <poshold-mode>`
-  :ref:`Sport <sport-mode>`
-  :ref:`Throw <throw-mode>`
-  :ref:`Follow Me <ac2_followme>`
-  :ref:`Simple and Super Simple <simpleandsuper-simple-modes>`
-  :ref:`Smart RTL (Return-to-Launch) <smartrtl-mode>`
-  :ref:`SysID (System Identificaton) <systemid-mode>`
-  :ref:`ZigZag <zigzag-mode>`
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

-  :ref:`Auto <auto-mode>`
-  :ref:`Heli_Autorotate <traditional-helicopter-autorotation-mode>`
-  :ref:`Circle <circle-mode>`
-  :ref:`Drift <drift-mode>`
-  :ref:`Follow <follow-mode>`
-  :ref:`Follow Me <ac2_followme>`
-  :ref:`Guided <ac2_guidedmode>`
-  :ref:`Loiter <loiter-mode>`
-  :ref:`PosHold <poshold-mode>`
-  :ref:`RTL (Return-to-Launch) <rtl-mode>`
-  :ref:`Smart RTL (Return-to-Launch) <smartrtl-mode>`
-  :ref:`Throw <throw-mode>`
-  :ref:`ZigZag <zigzag-mode>`

Do not require GPS lock:

-  :ref:`Acro <acro-mode>`
-  :ref:`AirMode<airmode>`
-  :ref:`Alt Hold <altholdmode>`
-  :ref:`Stabilize <stabilize-mode>`
-  :ref:`Sport <sport-mode>`
-  :ref:`SysID <systemid-mode>`
-  :ref:`Land <land-mode>`

Full list of flight modes
=========================

.. toctree::
    :maxdepth: 1

    Acro <acro-mode>
    Altitude Hold <altholdmode>
    AirMode<airmode>
    Auto <auto-mode>
    Brake <brake-mode>
    Circle <circle-mode>
    Drift <drift-mode>
    Flip <flip-mode>
    FlowHold <flowhold-mode>
    Follow <follow-mode>
    Follow Me (GSC Enabled) <ac2_followme>
    Guided <ac2_guidedmode>
    Heli_Autorotate <traditional-helicopter-autorotation-mode>
    Land <land-mode>
    Loiter <loiter-mode>
    PosHold <poshold-mode>
    Position <ac2_positionmode>
    RTL <rtl-mode>
    Simple and Super Simple <simpleandsuper-simple-modes>
    Smart RTL (Return-to-Launch) <smartrtl-mode>
    Sport <sport-mode>
    Stabilize <stabilize-mode>
    System Identification <systemid-mode>
    Throw <throw-mode>
    ZigZag <zigzag-mode>
