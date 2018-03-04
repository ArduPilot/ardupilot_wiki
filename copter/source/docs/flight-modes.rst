.. _flight-modes:

============
Flight Modes
============

This article provides an overview of the available flight modes for
Copter and instructions for how to configure which modes are available
through the RC Transmitter.

Overview
========

Copter has 14 flight built-in flight modes, 10 of which are regularly
used. There are modes to support different levels/types of flight
stabilization, a sophisticated autopilot, a follow-me system etc.

Flight modes are controlled through the radio (via a :ref:`transmitter switch <common-rc-transmitter-flight-mode-configuration>`),
via mission commands, or using commands from a ground station (GCS) or
companion computer.

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
