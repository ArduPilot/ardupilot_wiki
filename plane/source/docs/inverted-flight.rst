.. _inverted-flight:

===============
Inverted Flight
===============

Plane now supports inverted flight. You can tell if your version
supports inverted flight by looking for the
:ref:`INVERTEDFLT_CH <INVERTEDFLT_CH>` EEPROM configuration parameter.

Details
=======

The INVERTEDFLT_CH parameter allows you to configure a transmitter
switch to enable inverted flight with APM. The default is zero, which
means no inverted flight. To enable it, choose a RC input channel which
is not being used for anything else, and setup your transmitter to give
a PWM value larger than 1750 when a previously unused 2 position switch
is enabled, and a value well below 1750 when it is not enabled.

For example, if channel 7 is currently unused, and you set
INVERTEDFLT_CH to 7, then if your transmitter sends 1900 on channel 7
the APM will flip the plane upside down and continue flying in whatever
mode it is in, but with the plane inverted.

Transmitter Setup
=================

Enabling inverted flight only changes how APM stabilises the plane. It
will stabilise it with a roll of 180 degrees from normal whenever
inverted flight is enabled.

To get the most out of inverted flight it can be useful to also invert
your transmitter elevator and rudder controls while your inverted flight
switch is enabled. This should be possible with any good transmitter.
For example, with the ER9X firmware on a Turnigy 9X transmitter you
could use mixes like this:

::

    CH01   +100% AIL
    CH02   +100% ELE
         * -100% FULL Switch(GEA)
    CH03   +100% THR
    CH04   +100% RUD
         * -100% FULL Switch(GEA)
    CH07   +100% MAX  Switch(GEA)

that would setup the elevator and rudder to invert when the GEAR switch
is active, along with setting CH7 to full.

Interaction with flight modes
=============================

Inverted flight can be used with any of the stabilized flight modes
(which means anything but manual mode). That means you can switch on
inverted flight during a mission and the plane will continue the mission
inverted. Make sure you disable it before trying to land!

Tips
====

Here are some general tips about inverted flight

-  Try it out in HIL simulation first to get used to it, and ensure your
   transmitter is setup right.
-  make sure you have plenty of height before engaging or disengaging
   inverted flight. Depending on your airframe, you could lose a lot of
   height while flipping over, and it may be that your airframe
   struggles to maintain height while inverted
