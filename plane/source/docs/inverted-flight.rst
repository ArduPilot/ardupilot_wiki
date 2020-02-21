.. _inverted-flight:

===============
Inverted Flight
===============

Plane supports inverted flight. Inverted flight is initiated by setting an RC channel via RCx_OPTION parameter.

Details
=======

The RCx_OPTION parameter allows you to configure a transmitter switch
to enable inverted flight for fixed wing aircraft. To enable it,
choose a RC input channel which is not being used for anything else,
and setup your transmitter to give a PWM value larger than 1750 when a
previously unused 2 position switch is enabled, and a value below 1200
when it is not enabled.

For example, if channel 7 is currently unused, and you set :ref:`RC7_OPTION<RC7_OPTION>`
to InvertedFlight (which has value 43) then if your transmitter sends
1900 on channel 7 the plane will flip upside down and continue flying
in whatever mode it is in, but with the plane inverted.

Transmitter Setup
=================

Enabling inverted flight only changes how ArduPilot stabilises the
plane. It will stabilise it with a roll of 180 degrees from normal
whenever inverted flight is enabled.

To get the most out of inverted flight it can be useful to also invert
your transmitter elevator and rudder controls while your inverted flight
switch is enabled. This should be possible with any good transmitter.

Interaction with flight modes
=============================

Inverted flight can be used with any of the non-quadplane stabilized
flight modes (which means anything but manual mode). That means you
can switch on inverted flight during a mission and the plane will
continue the mission inverted. Make sure you disable it before trying
to land!

Tips
====

Here are some general tips about inverted flight

- Try it out in simulation first to get used to it, and ensure your
  transmitter is setup right.
- make sure you have plenty of height before engaging or disengaging
  inverted flight. Depending on your airframe, you could lose a lot of
  height while flipping over, and it may be that your airframe
  struggles to maintain height while inverted
- make sure your :ref:`PTCH2SRV_I<PTCH2SRV_I>` value and :ref:`PTCH2SRV_IMAX<PTCH2SRV_IMAX>` is high
  enough. Planes commonly need more pitch trim while inverted to hold
  height.
