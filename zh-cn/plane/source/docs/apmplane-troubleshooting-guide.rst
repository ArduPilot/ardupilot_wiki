.. _apmplane-troubleshooting-guide:

=============================
Troubleshooting Guide (Plane)
=============================

I can't connect with the Mission Planner with the 3DR Radios
============================================================

Remember that you cannot use wireless telemetry with APM 2 while the USB
cable is plugged in (they use the same port). Make sure it's unplugged.

Check that you've switching the Mission Planner to the COM port assigned
to your 3DR radio that's connected to your PC and set the baud rate to
57600.

My ESCs won't arm (motor won't turn) with APM 2.x in manual mode
================================================================

Although APM 2.x works well with almost every ESC we've tested, there
are countless variations out there and a few of them (some of the
cheaper Chinese ones, in particular) don't handle the standard APM
startup well. If yours will not arm (you don't get the beeps when you
plug in your battery, and your throttle stick won't spin the motor in
any APM mode), you may have one of those misbehaving ones. We're
constantly improving the code to minimize that issue, but as a temporary
work-around, you can do the following, which allows APM to boot up
before the ESCs:

#. With the LiPo unplugged, power APM with the USB cable.
#. Once it's booted up, plug in the LiPo
#. You can now unplug the USB cable.

My board isn't reading the radio inputs
=======================================

#. Are you powering the RC pins with an ESC or other 5v source? (You
   must). If your RC receiver's power LED isn't on, that means the RC
   rail isn't getting power.
#. Are you sure the RC connectors are plugged in the right way? Signal
   wires at the top, ground at the bottom, closest to the board
#. Is your transmitter on?
#. Plug a servo into one of your receiver's spare channels and make sure
   it's working.
#. We've had reports of people having trouble with Spektrum radios,
   which have a funky binding process. This is what works: To bind the
   receiver and telemetry (TM1000+AR8000 to DX8) module, connect them
   both together(TM1000 to AR8000). With the transmitter off (DX8 off),
   press the tiny button on the side of the telemetry module (TM1000)
   and power the receiver (plug the lipo batteries but do not plug the
   4-wire connection). Both the receiver and telemetry module will start
   to blink (if they do not blink TM1000 is no good). When that happens
   turn on your radio holding the trainer/bind button and it will bind
   (stay at least 10ft away). Make sure that when it binds it says
   something like "binding dsmx ------ receiver with telemetry". You may
   have to do this twice if it doesn't pick up the telemetry module the
   first time.
