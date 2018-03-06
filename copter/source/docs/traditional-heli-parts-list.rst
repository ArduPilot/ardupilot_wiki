.. _traditional-heli-parts-list:

=============================================
Traditional Helicopter – Suggested Parts List
=============================================

Electronics including flight controller
=======================================

-  :ref:`Pixhawk2 <common-pixhawk2-overview>`
   with :ref:`GPS and external compass <common-positioning-landing-page>`.
-  Extra long GPS cable
-  Extra long compass (i2c) cable
-  Flight controller mounting plate

RC helicopter frame
===================

ArduPilot supports single rotor with conventional or direct drive tail, tandem
rotor, or collective pitch quad-rotor helicopter frames.

The program can be configured to fly flybar or flybarless, with CCPM
swash mixing or single-servo (H1) swash types. The system has flown helicopters
ranging from micro size with a Pixracer to large piston and turbine powered
machines.

8-channel Transmitter / Receiver
================================

Your transmitter & receiver must support minimum 8 channels (elevator,
aileron, collective pitch, rudder, flight mode, tuning knob, auxiliary
function switch, throttle hold).

Use of electric and engine governors
=====================================

If you intend to use any Copter flight control modes other than Acro and
Stabilize, it is recommended that the speed controller you purchase for an
electric helicopter have a governor mode. For piston and turbine engines a
governor is required. This is because Copter will be controlling the pitch of
the main blades automatically along with its internal throttle control system.
The throttle curve in ArduPilot will be used for fallback and feedforward for
piston and turbine engines in the event the governor fails in flight, and for
faster governor response on high collective pitch loading.

For electric helicopters ArduPilot will set the throttle for the ESC governor to
hold the headspeed constant at all times. 

Digital servos
==============

You should use digital servos instead of analog. Digital servos have much faster
response time, more accurate positioning, and your helicopter will fly much more
accurate and stable with digital servos.

Batteries
=========

There are no special requirements for flight batteries. However, as a
general rule, it is safer if you can have a separate motor battery and
radio battery on electric helicopters. Electric motor power systems can have
very high power draw in helicopters. Having separate batteries will prevent
"brownout" of power to the flight electronics.
