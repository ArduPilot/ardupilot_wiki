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

Flybarless electric RC helicopter frame
=======================================

At this time, only conventional single-rotor collective pitch helicopter
types are supported such as the
`Trex450 <http://shop.align.com.tw/product_info.php?products_id=4089>`__.

The program can be configured to fly flybarred, flybarless, with CCPM
swash mixing or single-servo (H1) swash types. Many different commonly
available electric helicopters have been flown, from a 450 size, all the
way up to 700 size.  The system has not yet been proven to work on fuel
powered helis, but it should be possible assuming adequate vibration
damping is provided for the APM/Pixhawk.

8-channel Transmitter / Receiver
================================

Your transmitter & receiver should ideally support 8 channels (elevator,
aileron, collective pitch, rudder, flight mode, tuning knob, auxiliary
function switch, main rotor speed) although it's possible to fly with as
few as 5.

A discussion of which frequencies are legal to transmit on in which
region is outside the scope of this wiki.

ESC with built in governor
==========================

If you intend to use any Copter flight control modes other than Acro and
Stabilize, it is recommended than the speed controller you purchase
should have a governor mode. This is because Copter will be controlling
the pitch of the main blades automatically, but does not have an
internal throttle control system. As such, when AC is commanding high
rates of blade pitch, the rotor disk will slow if increased throttle is
not be applied. A governor mode in the ESC can be employed to
automatically compensate for this. The radio throttle curve and ESC
governor can be set up so that the rotor speed will be held at an
appropriate speed in all cases.

Digital servos
==============

You should use digital servos instead of analog. There are a number of
important reasons for this. First and foremost is because analog servos
do not center nearly as well as digital servos. They will always stop on
either side of true center, depending from which side they approach
center. This phenomenon will cause the PID control system used in Copter
to struggle to accurately control the helicopter. Furthermore, digital
servos respond much faster to small input changes than analog servos do.
This effect is over and above specified transit speed of the servos
which is measured for a 60° sweep. If two servos, analog and digital
both have the same specified transit time (eg: 0.20sec/60°), and both
are asked to move only 5°, the digital servo will move faster than the
analog. This also has an effect on how well AC can control the
helicopter. Similarly, faster servos will also benefit AC control. The
only downside to fast digital servos, besides cost, is that they will
naturally have a higher power requirement. This means you will need a
larger BEC, and more battery capacity if using a seperate radio system
battery. Make sure that your power supply system is capable of providing
the power they need.

Batteries
=========

There are no special requirements for flight batteries. However, as a
general rule, it is safer if you can have a separate motor battery and
radio battery. Not only does this help prevent radio and Copter power
loss when the motor battery drains, but also because disconnecting the
power to the motor is one easy method to be absolutely sure that your
motor will not turn while you are configuring the Copter system.
