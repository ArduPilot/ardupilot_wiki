.. _radio-failsafe:

==============
Radio Failsafe
==============

Copter supports Return-To-Launch in cases where contact between the
Pilot's RC transmitter and the flight controller's receiver is lost. 
This page explains this failsafe's setup and testing.  Note the "Radio
failsafe" was previously called "Throttle failsafe" because of the way
in which some receivers use the throttle channel to signal the loss of
contact.

.. image:: ../images/RadioFailsafe_Intro.jpg
    :target: ../_images/RadioFailsafe_Intro.jpg

.. note::

   Copter also supports :ref:`Battery <failsafe-battery>`, :ref:`Ground Station <gcs-failsafe>` and :ref:`EKF/DCM failsafes <ekf-inav-failsafe>`.

When the failsafe will trigger
==============================

If enabled and set-up correctly the radio failsafe will trigger if:

-  The pilot turns off the RC transmitter
-  The vehicle travels outside of RC range (usually at around 500m ~
   700m)
-  The receiver loses power (unlikely)
-  The wires connecting the receiver to the flight controller are broken
   (unlikely).  Note: with APM2 only the ch3 connection between receiver
   and flight controller will trigger the failsafe.

What will happen
================

When a radio failsafe is triggered one of the following will happen:

-  **Nothing** if the vehicle is already disarmed
-  **Motors will be immediately disarmed** if the vehicle is landed OR
   in stabilize or acro mode and the pilot's throttle is at zero
-  **Return-to-Launch (RTL)** if the vehicle has a GPS lock and is more
   than 2 meters from the home position
-  **LAND** if the vehicle has:

   -  no GPS lock OR
   -  is within 2 meters of home OR
   -  the FS_THR_ENABLE parameter is set to "Enabled Always Land"

**Continue with the mission** if the vehicle is in AUTO mode and the
FS_THR_ENABLE parameter is set to "Enabled Continue with Mission in
Auto Mode".

If the failsafe clears (i.e. transmitter and receiver regain contact)
the copter will remain in its current flight mode. It
will **not** automatically return to the flight mode that was active
before the failsafe was triggered. This means that if, for example, the
vehicle was in Loiter when the failsafe occurred and the flight mode was
automatically changed to RTL, even after the transmitter and receiver
regained contact, the vehicle would remain in RTL.  If the pilot wished
to re-take control in Loiter he/she would need to change your flight
mode switch to another position and then back to Loiter.

Receiver and flight controller set-up
=====================================

By default, a newly purchased receiver will be set-up to simply hold all
channels at their last known position when the receiver and transmitter
lose contact.  This is not good because the flight controller has no way
to know that the Pilot has lost control of the vehicle.  Instead the
receiver must be set-up to signal to the flight controller it has lost
contact and there are two ways that it can do this (the method depends
upon the receiver):

-  "Low-Throttle" method - the receiver pulls the throttle channel
   (normally channel 3) to a value below the bottom of it's normal range
   (normally below 975).  This method is used by Futaba systems and many
   older systems.
-  "No Signal" method - the receiver stops sending signals to the flight
   controller.  This is the preferred method and is how most modern
   FrSky receivers operate.

Each brand of Transmitter/Receiver is slightly different so please refer
to your transmitter's user manual to determine which method is available
and how to set it up.

Set-up for low-throttle method
------------------------------

..  youtube:: qf8YinLKQww
    :width: 100%

Above is the setup method for a Futaba T7C Transmitter with R617FS or
TFR4-B receiver which uses the "low throttle" method.

With the LiPo battery disconnected:

-  Connect your flight controller to the mission planner and select
   Initial Setup >> Mandatory Hardware >> Failsafe.
-  Set the Failsafe Options to one of the three options:

   -  "Enabled always RTL" to force the vehicle to always RTL even if
      flying a mission in AUTO mode
   -  "Enabled Continue with Mission in AUTO" to allow the vehicle to
      continue with missions even if it takes the vehicle outside of RC
      range (not recommended).  In all other cases the vehicle will RTL.
   -  "Enable always LAND" to force the vehicle to Land immediately if
      it loses RC contact

Set the "FS Pwm" value to be:

-  at least 10 PWM higher than your Channel 3's PWM value when the
   throttle stick is fully down and the transmitter is **off**
-  at least 10 lower than your channel 3's PWM value when the throttle
   stick is fully down and the transmitter is **on**\ 

-  *above 910*

.. image:: ../images/RadioFailsafe_MPSetup.png
    :target: ../_images/RadioFailsafe_MPSetup.png

Set-up for No-Signal method
---------------------------

..  youtube:: FhKREgqjCpM
    :width: 100%

Above is the setup method for a FlySky 9 channel transmitter with FrSky
D4R-II receiver which uses the "No Signal" method.

Similar to the "Low-Throttle" method, with the LiPo battery
disconnected:

-  Connect your flight controller to the mission planner and select
   Initial Setup >> Mandatory Hardware >> Failsafe.
-  Set the Failsafe Options to one of the three options:

   -  "Enabled always RTL" to force the vehicle to always RTL even if
      flying a mission in AUTO mode
   -  "Enabled Continue with Mission in AUTO" to allow the vehicle to
      continue with missions even if it takes the vehicle outside of RC
      range (not recommended).  In all other cases the vehicle will RTL.
   -  "Enable always LAND" to force the vehicle to Land immediately if
      it loses RC contact

Because the throttle is not pulled low, there is normally no need to
adjust the "FS Pwm" value from it's default (975).  Just ensure that
it's well below (i.e. at least 10pwm points below) the minimum value
that channel 3 (throttle) can be.

Testing
=======

You can check your failsafe by performing the following tests with the
Pixhawk/APM connected to the Mission Planner either via a USB cable or
telemetry link. You can complete these tests without plugging in your
LiPo battery but if you do connect a battery you should first remove the
propellers.

**Test #1 : if using the "Low-Throttle" method, ensure the throttle
channel drops with loss of radio contact**

#. Ensure your RC transmitter is on and connected with the throttle all
   the way down and flight mode set to Stabilize
#. The throttle (channel 3) PWM value should be approximately as in
   first illustration below.  It's value may be higher or lower but it
   should definitely be at least 10 higher than the value held in the FS
   PWM field
#. Turn the transmitter off and the throttle PWM value should drop to be
   at least 10 below the FS PWM field value (as in the second
   illustration below) below

.. image:: ../images/MPFailsafeSetup1.jpg
    :target: ../_images/MPFailsafeSetup1.jpg

Test #2 : ensuring motors disarm if in stabilize or acro with throttle
at zero

-  Switch to stabilize mode, arm your motors but keep your throttle at
   zero. Turn off your transmitter. The motors should disarm immediately
   (red led will start flashing, DISARMED will be displayed in the
   Mission Planner's Flight Data screen).

Test #3 : ensuring flight mode changes to RTL or LAND when throttle is
above zero

-  Switch to stabilize mode, arm your motors and raise your throttle to
   the mid point. Turn off your transmitter. The Flight Mode should
   switch to RTL if you have a GPS lock or LAND if you do not have a GPS
   lock (the flight mode and GPS lock status are visible in the Mission
   Planner's flight data screen).

Test #4 : retaking control after the failsafe has cleared

-  continuing on from test #3, turn your transmitter back on
-  while the flight mode is still in RTL or LAND and armed, change the
   flight mode switch to another position and then back to stabilize
   mode.  Ensure that the flight mode displayed on the Failsafe page is
   updating appropriately.

Test #5 (optional) : removing power from the receiver

-  Switch to stabilize mode, arm your motors and keep your throttle
   above zero.
-  Carefully disconnect the power wires connecting the receiver to the
   APM
-  The Flight Mode should switch to RTL or LAND as described in Test #3
-  Warning: unplug the APM so that it is powered down before reattaching
   the receiver's power

Using the receiver to set the flight mode (not recommended)
===========================================================

Instead of setting up the receiver and flight controller as described
above (i.e. "Low-Throttle" and "No Signal" methods) the receiver can be
set-up to set channel 5 (flight mode channel) to a :ref:`flight mode <flight-modes>`\ slot that has
been set to RTL.  For example the receiver could be setup to move ch5's
pwm value to 1700 which is "Flight Mode 5" which could then be set to
RTL on the Mission Planner's Initial Setup >> Mandatory Hardware >>
Flight Modes screen.

Although this mostly works it is not recommended because it will not
trigger if the receiver loses power or if the wires between the receiver
and flight controller are broken.

Warning to Copter 3.1.5 and FRSky receiver users and users of other receivers that modify channel 5 during a failsafe event
===========================================================================================================================

Some FRSky tx/rx systems receivers can only be set-up to modify all
channels including the flight mode channel (channel 5) when a failsafe
event occurs.  For these receivers if using Copter 3.1.5 (or earlier) it
is important to setup the receiver's channel 5 failsafe value so that
the Pixhawk/APM is switched into RTL, Loiter or LAND.  This is critical
because there is a very short period of time (3/50ths of a second)
between when the receiver pulls the throttle low and when the
Pixhawk/APM initiates the RTL or LAND.  During this time, if the
receiver also switches the flight mode channel to stabilize or acro the
Pixhawk/APM may switch to stabilize momentarily and then because the
copter is in stabilize with throttle at zero it will disarm the copter
(i.e. Test #2).
