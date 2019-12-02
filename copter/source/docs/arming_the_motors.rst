.. _arming_the_motors:

=================
Arming the motors
=================

Arming the vehicle allows the motors to start spinning.  Before arming,
make sure all people, objects, and any body parts (e.g., hands) are
clear of the propellers. Then do the following:

.. note::

   You can only arm or disarm in Stabilize, ACRO, AltHold, Loiter,
   and PosHold modes.  You cannot arm your copter in AUTO
   mode.

#. Turn on your transmitter.
#. Plug in the LiPo battery.  The red and blue lights should flash for a few seconds as the gyros are calibrated (do not move the copter)
#. The pre-arm checks will run automatically and if any problems are found the RGB LED will blink yellow and the failure will be displayed on the ground station.  Please refer to :ref:`this page <common-prearm-safety-checks>`
#. Check that your flight mode switch is set to Stabilize, ACRO, AltHold, Loiter, or PosHold
#. If using a autopilot with a safety switch, press it until the light goes solid
#. If you are planning on using an autonomous mode (i.e. Loiter, RTL, Auto, etc) swith the vehicle to Loiter or PosHold and wait until the LEDs blink green indicating a good GPS lock
#. Arm the motors by holding the throttle down, and rudder right for 5 seconds.  Do not hold the rudder right for too long (>15 seconds) or you will begin the :ref:`AutoTrim <autotrim>` feature
#. Once armed, the LEDs will go solid and the propellers will begin to spin
#. Raise the throttle to take-off

.. note::

   If you leave the throttle at minimum for 15 seconds while in any
   of the above modes the motors will automatically disarm.

Disarming the motors
====================

Disarming the motors will cause the motors to stop spinning. To disarm the motors do the following:

#. Check that your flight mode switch is set to Stabilize, ACRO, AltHold, Loiter, or PosHold
#. Hold throttle at minimum and rudder to the left for 2 seconds
#. The LED will start flashing indicating the vehicle is disarmed
#. If using a autopilot with a safety switch, press it until the LED begins flashing
#. Disconnect the Lipo battery.
#. Turn off your transmitter.
