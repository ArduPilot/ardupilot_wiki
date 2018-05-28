.. _arming_the_motors:

=================
Arming the motors
=================

Arming the motors causes ArduPilot to apply power to your motors,
which will cause them to start spinning.  Before arming the motors,
make sure all people, objects, and any body parts (e.g., hands) are
clear of the propellers. Then do the following:

.. note::

   You can only arm or disarm in Stabilize, ACRO, AltHold, Loiter,
   and PosHold modes.  You cannot arm your copter in AUTO
   mode.

#. Turn on your transmitter.
#. Plug in your LiPo battery.  The red and blue lights should flash for
   a few seconds as the gyros are calibrated (do not move the copter)
#. The pre-arm checks will run automatically and if any problems are
   found an APM2.x will double blink the red arming light, on a Pixhawk
   the RGB led will blink yellow.  Please refer to :ref:`this page <prearm_safety_check>`.
#. Check that your flight mode switch is set to Stabilize, ACRO, AltHold
   Loiter, or PosHold.
#. If using a PX4, press the safety button until the light goes solid.
#. If you are planning on using the autopilot (i.e. Loiter, RTL, Drift,
   Auto or Guided modes) you should wait for 30 seconds after the GPS
   has gotten 3d lock.  This will give the GPS position time to settle.
   On APM2 the GPS lock is indicated by the blue LED going solid.  On an
   Pixhawk the RGB LED will blink green.
#. Arm the motors by holding the throttle down, and rudder right for 5
   seconds.  It takes approximately 5 seconds the first time the copter
   is armed as it re-initialises the gyros and barometer.  Do not hold
   the rudder right for too long (>15 seconds) or you will begin the
   :ref:`AutoTrim <autotrim>` feature.
#. Once armed, the red arming light should go solid and the propellers
   will begin to spin.
#. Raise the throttle to take-off.

.. note::

   If you leave the throttle at minimum for 15 seconds while in any
   of the above modes the motors will automatically disarm.

Disarming the motors
====================

Disarming the motors will cause the motors to stop spinning. To disarm
the motors do the following:

#. Check that your flight mode switch is set to Stabilize, ACRO, AltHold
   ,Loiter, or PosHold.
#. Hold throttle at minimum and rudder to the left for 2 seconds.
#. The red arming light should start flashing on the APM2.  On the
   Pixhawk the RGB LED will start flashing green.
#. If using a PX4, press the safety button until the led begins flashing
#. Disconnect the Lipo battery.
#. Turn off your transmitter.
