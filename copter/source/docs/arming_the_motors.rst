.. _arming_the_motors:

=================
Arming the motors
=================

Arming the vehicle allows the motors to start spinning.  Before arming,
make sure all people, objects, and any body parts (e.g., hands) are
clear of the propellers. Then do the following:


#. Turn on your transmitter.
#. Plug in the LiPo battery.  The red and blue lights should flash for a few seconds as the gyros are calibrated (do not move the copter)
#. The pre-arm checks will run automatically and if any problems are found the RGB LED will blink yellow and the failure will be displayed on the ground station.  Please refer to :ref:`this page <common-prearm-safety-checks>`
#. Check that your flight mode switch is set to Stabilize, ACRO, AltHold, Loiter, or PosHold
#. If using an autopilot with a safety switch, press it until the light goes solid
#. If you are planning on using an autonomous mode (i.e. Loiter, RTL, Auto, etc) switch the vehicle to Loiter or PosHold and wait until the LEDs blink green indicating a good GPS lock
#. Arm the motors by holding the throttle down, and rudder right for 5 seconds.  Do not hold the rudder right for too long (>15 seconds) or you will begin the :ref:`AutoTrim <autotrim>` feature
#. Once armed, the LEDs will go solid and the propellers will begin to spin
#. Raise the throttle to take-off

.. note::

   If you leave the throttle at minimum for 15 seconds while in any
   of the above modes the motors will automatically disarm.

.. note:: you cannot arm while in certain modes. See the table below:

=========================           =====================
Modes not allowing Arming           Exceptions
=========================           =====================
AUTO                                :ref:`AUTO_OPTIONS<AUTO_OPTIONS>` bit 0 is set (Allow arming in mode)
AUTOTUNE
BRAKE
CIRCLE
FLIP
GUIDED                              via MAVLink DO_ARM/DISARM command, :ref:`common-lua-scripts`, or :ref:`common-auxiliary-functions<common-auxiliary-functions>` switch if enabled with :ref:`GUID_OPTIONS<GUID_OPTIONS>`
LAND
RTL
SMARTRTL
SYSID
AVOIDADSB
FOLLOW
AUTOTUNE
=========================           =====================


.. note:: If setup, you can use one of the **RC_xOPTION switches** that includes the arm function. See switch option "153", "154, or "160".

Disarming the motors
====================

Disarming the motors will cause the motors to stop spinning. To disarm the motors do the following:

#. Check that your flight mode switch is set to Stabilize, ACRO, AltHold, Loiter, or PosHold
#. Hold throttle at minimum and rudder to the left for 2 seconds
#. The LED will start flashing indicating the vehicle is disarmed
#. If using an autopilot with a safety switch, press it until the LED begins flashing
#. Disconnect the Lipo battery.
#. Turn off your transmitter.

.. note:: If setup, you can use one of the ``RC_xOPTION`` switches that includes the disarm function. See switch option "81", "153, or "154".

GCS Status Messages
===================

Unless the :ref:`ARMING_OPTIONS<ARMING_OPTIONS>` bit 2 is set, text messages will be sent to the GCS to indicate when arming or disarming has occurred.

