.. _arming_the_motors:

=================
Arming the motors
=================

Arming the vehicle allows the thrusters to start spinning.  Before arming, make sure all people, objects, and any body parts (e.g., hands) are clear of the thruster propellers.

In order to arm:

- Pre-Arm safety checks must be passing (See :ref:`common-prearm-safety-checks`). It is recommended that :ref:`ARMING_CHECK<ARMING_CHECK>` be enabled for all available systems/sensors checks to prevent arming with system issues present. You must have also provided a method to Disarm the vehicle, either by GCS button assignments (``BTNx_FUNCTION``), or :ref:`AUX_FUNCTION <common-auxiliary-function>`.
- The throttle must be at idle value if :ref:`RC_OPTIONS<RC_OPTIONS>` bit 5 is set ("Arming check throttle for 0 input"). Normally, this would be a center stick if bi-directional vertical thrusters are used.
- An arming pilot control must be activated, eg. GCS joystick button with "Arm" function assigned, GCS MAVLink ARM command given, or an :ref:`Auxiliary function <common-auxiliary-functions>` such as ARM/DISARM ("153") is used by either GCS command or RC channel switch (if enabled).

Disarming the motors
====================

Disarming the motors will prevent the motors from spinning. The motors are disarmed when:

- GCS joystick button with a "Disarm" function assigned is pressed.
- A MAVLink DISARM command is received
- An :ref:`Auxiliary function <common-auxiliary-functions>` such as ARM/DISARM ("153") is used by either GCS command or RC channel switch (if enabled).
.. note:: the AUX Function "31" (Motor Emergency Stop) or "165" (Arm/Motor Emergency Stop) can also be used to prevent motor operations, but do not "Disarm" which would reset the home location and require the pre-arming checks to be passed before re-arming.

GCS Status Messages
===================
Unless the :ref:`ARMING_OPTIONS<ARMING_OPTIONS>` bit 2 is set, text messages will be sent to the GCS to indicate when arming or disarming has occurred.

