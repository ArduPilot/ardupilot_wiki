.. _common-ice:

=================================
Internal Combustion Engines (ICE)
=================================

..  youtube:: RjjF_S69Ywk
    :width: 100%

Canberra UAV's petrol powered tricopter.  `See blog post here <https://discuss.ardupilot.org/t/petrol-boosted-tricopter/17823>`__

Internal combustion engines can increase flight times but care must be taken to handle the extra complexity and increased vibration.

What to Buy
-----------

- Gas motor with built-in ignition
- Optionally an electric starter like `this one from milehighrc.com <http://milehighrc.com/EME_E_Start.html>`__

Connection and Configuration
----------------------------

The engine's ignition and (optional) starter should be connected to the flight controller's servo outputs

[site wiki="plane"]
- Set :ref:`ICE_ENABLE <ICE_ENABLE>` = 1 to enable the ICE feature (you may need to reload parameters after setting this in order to see below parameters)
- Set :ref:`ICE_START_CHAN <ICE_START_CHAN>` to the channel number corresponding to a switch on the transmitter which will be used to start the engine

These parameters may also need to be adjusted:

- Set :ref:`ICE_PWM_STRT_ON <ICE_PWM_STRT_ON>` is the PWM value sent to the starter to start the engine
- Set :ref:`ICE_STARTER_TIME <ICE_STARTER_TIME>` is the time (in seconds) that the starter motor should run to start the engine
- Set :ref:`ICE_PWM_IGN_ON <ICE_PWM_IGN_ON>` is the PWM value sent to the ignition when the engine should be running
- Set :ref:`ICE_PWM_IGN_OFF <ICE_PWM_IGN_OFF>` is the PWM value sent to the ignition when the engine should be stopped
[/site]
[site wiki="copter,rover"]
To allow the pilot to directly control the ignition and (optional) starter from the transmitter, RC pass through should be set-up:

- Set ``SERVOx_FUNCTION`` (where "x" is the flight controller's output channel connected to the ignition or starter) to ``RCINy`` (where "y" is the transmitter channel).  For example set :ref:`SERVO8_FUNCTION <SERVO8_FUNCTION>` = 59/"RCIN9" to allow the transmitter's channel 9 to control the flight controller Output 8

Be sure to check the engine's behaviour when the transmitter is turned off to simulate what will happen during an RC failsafe.
[/site]

Vibration isolation
-------------------

The high vibration from internal combustion engines means that :ref:`vibration dampening <common-vibration-damping>` is critical.  It is often necessary to mount the flight controller on an plate with `self adhesive lead weights <https://www.amazon.com/Great-Planes-Segmented-Weights-6-Ounce/dp/B0015KLJE0>`__ added to increase its mass.

[site wiki="plane"]
Starting and Stopping the Motor
-------------------------------

To start the motor:

- Arm the vehicle
- Raise the transmitter's starter switch to start the motor
- If the engine does not include a starter motor, use a hand-held starter motor to start it

To stop the motor

- Lower the transmitter's starter switch
- Disarm the vehicle
[/site]