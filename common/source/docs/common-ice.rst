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

- Gas motor and ignition module
- RC Switch for controlling power to ignition module like `this one from milehighrc.com <http://milehighrc.com/switch.html>`__
- Optionally an electric starter like `this one from milehighrc.com <http://milehighrc.com/EME_E_Start.html>`__

Connection and Configuration
----------------------------


The engine's ignition power switch and (optional) starter should be connected to the autopilot's servo outputs, ``Ignition`` and ``Starter`` (see ICE section in :ref:`common-rcoutput-mapping`).

[site wiki="plane"]
- Set :ref:`ICE_ENABLE <ICE_ENABLE>` = 1 to enable the ICE feature (you may need to reload parameters after setting this in order to see below parameters)
- Set :ref:`ICE_START_CHAN <ICE_START_CHAN>` to the channel number corresponding to a switch on the transmitter which will be used to start the engine

These parameters may also need to be adjusted:

- Set :ref:`ICE_PWM_STRT_ON <ICE_PWM_STRT_ON>` is the PWM value sent to the starter to start the engine
- Set :ref:`ICE_STARTER_TIME <ICE_STARTER_TIME>` is the time (in seconds) that the starter motor should run to start the engine
- Set :ref:`ICE_PWM_IGN_ON <ICE_PWM_IGN_ON>` is the PWM value sent to the ignition power switch when the engine should be running
- Set :ref:`ICE_PWM_IGN_OFF <ICE_PWM_IGN_OFF>` is the PWM value sent to the ignition power switch when the engine should be stopped

If using an on-board starter, it is important to configure an RPM sensor for the engine. This will allow the ArduPilot to detect an in-flight engine failure and attempt to restart the engine. ArduPilot supports generic pulse tachometers connected to GPIO pins (such as the Aux servo pins on a PixHawk or Cube). A tachometer may be made using a simple hall effect switch IC. Alternatively, some ignition modules support a tachometer output that can be connected directly to the GPIO pins. Desert Aircraft Electronic Ignition V2 modules support tachometer output on the signal pin of the power input connector. Note that when connecting an RPM sensor to an AUX pin, it is important to make sure that this pin is not configured to output a PWM value. On a PixHawk or Cube, Aux 5 and 6 are configured to not output PWM by default. If you need to use a different Aux pin you may need to adjust :ref:`BRD_PWM_COUNT <BRD_PWM_COUNT>`.

In order to configure an RPM sensor, the following parameters must be set:

- Set :ref:`RPM_TYPE <RPM_TYPE>` to 2 for a standard GPIO input pin. 
- Set :ref:`RPM_PIN <RPM_PIN>` to the appropriate value for the auxiliary pin you are using.
- Set the remaining 'RPM_*' parameters as appropriate for your system.
- Set :ref:`ICE_RPM_CHAN <ICE_RPM_CHAN>` to 1.

Throttle control for an ICE engine is similar to controlling a standard brushless ESC. The throttle servo can be connected to any servo output with a ``SERVOx_FUNCTION`` set to 70 (Channel 3 is configured this way by default). It is important to set the Min and Max PWM values on this servo output to be within the mechanical limits of your throttle assembly (using ``SERVOx_MIN`` and ``SERVOx_MAX``). While doing this, also verify that the servo moves in the correct direction with respect to a manual throttle input. Note that the throttle servo will not move unless the vehicle is armed. It is recommended to arm the vehicle with ignition power disconnected in order to test the throttle servo.

After configuring the limits of your throttle servo, the following parameters must be set:

- Set :ref:`THR_MIN <THR_MIN>` to the desired setting at idle. This will be found empirically during engine testing.
- Set :ref:`THR_SLEWRATE <THR_SLEWRATE>` to a value appropriate for your engine. 20%/s is a good starting point.
- Set :ref:`THR_MAX <THR_MAX>` if you would like to prevent your engine from reaching full throttle.

If you are using a quadplane and would like the ICE engine to be disabled during a VTOL descent to reduce the risk of prop strikes, set :ref:`Q_LAND_ICE_CUT <Q_LAND_ICE_CUT>` to 1.

Advanced Starter Configuration
------------------------------
A variety of parameters are available for configuring the engine start routine. The auto-start functionality will attempt to start the engine any time the vehicle is armed, the engine is enabled, and the measured RPM is below the :ref:`ICE_RPM_THRESH <ICE_RPM_THRESH>`. If the engine is not successfully started within a configurable amount of time, the program will wait for a configurable delay before attempting to start again. It is important to remember that the starter will run in pulses. DO NOT approach the engine between failed start attempts as the starter will attempt to start again if the engine is still enabled.

- :ref:`ICE_START_PCT <ICE_START_PCT>` overrides the throttle setting during start. 
- :ref:`ICE_START_TIME <ICE_STARTER_TIME>` controls the maximum amount of time the starter will run in each start attempt.
- :ref:`ICE_START_DELAY <ICE_START_DELAY>` sets a delay between start attempts. This can be useful when your starter has a limited duty cycle.
- :ref:`ICE_RPM_THRESH <ICE_RPM_THRESH>` sets the minimum RPM reading for the engine to be considered running. This should be set to a value below your idle RPM.


[/site]
[site wiki="copter,rover"]
To allow the pilot to directly control the ignition and (optional) starter from the transmitter, RC pass through should be set-up:

- Set ``SERVOx_FUNCTION`` (where "x" is the autopilot's output channel connected to the ignition or starter) to ``RCINy`` (where "y" is the transmitter channel).  For example set :ref:`SERVO8_FUNCTION <SERVO8_FUNCTION>` = 59/"RCIN9" to allow the transmitter's channel 9 to control the autopilot Output 8

Be sure to check the engine's behavior when the transmitter is turned off to simulate what will happen during an RC failsafe.
[/site]

[site wiki="plane"]
Starting and Stopping the Motor
-------------------------------

When using the ArduPilot ICE library to control an engine, the engine can be enabled or disabled using an RC switch, via MAVLink command, or via a mission command. Typically, a 3 position switch on an RC transmitter is used to control the engine. This switch has the following three positions:

- Low: Force the engine to be disabled. This ignores MAVLink commands and mission items that attempt to control the engine's state.
- Mid: Keep the current state of the engine, but allow MAVLink commands and mission items to change the state of the engine.
- High: Force the engine to be enabled. This ignores MAVLink commands and mission items that attempt to control the engine's state.

If an :ref:`ICE_START_CHAN <ICE_START_CHAN>` is not configured, the behavior will be the same as when the switch is in the middle position.

To start the motor with RC control:

- Arm the vehicle
- Raise the transmitter's starter switch to start the motor
- If the engine does not include a starter motor, use a hand-held starter motor to start it

To stop the motor with RC control:

- Lower the transmitter's starter switch
- Disarm the vehicle

Quadplanes can also automatically stop the engine once the final phase of a VTOL landing is entered by setting the :ref:`Q_LAND_ICE_CUT<Q_LAND_ICE_CUT>` parameter to 1.

[/site]

.. note:: The engine can be started and stopped in autonomous missions by using the ``DO_ENGINE_CONTROL`` mission command. This useful preceding NAV_VTOL_TAKEOFF or NAV_VTOL_LAND to start or stop the engine to prevent prop strikes while landing in wind with a low clearance prop. See :ref:`common-mavlink-mission-command-messages-mav_cmd` section. 

Vibration isolation
-------------------

The high vibration from internal combustion engines means that :ref:`vibration dampening <common-vibration-damping>` is critical.  It is often necessary to mount the autopilot on an plate with `self adhesive lead weights <https://www.amazon.com/Great-Planes-Segmented-Weights-6-Ounce/dp/B0015KLJE0>`__ added to increase its mass.
