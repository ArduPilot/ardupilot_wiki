.. _common-ice:
[copywiki destination="plane"]
=================================
Internal Combustion Engines (ICE)
=================================
ArduPilot can manage ignition, starter, and throttle for internal combustion engines (ICE) in planes and quadplanes. It allows engines to be started and stopped (manually, or automatically in a mission), and optional min/max RPM governors. It can also be configured to allow the throttle to be controlled in Manual mode while disarmed, which is useful for ground testing and preflight run-up for quadplanes without needing to arm the vehicle (which risks a failsafe turning on the VTOL motors).

What to Buy
-----------

The only required hardware is an engine (gas, glow, etc.) and a servo to control the throttle.

Optional:

- An RPM sensor. This can be a simple hall-effect sensor connected to a GPIO pin, or an ignition module with a tach output. This is needed for the RPM governors and the auto-restart feature if using an onboard starter.
- Ignition module and an RC Switch for controlling power to it like `this one from milehighrc.com <http://milehighrc.com/switch.html>`__
- An electric starter like `this one from milehighrc.com <http://milehighrc.com/EME_E_Start.html>`__

Setup
-----

Set :ref:`ICE_ENABLE <ICE_ENABLE>` = 1 to enable the ICE feature (you may need to reload parameters after setting this in order to see below parameters).

Throttle Servo
^^^^^^^^^^^^^^
Throttle control for an IC engine is similar to controlling a standard brushless ESC. The throttle servo can be connected to any servo output with a :ref:`SERVOx_FUNCTION<SERVO3_FUNCTION>` set to 70 (Channel 3 is configured this way by default). It is important to set the Min and Max PWM values on this servo output to be within the mechanical limits of your throttle assembly (using :ref:`SERVOx_MIN<SERVO3_MIN>` and :ref:`SERVOx_MAX<SERVO3_MAX>`). While doing this, also verify that the servo moves in the correct direction with respect to manual throttle input.

Note that, by default, the throttle servo will not move unless the vehicle is

1. Armed, and
2. The engine is considered "running" (start command received and RPM above threshold, or no RPM sensor configured).

To test your throttle servo limits safely, you can set :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 2 (Value +4) to allow throttle control while disarmed in Manual mode, and it does not require the engine to be considered "running". If you do not want this option long-term, be sure to disable it after testing.

For setting :ref:`SERVOx_MIN<SERVO3_MIN>`, you can either set it to the PWM value that corresponds to idle throttle, or set it to the PWM value that corresponds to fully closed throttle, allowing you to cut the engine even if you do not have and ignition switch. If you set it to the fully closed throttle, be sure to set :ref:`ICE_IDLE_PCT<ICE_IDLE_PCT>` to an appropriate value so that the engine does not stall after starting.

.. warning:: If you set :ref:`ICE_IDLE_PCT<ICE_IDLE_PCT>` to a value greater than zero, the engine will be allowed to start while disarmed. If you do not want this, set :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 3 (value +8) to prevent starting while disarmed. See `Controlling the Engine`_ below for more information.

After configuring the limits of your throttle servo, the following parameters must be set:

- Set :ref:`THR_MIN <THR_MIN>` to the desired setting at idle. This will be found empirically during engine testing.
- Set :ref:`THR_SLEWRATE <THR_SLEWRATE>` to a value appropriate for your engine. 20%/s is a good starting point.
- Set :ref:`THR_MAX <THR_MAX>` if you would like to prevent your engine from reaching full throttle.

Engine Control States
^^^^^^^^^^^^^^^^^^^^^
The engine control logic has the following states:

=========== ======================================= ======== ======= ============================================================================== ===========
Name        Throttle %                              Ignition Starter Duration/Condition                                                             Next State 
=========== ======================================= ======== ======= ============================================================================== ===========
Off         0 [1]_                                  Off      Off     Until commanded to start                                                       Starting   
Starting    :ref:`ICE_START_PCT<ICE_START_PCT>`     On       On      :ref:`ICE_STARTER_TIME<ICE_STARTER_TIME>` seconds                              Running    
Running     :ref:`ICE_IDLE_PCT<ICE_IDLE_PCT>` (min) On       Off     Until RPM drops below :ref:`ICE_RPM_THRESH<ICE_RPM_THRESH>`                    Start Delay
Start Delay :ref:`ICE_START_PCT<ICE_START_PCT>`     On       Off     Until :ref:`ICE_START_DELAY<ICE_START_DELAY>` seconds since last start attempt Starting   
=========== ======================================= ======== ======= ============================================================================== ===========

When the engine is commanded to stop, it will immediately go to the "Off" state.

.. [1] If enabled via :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 2, the throttle can be moved in Manual mode (only while disarmed)

Controlling the Engine
^^^^^^^^^^^^^^^^^^^^^^
The engine can be commanded in two ways:

- Assigning an RC switch (see `RC Switch Setup`_ below)
- Sending a :ref:`DO_ENGINE_CONTROL<mav_cmd_do_engine_control>` command via MAVLink (which can be done on the Actions tab in Mission Planner)

The engine is also automatically stopped or started in some situations:

- In auto missions, the engine can be started and stopped using the :ref:`DO_ENGINE_CONTROL<mav_cmd_do_engine_control>` mission command.
- On quadplanes, the engine will be automatically disabled during the final stage of a VTOL landing if :ref:`Q_LAND_ICE_CUT<Q_LAND_ICE_CUT>` is set to 1. This is useful to reduce dust ingestion and the risk of prop strikes while landing.
- If :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 0 is set (value +1), the engine will automatically disable during RC failsafe.
- The engine will automatically disable during an emergency stop AUX function, if configured. See :ref:`common-auxiliary-functions`.
- The engine will automatically disable during parachute deployment, if used. See :ref:`common-parachute`.

By default, the engine cannot be started while disarmed. If you want to allow it to start while disarmed, either set :ref:`ICE_IDLE_PCT<ICE_IDLE_PCT>` to a value greater than zero, or set :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 2 (value +4) to allow throttle control while disarmed in Manual mode.

If you would like to use :ref:`ICE_IDLE_PCT<ICE_IDLE_PCT>`, but *don't* want to allow starting while disarmed, set :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 3 (value +8).

.. warning::
    Even with :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 3 set, the engine can still be started while disarmed with a :ref:`DO_ENGINE_CONTROL<mav_cmd_do_engine_control>` command if param4 is set to 1, though in this case it will only start once and won't attempt to restart if it stalls while still disarmed.

Ignition and Starter Outputs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The optional ignition power switch and starter outputs can be each be controlled using a servo output (connected to a PWM switch) or a GPIO pin (connected to a relay, or to the enable pin of an engine control unit). See :ref:`common-rcoutput-mapping` and :ref:`common-relay` for information on how to configure servo outputs as relays for ignition and starter control.

If using PWM outputs,

.. tabs::
   .. tab:: ArduPilot 4.6 and later

      - the PWM value sent to the ignition power switch when the engine should be running or stopped is set by the  :ref:`SERVOx_MIN<SERVO10_MIN>` and :ref:`SERVOx_MAX<SERVO10_MIN>` value of the output used for the `Ignition` function.
      - the PWM value sent to the starter switch when it should be cranking is set by the :ref:`SERVOx_MIN<SERVO10_MIN>` and :ref:`SERVOx_MAX<SERVO10_MIN>` value of the output used for the `Starter` function.

   .. tab:: ArduPilot prior to 4.6

      - ``ICE_PWM_STRT_ON`` is the PWM value sent to the starter to start the engine
      - ``ICE_PWM_STRT_OFF`` is the PWM value sent to the starter when not trying to start the engine
      - ``ICE_PWM_IGN_ON`` is the PWM value sent to the ignition power switch when the engine should be running
      - ``ICE_PWM_IGN_OFF`` is the PWM value sent to the ignition power switch when the engine should be stopped

RC Switch Setup
^^^^^^^^^^^^^^^

It is common to use a 3 position switch on an RC transmitter is used to control the engine. This switch has the following three positions:

- Low: Force the engine to be disabled. This ignores MAVLink commands and mission items that attempt to control the engine's state.
- Mid: Keep the current state of the engine, but allow MAVLink commands and mission items to change the state of the engine.
- High: Force the engine to be enabled. This ignores MAVLink commands and mission items that attempt to control the engine's state.

To start the engine, raise the switch to the high position. If you want the automatic engine-cut features to work, be sure to lower the switch to the middle position after the engine has started.

To stop the engine, lower the switch to the low position. If you want the engine to be automatically start with a mission, or want to use a MAVLink command to start the engine, be sure to raise the switch to the middle position after the engine has stopped.

To configure the RC switch,

.. tabs::
   .. tab:: ArduPilot 4.6 and later

      - Set an RC auxiliary switch (:ref:`RCx_OPTION<RC6_OPTION>` = 179) be used to start the engine. This channel can start the engine or stop the engine via the ``Ignition`` and ``Starter`` motor/servo outputs (see :ref:`common-rcoutput-mapping`). Normally, the "kill" PWM value is anything below 1300us, but this can be changed using the :ref:`ICE_STARTCHN_MIN<ICE_STARTCHN_MIN>` parameter. Setting up this channel and its RC control is required for operation in order to provide a "kill" function on the throttle, even if there is no ignition or starter control.
       
   .. tab:: ArduPilot prior to 4.6

      - Set ``ICE_START_CHAN`` to the channel number corresponding to a switch on the transmitter which will be used to start the engine. This channel can start the engine or stop the engine via the ``Ignition`` and ``Starter`` outputs. Normally, the "kill" PWM value is anything below 1300us, but this can be changed using the :ref:`ICE_STARTCHN_MIN<ICE_STARTCHN_MIN>` parameter. Setting up this channel and its RC control is required for operation in order to provide a "kill" function on the throttle, even if there is no ignition or starter control.

      These parameters may also need to be adjusted:

      - :ref:`ICE_STARTCHN_MIN<ICE_STARTCHN_MIN>` is the minimum PWM below which the start channel input will be ignored. This is a safety feature to prevent bad RC input from stopping the motor while the aircraft is beyond line of sight, if the RC protocol is incorrectly configured. Default is zero, meaning disabled.

Starting with an Onboard Starter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If using an onboard starter, it is strongly recommended to configure an RPM sensor for the engine, since the primary benefit of an onboard starter is to automatically attempt to restart the engine if it fails in flight. See `RPM Configuration`_ below for more information.

The auto-start functionality will attempt to start the engine any time the vehicle is armed, the engine is enabled, and the measured RPM is below the :ref:`ICE_RPM_THRESH <ICE_RPM_THRESH>`. If the engine is not successfully started within a configurable amount of time, the program will wait for a configurable delay before attempting to start again. It is important to remember that the starter will run in pulses. DO NOT approach the engine between failed start attempts as the starter will attempt to start again if the engine is still enabled.

- :ref:`ICE_START_PCT <ICE_START_PCT>` overrides the throttle setting during start.
- :ref:`ICE_STARTER_TIME <ICE_STARTER_TIME>` controls the maximum amount of time the starter will run in each start attempt.
- :ref:`ICE_START_DELAY <ICE_START_DELAY>` sets a delay between start attempts. This can be useful when your starter has a limited duty cycle.
- :ref:`ICE_RPM_THRESH <ICE_RPM_THRESH>` sets the minimum RPM reading for the engine to be considered running. This should be set to a value below your idle RPM.
- :ref:`ICE_STRT_MX_RTRY<ICE_STRT_MX_RTRY>` if set to a value greater than 0 then the engine will retry starting the engine this many times before giving up.


Starting without an Onboard Starter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If using a hand-held starter motor to start the engine, set :ref:`ICE_STARTER_TIME <ICE_STARTER_TIME>` to a relatively high value (at least 10 seconds). This will give you time between hitting the switch and starting the engine with the hand-held starter. After this time period has elapsed, the throttle will drop from :ref:`ICE_START_PCT <ICE_START_PCT>` to :ref:`ICE_IDLE_PCT <ICE_IDLE_PCT>` regardless of whether the engine has started or not. *After* this point, if you are using an optional RPM sensor, the engine start logic may begin again if the engine has not started, or if it stalls after starting and the RPM drops below the :ref:`ICE_RPM_THRESH <ICE_RPM_THRESH>`. Because of this, it is not recommended to set a short time period for the starter cycle, since it will periodically dip the throttle as you are trying to start the engine, which might make it harder to start.

If the engine stalls after startup, toggle the engine control off and on, then try to start again.

RPM Configuration
^^^^^^^^^^^^^^^^^
ArduPilot supports generic pulse tachometers connected to GPIO pins (such as the Aux servo pins on a PixHawk or Cube). A tachometer may be made using a simple hall effect switch IC. Some ignition modules support a tachometer output that can be connected directly to the GPIO pins. Desert Aircraft Electronic Ignition V2 modules support tachometer output on the signal pin of the power input connector. Note that when connecting an RPM sensor to an AUX pin, it is important to make sure that this pin is not configured to output a PWM value and, instead, be a GPIO pin. See :ref:`common-gpios` and :ref:`common-rpm` for more information.

Alternatively, many electronic fuel injection (EFI) systems transmit engine RPM via their own telemetry system. If you have configured an EFI system, and it supports RPM telemetry, then ArduPilot can use this RPM value directly without needing a separate RPM sensor. See :ref:`Electronic Fuel Injection <common-efi>`.

In order to configure an RPM sensor, the following parameters must be set:

- Set :ref:`RPM1_TYPE <RPM1_TYPE>` for whichever RPM source you are using.
- Set the remaining 'RPM_*' parameters as appropriate for your system.
- Set :ref:`ICE_RPM_CHAN <ICE_RPM_CHAN>` to 1.

Idle and redline governors
--------------------------

These features rely on having a source for engine RPM. See `RPM Configuration`_ above for more information.

The idle governor allows the autopilot to adjust the throttle to maintain an RPM value when the commanded throttle is low. This can be useful when the engine is waiting for takeoff and reduces the workload on the pilot during that time. Increasing the throttle command will give the expected throttle response.

.. note:: Prior to ArduPilot 4.7, while the aircraft is disarmed, the idle governor only works in Manual mode (and only if :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 2 is set). If your engine cannot run reliably without the idle governor, you will need to arm in Manual mode before switching to another mode to take off. Once armed, the idle governor will work in all modes.

- :ref:`ICE_IDLE_PCT<ICE_IDLE_PCT>` This is the minimum percentage throttle output while running, this includes being disarmed, but not while outputs are disabled by the safety switch.
- :ref:`ICE_IDLE_RPM<ICE_IDLE_RPM>` This configures the RPM that will be commanded by the idle governor. Set to -1 to disable.
- :ref:`ICE_IDLE_DB<ICE_IDLE_DB>` This configures the RPM deadband that is tolerated before adjusting the idle setpoint.
- :ref:`ICE_IDLE_SLEW<ICE_IDLE_SLEW>` This configures the slewrate used to adjust the idle setpoint in percentage points per second.

The redline governor will slowly reduce the throttle if the RPM remains above the given RPM value. Generally, this RPM value is provided by the manufacturer of the engine. If the commanded throttle drops faster or lower than the point the governor started, then the governor will be released and normal control of the throttle will return to normal throttle response. A warning message will appear on the GCS. Setting :ref:`ICE_OPTIONS<ICE_OPTIONS>` bit 1 allows disabling the throttle actions of the redline governor, but still displays the GCS warning message.

- :ref:`ICE_REDLINE_RPM<ICE_REDLINE_RPM>` Maximum RPM for the engine provided by the manufacturer. A value of 0 disables this feature.

Options
-------

Several options are provided using the :ref:`ICE_OPTIONS<ICE_OPTIONS>` bitmask parameter:

===================     ========
Bit (Set to enable)     Function
===================     ========
0                       Disable ignition during RC failsafe
1                       Disable the Redline Governor feature (but still display GCS warnings)
2                       Allows throttle servo output while disarmed in MANUAL mode
3                       Prevent starting while disarmed
4                       Reverse engine cranking direction on starters which support this
===================     ========

Vibration isolation
-------------------

The high vibration from internal combustion engines means that :ref:`vibration dampening <common-vibration-damping>` is critical.  It is often necessary to mount the autopilot on a plate with `self-adhesive lead weights <https://www.amazon.com/Great-Planes-Segmented-Weights-6-Ounce/dp/B0015KLJE0>`__ added to increase its mass.

.. note::

   When using Internal Combustion Engine (ICE) motors, vibration during
   engine run-up can cause the IMU consistency checks to fail, preventing
   arming.
   In ArduPilot 4.7 and later an option has been added to :ref:`ARMING_OPTIONS<ARMING_OPTIONS>`: bit 2 (Skip IMU consistency checks when ICE motor running)

   When this bit is set (value +4), ArduPilot will bypass the IMU consistency check
   if an ICE motor is running. This is intended for setups
   where ICE engines are typically started before arming, and their
   vibration may otherwise trigger false failures.
