.. _common-auxiliary-functions:

===================
Auxiliary Functions
===================

This feature is firmware versions 4.0 and higher. In Copter versions before 4.0 similar functions were implemented using the CHx_OPT parameters. See :ref:`Auxiliary Function Switches (3.6 and earlier)<channel-7-and-8-options>` page.

This page describes how to set up additional features which can be invoked from the transmitter's auxiliary function switches or from external :ref:`buttons <common-buttons>`.

Configuring which transmitter channel is used
=============================================

Any RC input channel can be assigned any auxiliary function. However, the RC channel should not be used by any other function, such as flight mode selection or flight controls. By default, channels 1-4 are used for flight controls (i.e. roll, pitch, throttle, and yaw). The default flight mode channel is 8 for Plane and Rover and 5 for Copter.

Assigning the Feature
=====================

The RCx_OPTION parameters control these features. For example, the :ref:`RC7_OPTION <RC7_OPTION>` parameter controls which feature is assigned to RC input 7. Each RC channel has its
own RCx_OPTION parameter that can be accessed in the full parameter list.


Supported Features
==================

+----------------------+----------------------------+----------+---------+---------+
| **RCx_OPTION value** | **Feature Description**    |**Copter**|**Plane**|**Rover**|
+----------------------+----------------------------+----------+---------+---------+
|        0             | Do Nothing (Default)       |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        2             | FLIP                       |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        3             | SIMPLE mode (Copter)       |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        4             | RTL mode                   |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        5             | Save Trim                  |    X     |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        7             | Save Waypoint              |    X     |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        9             | Camera Trigger             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        10            | Rangefinder                |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        11            | Fence                      |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        12            | ResetToArmedYaw            |          |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        13            | SUPERSIMPLE mode           |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        14            | Acro Trainer               |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        15            | Sprayer                    |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        16            | AUTO mode                  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        17            | AUTOTUNE mode              |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        18            | LAND Mode                  |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        19            | Gripper                    |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        21            | Parachute Enable           |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        22            | Parachute Release          |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        23            | Parachute 3 Pos Switch     |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        24            | Reset Auto Mission to start|    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        25            | Attitude Controller FF     |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        26            | Attitude Controller AccLim |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        27            | Retract Mount              |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        28            | Relay 1 On/off             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        29            | Landing Gear               |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        30            | Lost Vehicle Sound         |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        31            | Motor Emergency Stop       |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        32            | Motor Interlock            |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        33            | BRAKE                      |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        34            | Relay 2 On/Off             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        35            | Relay 3 On/Off             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        36            | Relay 4 On/Off             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        37            | THROW mode                 |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        38            | ADSB Avoidance Enable      |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        39            | Precision Loiter           |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        40            | Object Avoidance           |    X     |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        41            | Arm Disarm (4.1 and before)|    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        42            | SMARTRTL mode              |    X     |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        43            | Inverted Flight            |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        44            | Winch Enable               |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        45            | Winch Control              |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        46            | RC Override Enable         |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        47            | Rsvd for custom function   |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        48            | Rsvd for custom function   |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        49            | Rsvd for custom function   |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        50            | Learn Cruise               |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        51            | MANUAL mode                |          |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        52            | ACRO mode                  |    X     |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        53            | STEERING mode              |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        54            | HOLD mode                  |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        55            | GUIDED mode                |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        56            | LOITER mode                |    X     |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        57            | FOLLOW mode                |    X     |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        58            | Clear Waypoints            |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        59            | SIMPLE mode (Rover)        |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        60            | ZIGZAG mode                |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        61            | ZIGZAG mode - Save Waypts  |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        62            | Compass Learn              |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        63            | Sailboat Tack              |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        64            | Reverse Throttle           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        65            | GPS Disable                |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        66            | Relay 5 On/Off             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        67            | Relay 6 On/Off             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        68            | STABILIZE mode             |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        69            | POSHOLD mode               |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        70            | ALTHOLD mode               |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        71            | FLOWHOLD mode              |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        72            | CIRCLE mode                |    X     |    X    |   X     |
+----------------------+----------------------------+----------+---------+---------+
|        73            | DRIFT mode                 |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        74            | Sailboat Motor 3Pos Sw     |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        75            | Surface Tracking Up/Down   |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        76            | STANDBY mode               |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        77            | TAKEOFF mode               |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        78            | RunCam Control             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        79            | RunCam OSD Control         |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        80            | Viso Align                 |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        81            | Disarm                     |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        82            | Q_Assist 3Pos Sw           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        83            | ZIGZAG Auto                |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        84            | AIRMODE(not a flight mode) |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        85            | Generator                  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        86            | Non Auto Terrain Follow    |          |    x    |         |
|                      | Disable                    |          |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        87            | CROW Mode Switch           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        88            | Soaring Enable             |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        89            | Force Flare                |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        90            | EKF Position Source        |     X    |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        91            | Airspeed Ratio Calibrate   |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        92            | FBWA Mode                  |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        94            | VTX Power                  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        95            | FBWA_TAILDRAGGER           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        96            | MODE_SWITCH_RESET          |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        97            | WindVane home dir offset   |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        102           | Camera Mode Toggle         |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        105           | GPS Disable Yaw            |    X     |    X    |    X    |
|                      | (testing only!)            |          |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        106           | Disable Airspeed Use       |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        107           | Enable Autotuning          |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        108           | QRTL Mode                  |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        112           | SwitchExternalAHRS         |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        150           | CRUISE Mode                |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        151           | TURTLE Mode                |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        152           | SIMPLE mode heading reset  |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        153           | ARM/DISARM (4.2 and higher)|    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        154           | ARM/DISARM with AIRMODE on |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        155           | TRIM RC/SERVO Save         |          |    X    |   X     |
+----------------------+----------------------------+----------+---------+---------+
|        156           | Torqeedo Error Clear       |          |         |   X     |
+----------------------+----------------------------+----------+---------+---------+
|        157           | Force FBWA Long FS Action  |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        158           | Optflow Calibration        |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        159           | Force Flying State         |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        160           | WeatherVane Enable         |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        161           | Turbine Start (Heli)       |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        162           | In-Flight FFT AutoSetup    |    X     |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        163           | Mount Lock                 |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        164           | Pause Streaming Logging    |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        165           | ARM/Motor Emergency Stop   |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        166           | Camera Record Video        |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        167           | Camera Zoom                |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        168           | Camera Manual Focus        |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        169           | Camera Auto Focus          |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        170           | QSTABILIZE mode            |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        171           | Compass Calibration        |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        172           | Battery MPPT Enable        |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        173           | Plane AUTO Landing Abort   |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        174           | Camera Image Tracking      |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        175           | Camera Lens                |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+

Intended as continuous PWM range control inputs:

+----------------------+----------------------------+----------+---------+---------+
| **RCx_OPTION value** | **Feature Description**    |**Copter**|**Plane**|**Rover**|
+----------------------+----------------------------+----------+---------+---------+
|        201           | ROLL Input                 |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        202           | PITCH Input                |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        203           | THROTTLE Input             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        204           | YAW Input                  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        207           | Mainsail                   |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        208           | Flap Control               |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        209           | Forward Throttle           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        210           | Airbrakes                  |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        211           | Walking Robot Height       |          |         |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        212           | Mount1 Roll                |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        213           | Mount1 Pitch               |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        214           | Mount1 Yaw                 |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        215           | Mount2 Roll                |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        216           | Mount2 Pitch               |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        217           | Mount2 Yaw                 |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        300-307       | Scripting RC channels      |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+

.. note:: 201-204 are not implemented, reserved for future use.

Description of Features
=======================

.. note:: in descriptions below, LOW and HIGH for a channel refers to PWM <1200us and >1800us, respectively.

Mode Switches
-------------

Any feature ending with "mode" provides the ability to switch the vehicle into that mode by setting the RC channel to high. You can have multiple "mode" option switches and more than one can be high at a time. The last "mode" change switch will determine the current mode, as well as any change of the normal mode switch.

For example, if you have a "LOITER mode" switch active and then an "AUTO mode" switch is switched high, the mode will change to AUTO. Changing the normal flight mode switch will again change the mode to the new flight mode setting, even though both RCx_OPTION mode switches are high. Lowering an active RCx_OPTION mode switch back to low will return the flight mode to whatever is set on the flight mode channel, but only if the current mode matches the mode set by that switch. Otherwise, it will have no effect.

.. note:: Copter and Rover mode changes are not guaranteed. They may be denied if the conditions required for that mode are not met. For example, changing to LOITER mode in Copter would fail if the GPS lock is not active, whereas in Plane the demanded mode will change and operate as best as it can.

.. note:: If mapped to a three-position switch then the SUPERSIMPLE mode function will allow **SUPERSIMPLE** and **SIMPLE** modes to be enabled using the high and middle switch positions, respectively (a two-position switch will enable/disable SUPERSIMPLE mode only). :ref:`See here for more details<simpleandsuper-simple-modes>`.

Other functions are:

   ===================================== =======================================================================
    Option                                Description
   ===================================== =======================================================================
    Flip                                 | The vehicle will flip on its roll or pitch axis depending upon the
                                         | pilot's roll and pitch stick position. See :ref:`Flip Mode<flip-mode>`.
    Save Trim                            | In Rover, a high saves the current steering channel trim,
                                         | see :ref:`Save Steering Trim <savetrim>`.
                                         | In Copter, it adjusts the vehicle level position using the current roll
                                         | and pitch stick inputs. See details :ref:`here <auto-trim>`.
    Save Waypoint                        | Save the current location (including altitude) as a waypoint in the
                                         | mission.If in AUTO mode no waypoint will be saved, instead the 
                                         | vehicle will RTL.
    Camera Trigger                       | The camera shutter will be activated.
                                         | See more details :ref:`here <common-camera-shutter-with-servo>`.
    Range Finder                         | :ref:`RangeFinder <common-rangefinder-landingpage>` is disabled when
                                         | the switch is in a low position, and enabled when in a high position.
    Fence                                | Fence is disabled when the switch is in a low position, and enabled
                                         | when in a high position.
    Acro Trainer                          Turn on automatic leveling in the ACRO flight mode.
    Sprayer                               Turn on the :ref:`crop sprayer <sprayer>` when the switch is pulled high.
    Gripper                              | Operates the :ref:`gripper <common-gripper-landingpage>`. Switch pulled
                                         | low releases the gripper, high closes or grabs.
    Parachute Enable                     | Enables the automatic release of the :ref:`parachute <common-parachute>`
                                         | (this does not immediately trigger the release).
    Parachute Release                    | Immediately triggers the release of the :ref:`parachute <common-parachute>`
                                         | as long as the vehicle is not landed, or too low.
    Parachute 3Pos                       | Switch pulled low disables the :ref:`parachute <common-parachute>`.
                                         | The switch in the middle position enables the parachute for
                                         | automatic release. The switch pulled high triggers the release of the
                                         | parachute as long as the vehicle is not landed, or too low.
    Mission Reset                         Reset AUTO to run the first mission command in the command list.
    AttCon Feed Forward                  | Turns on/off attitude controllers feed forward.
                                         | For developers only.
    AttCon Accel Limits                  | Turns on/off attitude controller acceleration limits.
                                         | For developers only.
    Retract Mount                         Move the :ref:`camera mount <common-cameras-and-gimbals>` to its retracted position.
    Relay 1 On/Off                       | Switch pulled low turns off the first :ref:`relay <common-relay>`,
                                         | pulled high turns on the first relay.
    Landing Gear                          Deploys or Retracts :ref:`Landing Gear <common-landing-gear>`
    Lost Vehicle Alarm                    Plays the `lost copter alarm <https://download.ardupilot.org/downloads/wiki/pixhawk_sound_files/LostCopter.wav>`__ through the buzzer
    Emergency Stop Motors                 Stops motors immediately (`video <https://www.youtube.com/watch?v=-Db4u8LJE5w>`__)
    Motor Interlock                      | Motor Interlock controls the way the heliRSC (motor throttle control)
                                         | output is generated in Traditional Helicopters and HeliQuads. If
                                         | >1200us, it enables the Motor Interlock function, below it is disabled.
                                         | When <1200us, it is similar to what is sometimes referred to as
                                         | Throttle Hold in RC Helicopter terminology.
                                         | For Mulit-copters, it is used as a motor stop function when <1200us.
                                         | (`video <https://youtu.be/-Db4u8LJE5w?t=51>`__).
    Brake                                | Invokes the :ref:`Brake flight mode <brake-mode>` when the switch goes high.
                                         | Bringing the switch back to low will return the vehicle to the mode 
                                         | indicated by the flight mode switch.
    Relay2 On/Off                        | Switch pulled low turns off the second :ref:`relay <common-relay>`,
                                         | pulled high turns on the second relay.
    Relay3 On/Off                        | Switch pulled low turns off the third :ref:`relay <common-relay>`,
                                         | pulled high turns on the third relay.
    Relay4 On/Off                        | Switch pulled low turns off the fourth :ref:`relay <common-relay>`,
                                         | pulled high turns on the fourth relay.
    Throw                                | Invokes the :ref:`Throw flight mode <throw-mode>` when the switch
                                         | goes high. Bringing the switch back to low will return the vehicle
                                         | to the mode indicated by the ch5 flight mode switch.
    ADSB-Avoidance                       | When the switch is high, :ref:`ADSB avoidance <common-ads-b-receiver>`
                                         | (avoidance of manned aircraft) is enabled, otherwise it's disabled
    Precision Loiter                     | Turns on/off :ref:`Precision Loiter <precision-landing-with-irlock>`.
                                         | (i.e. holding position above a target in Loiter mode
                                         | using IR-Lock sensor)
    Object Avoidance                     | When the switch is high, avoid objects using :ref:`Lightware SF40c <common-lightware-sf40c-objectavoidance>`
                                         | or :ref:`TeraRanger Tower<common-teraranger-tower-objectavoidance>`. When low, object avoidance is disabled.
    Arm/Disarm(4.1 and earlier)          | Arms the vehicle if the switch goes high (subject to arming checks).
                                         | Disarms the vehicle if brought low.
   ===================================== =======================================================================

   ===================================== =======================================================================
    Option                                Description
   ===================================== =======================================================================
    Inverted Flight                      | Enabling inverted flight only changes how ArduPilot stabilizes
                                         | the vehicle. It will stabilize it with a roll of 180 degrees from
                                         | normal whenever inverted flight is enabled in a stabilized mode.
                                         | Unless the vehicle is capable of inverted flight, do **NOT** use
                                         | this option.
    Winch Enable                         | Enables Winch operation. The switch in the low position on this
                                         | channel relaxes the winch.
    Winch Control                        | Controls the speed and direction of the winch. Low: takeup,
                                         | Middle: stop, High: unreel
    RC Override Enable                   | This is a 3-position switch that enables (high) or disables (low)
                                         | the use of RC overrides from the Ground Control Station.
    Learn Cruise                         | This starts the cruise speed and the throttle learning sequence
                                         | on Rover when switched to high. See :ref:`rover-tuning-throttle-and-speed`.
    Clear Waypoints                       Clears currently loaded mission waypoints.
    Compass Learn                        | Inflight compass offset learning. See Automatic Offset
                                         | Calibration section of :ref:`common-compass-setup-advanced`.
    Sailboat Tack                        | Any high to low, or low to high change on this channel will start
                                         | a tack in the opposite direction
                                         | to the last tack. See Sailboat :ref:`sailboat-configure`.
    Reverse Throttle                     | When switched high, forces throttle reverse in Plane for
                                         | steepening descents. Normally, this is
                                         | controlled by flight mode via the :ref:`USE_REV_THRUST<USE_REV_THRUST>`
                                         | parameter. See :ref:`automatic-landing` for more information on
                                         | the setup of reverse thrust.
    GPS Disable                           Simulates GPS failure by disabling GPS.
    Relay 5 On/Off                       | Switch pulled low turns off the third :ref:`relay <common-relay>`, pulled
                                         | high turns on the fifth relay.
    Relay 6 On/Off                       | Switch pulled low turns off the third :ref:`relay <common-relay>`, pulled
                                         | high turns on the sixth relay.
    Sailboat Motor 3Pos Switch           | This 3-position switch controls the Sailboat motor. The motor is
                                         | always used when high, never used 
                                         | when low, and as needed, otherwise.
    Surface Tracking Up/Down             | This 3-position switch determines if surface tracking via
                                         | rangefinder is toward the ground (low) or ceiling (high),
                                         | or disabled, otherwise.
    Standby                              | This puts the autopilot control loops into a soft standby mode
                                         | so that a parallel,redundant autopilot or
                                         | companion computer can assume control of the vehicle.
                                         | The PID loops, position, and altitude controllers are modified
                                         | such that the autopilot can smoothly resum autopilot can
                                         | smoothly resume control of the vehicle when standby is
                                         | subsequently disabled. Switching of outputs or other peripherals
                                         | must be done by external circuitry.
    RunCam Control                       | Allows starting and stopping video recording of compatible
                                         | RunCam cameras. See :ref:`common-camera-runcam`.
    RunCam OSD Control                   | Enables control of RunCam cameras OSDs.
                                         | See :ref:`common-camera-runcam`.
    VISO Align                            Aligns external Visual Odometry with current autopilot AHRS
    Disarm                               | Disarm the vehicle unconditionally and immediately. Unlike
                                         | Emergency Stop Motors, which waits for :ref:`DISARM_DELAY<DISARM_DELAY>`
                                         | in Copter.
    Q_Assist 3Pos SW                     | Low: disable Q_Assist entirely, Middle: Normal Q_Assist
                                         | operation, High: Q_Assist active at all times. See Assisted Fixed
                                         | Wing Flight section of :ref:`quadplane-flying`.
    ZigZag Mode Auto Enable              | Enable automatic zigzag and sprayer in ZIGZAG mode.
                                         | See :ref:`zigzag-mode`.
    AIRMODE(not a flight mode)            Enables and disables AIRMODE feature. See :ref:`airmode`
    Generator                             Mode control for Richenpower Hybrid Power Generator
    Non Auto Terrain Follow Disable       Disables Terrain Following in CRUISE and FBWB modes
    CROW Mode Switch                      Selects between different CROW aileron operating modes
    Soaring Enable                        Enables Soaring function operating modes
    Force Flare                          | Moves tilt motors to the upright position and optionally sets
                                         | pitch for flare when landing Tilt Rotor QuadPlanes.
                                         | Middle: The pilot retains pitch control during a flare.
                                         | High: Pitch set to :ref:`LAND_PITCH_DEG<LAND_PITCH_DEG>`. Allows switching
    EKF Position Source                  | between up to three source sets manually for EKF3 (only).
                                         | See :ref:`common-non-gps-to-gps`.
    Airspeed Ratio Calibrate             | Activates calibration of airspeed ratio in flight. Best results
                                         | occur while executing course changes over 360 degrees over
                                         | time, as in LOITER mode. See :ref:`calibrating-an-airspeed-sensor`.
    VTX Power                            | Allows reading up to a 6-position switch for controlling Video
                                         | Transmitter Power. See :ref:`common-vtx`.
    FBWA_TAILDRAGGER                     | Enables FBWA taildragger takeoff mode using elevator to
                                         | hold tail on the ground until airspeed is reached
    MODE_SWITCH_RESET                     Forces mode switch to be re-read.
    WindVane home direction offset       | This is a continuous input channel providing a -45 to +45
                                         | degree offset of the initial wind direction when using
                                         | :ref:`WNDVN_TYPE<WNDVN_TYPE>` = 2.
    Camera Mode Toggle                   | Toggle camera mode (Photo/Video/etc.). Ideally, this should be
                                         | on a momentary switch since only low to high transitions
                                         | toggle the camera mode. Used only with Solo gimbals
                                         | presently.
    GPS Disable Yaw                       Disables yaw for testing (advanced users only!)
    Disable Airspeed Use                  Forces Airspeed Use to be disabled for testing in the air.
    Enable Autotuning                    | Allows tuning without entering AUTOTUNE mode. (i.e. if you
                                         | place the vehicle in LOITER/AUTO with stick mixing enabled,
                                         | it can autotune while the vehicle is loitering by using sticks,
                                         | but enabling autotuning can occur in any mode other
                                         | than MANUAL.)
    SwitchExternalAHRS                   | If EKF3 and an external AHRS module are enabled, high on this
                                         | switch switches from EKF3 to the external AHRS
    SIMPLE mode heading reset            | Resets original heading reference to current heading
                                         | for SIMPLE Mode.
    ARM/DISARM (4.2 and higher)          | Arms the vehicle if the switch goes high (subject to arming
                                         | checks). Disarms the vehicle unconditionally if brought low.
    ARM/DISARM with AIRMODE on           | Arms the vehicle if the switch goes high (subject to arming
                                         | checks) with AIRMODE active. Airmode RC option switch can
                                         | subsequently enable or disable if it is configured. Disarms
                                         | the vehicle unconditionally if brought low.
    TRIM RC/SERVO Save                   | Saves current RC input trim and SERVO output trim for pitch,
                                         | roll, and yaw in Plane and for Steering in Rover.
    Torqeedo Error Clear                  Clears error condition in Torqeedo motor controller.
    Force FBWA as Long FS Action         | Forces mode change to FBWA in Long FS, overriding the
                                         | :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` parameter value for emergency
                                         | landings beyond RC control range to prevent normal
                                         | failsafe action from occurring.
    Optflow Calibration                   Enables calibration of optical flow parameters.
    Force Flying                         | Disables the landing detection heuristic to prevent false
                                         | landing detections during a mission or manual flight
                                         | if sudden Z changes can occur due to wind gusts, etc.
    WeatherVane Enable                    Enables or disables weathervaning in Quadplane VTOL modes.
    Turbine Start (Heli)                 | When armed and RSC is idle, the high position signals the
                                         | helicopter rotor speed governor to ramp the throttle to full and
                                         | back to idle, which signals the turbine engine ECU to
                                         | initiate the start sequence. The switch must be set back low and
                                         | the aircraft has to be disarmed to re-enable this feature.
    In-Flight FFT AutoSetup              | Allows automatic setup of in-flight FFT notch parameters. Set
                                         | :ref:`FFT_ENABLE<FFT_ENABLE>` =1, takeoff with switch low,
                                         | hover 30 seconds with switch high,switch low, and land and
                                         | notch parameters will have been configured.
    Mount Lock                           | If high, locks the heading of all mounts to earth-frame,
                                         | otherwise, yaw is heading locked to vehicle heading.
                                         | If pilot controlled positioning is active, the pilot's inputs changes
                                         | the heading target appropriately in whatever frame is selected.
                                         | Without this switch,it's vehicle heading.
    Pause Streaming Logging              | If high, will not log streaming type log messages (sensors,
                                         | attitudes, EKF, etc.) to allow logging only when desired
                                         | for autopilots with limited logging capabilities
                                         | (i.e. no SD card). Events, mode changes, etc. are still logged.
                                         | Logging is unaffected if the switch is low.
    ARM/Motor Emergency Stop             | Three position switch. If high, will request arming. If switched to
                                         | low position, will emergency stop any rotating motor output
                                         | like the Motor Emergency Stop switch. If switched to middle
                                         | position,will de-activate the Motor Emergency Stop, but not
                                         | request an arm condition. This is a safer alternative to
                                         | ARM/DISARM in that accidental switching to low position
                                         | will not disarm, and will allow recovery in the air if
                                         | switched back to middle or high quickly.
    Camera Record Video                   Controls video recording on some cameras/mounts.
    Camera Zoom                           Controls camera zoom on some cameras/mounts.
    Camera Manual Focus                   Changes manual focus on some cameras/mounts.
    Camera AutoFocus                      Controls autofocus on some cameras/mounts.
    Compass Calibration                  | Switching to high will behave the same as if the Start button 
                                         | for :ref:`onboard calibration <onboard_calibration>` had
                                         | been pressed. Returning the switch to low will cancel the
                                         | calibration if still in progress.
    Battery MPPT Enable                   Enable the Packet Digital MPPT solar panel power monitor
    Plane AUTO Mode Landing Abort        | If switched to the HIGH position, will abort any landing that
                                         | is currently in progress while in AUTO mode.
                                         | This includes the VTOL, or fixed wing,
                                         | landing phase of any AUTO mission, and the :ref:`PAYLOAD_PLACE <mav_cmd_nav_payload_place>`
                                         | mission command. It does not  affect the fixed wing
                                         | approach phase of a VTOL landing, QLAND or QRTL modes.
    Camera Image Tracking                 Activate Camera Image Tracking (only supported by ViewPro camera gimbals)
    Camera Lens                           Select the active camera lens (only supported by ViewPro camera gimbals)
    ROLL Input                            ROLL input channel. (replaces RCMAP)
    PITCH Input                           PITCH input channel. (replaces RCMAP)
    THROTTLE Input                        THROTTLE input channel. (replaces RCMAP)
    YAW Input                             YAW input channel. (replaces RCMAP)
    Mainsail                             | This RC channel will drive the output of the MainSail 
                                         | output ( ``SERVOx_FUNCTION`` = 89)instead of being set from 
                                         | the Throttle Input channel (useful if it has an auxiliary motor
                                         | using that input). See Sailboat :ref:`sailboat-configure` for 
                                         | more information about the main sail setup.
    Flaps                                | This RC channel provides manual control the amount of FLAP 
                                         | deflection and can also be used in conjunction with
                                         |  :ref:`automatic-flaps` and/or :ref:`flaperons<flaperons-on-plane>`.
                                         | (Replaces the old FLAP_IN_CHANNEL parameter)
    Forward Throttle                     | Manual forward motor throttle in QSTABILIZE, QACRO, and
                                         | QHOVER modes
    Airbrakes                             Controls deployment of :ref:`Airbrakes<airbrakes-on-plane>`
    Walking Robot Height                  Input channel for Walking Robot Height. See :ref:`walking-robots`.
    Mount1 Roll                           1st camera gimbal's roll control
    Mount1 Pitch                          1st camera gimbal's pitch control
    Mount1 Yaw                            1st camera gimbal's yaw control
    Mount2 Roll                           2nd camera gimbal's roll control
    Mount2 Pitch                          2nd camera gimbal's pitch control
    Mount2 Yaw                            2nd camera gimbal's yaw control
    Scripting RC channels                 Allows reading a dedicated RC channel for script inputs
   ===================================== =======================================================================

Check the channel range
=======================

.. image:: ../../../images/aux-switch-check.png
    :target: ../_images/aux-switch-check.png

The configured feature will be triggered when the auxiliary switch's PWM value becomes higher than 1800.  It will be deactivated when the value falls below 1200.

You can check the PWM value sent from the transmitter when the switch is high and low using the Mission Planner's Initial Setup >> Mandatory Hardware >> Radio Calibration screen.  If it does not climb higher than 1800 or lower than 1200, it is best to adjust the servo endpoints in the transmitter.
