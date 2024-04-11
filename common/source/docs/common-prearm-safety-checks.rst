.. _common-prearm-safety-checks:

=====================
Pre-Arm Safety Checks
=====================

ArduPilot includes a suite of Pre-arm Safety Checks which will prevent the
vehicle from arming its propulsion system if any of a fairly large number of issues are
discovered before movement including missed calibration, configuration
or bad sensor data. These checks help prevent crashes or fly-aways but
some can also be disabled if necessary.

..  youtube:: gZ3H2eLmStI
    :width: 100%

.. warning:: Never disable the arming checks (ie :ref:`ARMING_CHECK<ARMING_CHECK>` not = "1", except for bench testing. Always resolve any prearm or arming failures BEFORE attempting to fly. Doing otherwise may result in the loss of the vehicle.

Recognising which Pre-Arm Check has failed using the GCS
========================================================

The pilot will notice a pre-arm check failure because he/she will be
unable to arm the vehicle and the notification LED, if available, will be flashing yellow.  To
determine exactly which check has failed:

#. Connect the Autopilot to the ground station using a USB cable
   or :ref:`Telemetry <common-telemetry-landingpage>`.
#. Ensure the GCS is connected to the vehicle (i.e. on Mission
   Planner and push the "Connect" button on the upper right).
#. Turn on your radio transmitter and attempt to arm the vehicle
   (regular procedure is using throttle down, yaw right or via an RCx_OPTION switch)
#. The first cause of the Pre-Arm Check failure will be displayed in red
   on the HUD window

Pre-arm checks that are failing will also be sent as messages to the GCS while disarmed, about every 30 seconds. If you wish to disable this and have them sent only when an attempt arm fails, then set the :ref:`ARMING_OPTIONS<ARMING_OPTIONS>` bit 1 (value 1).

.. table:: Pre-Arm Failure Messages (All vehicle types)
    :widths: auto

    ======================================================= =================================================== ====================================================
    Message                                                 Cause                                               Solution
    ======================================================= =================================================== ====================================================
    3D Accel calibration needed                             Accelerometer calibration has not been done         Complete the :ref:`accel calibration <common-accelerometer-calibration>`
    Accels calibrated requires reboot                       Autopilot must be rebooted after accel calibration  Reboot autopilot
    Accels inconsistent                                     Two accelerometers are inconsistent by 0.75 m/s/s   Re-do the :ref:`accel calibration <common-accelerometer-calibration>`.  Allow autopilot to warm-up and reboot.  If failure continues replace autopilot
    ADSB out of memory                                      Autopilot has run out of memory                     Disable features or replace with a higher powered autopilot
    Accels not healthy                                      At least one accelerometer is not providing data    Reboot autopilot.  If failure continues replace autopilot
    AHRS: not using configured AHRS type                    EKF3 is not ready yet and vehicle is using DCM      If indoors, go outside.  Ensure good GPS lock.  Check for misconfiguration of EKF (see :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>`)
    AHRS: waiting for home                                  GPS has not gotten a fix                            If indoors, go outside.  Ensure compass and accelerometer calibrations have been completed.  Eliminate radio-frequency sources that could interfere with the GPS
    Airspeed 1 not healthy                                  Autopilot is unable to retrieve data from sensor    Check the physical connection and :ref:`configuration <airspeed>`
    AP_Relay not available                                  Parachute misconfigured                             Parachute is controlled via Relay but the relay feature is missing. `Custom build server <https://custom.ardupilot.org/>`__ was probably used, rebuild with relay enabled
    Auxiliary authorisation refused                         External system has disallowed authorisation        Check external authorisation system
    Batch sampling requires reboot                          Batch sampling feature requires Autopilot reboot    Reboot autopilot or check :ref:`Batch sampling<common-imu-batchsampling>` configuration
    Battery below minimum arming capacity                   Battery capacity is below BATT_ARM_MAH              Replace battery or adjust :ref:`BATT_ARM_MAH<BATT_ARM_MAH>`
    Battery below minimum arming voltage                    Battery voltage is below BATT_ARM_VOLT              Replace battery or adjust :ref:`BATT_ARM_VOLT<BATT_ARM_VOLT>`
    Battery capacity failsafe critical >= low               Battery failsafe misconfiguration                   Check :ref:`BATT_LOW_MAH<BATT_LOW_MAH>` is higher than :ref:`BATT_CRT_MAH<BATT_CRT_MAH>`
    Battery critical capacity failsafe                      Battery capacity is below BATT_CRT_MAH              Replace battery or adjust :ref:`BATT_CRT_MAH<BATT_CRT_MAH>`
    Battery critical voltage failsafe                       Battery voltage is below BATT_CRT_VOLT              Replace battery or adjust :ref:`BATT_CRT_VOLT<BATT_CRT_VOLT>`
    Battery low capacity failsafe                           Battery capacity is below BATT_LOW_MAH              Replace battery or adjust :ref:`BATT_LOW_MAH<BATT_LOW_MAH>`
    Battery low voltage failsafe                            Battery voltage is below BATT_LOW_VOLT              Replace battery or adjust :ref:`BATT_LOW_VOLT<BATT_LOW_VOLT>`
    Battery unhealthy                                       Battery is not providing data                       Check battery monitors physical connect and :ref:`configuration <common-powermodule-landingpage>`
    Battery voltage failsafe critical >= low                Battery failsafe misconfiguration                   Check :ref:`BATT_LOW_VOLT<BATT_LOW_VOLT>` is higher than :ref:`BATT_CRT_VOLT<BATT_CRT_VOLT>`
    BendyRuler OA requires reboot                           Object avoidance config change requires reboot      Reboot autopilot.  See :ref:`Object Avoidance configuration<common-object-avoidance-landing-page>`
    Board (Xv) out of range 4.3-5.8v                        Board voltage below BRD_VBUS_MIN or too high        Check power supply. If powering via USB plug in a battery or replace USB cable
    BTN_PINx=y invalid                                      Button misconfigured                                BTNx_PIN is set to an invalid value. Check :ref:`Button setup instructions <common-buttons>`
    BTN_PINx=y, set SERVOz_FUNCTION=-1                      Button misconfigured                                Set SERVOz_FUNCTION to -1
    Can't check rally without position                      EKF does not have a position estimate yet           Wait or move to location with better GPS reception
    Check fence                                             Fence feature has failed to be initialised          Reboot autopilot
    Check mag field (xy diff:x>875)                         Compass horiz field strength is too large or small  Relocate vehicle away from metal in the environment.  Move compass away from metal in the frame.  repeat :ref:`compass calibration <common-compass-calibration-in-mission-planner>`.  Disable internal compass.
    Check mag field (z diff:x>875)                          Compass vert field strength is too large or small   Relocate vehicle away from metal in the environment.  Move compass away from metal in the frame.  repeat :ref:`compass calibration <common-compass-calibration-in-mission-planner>`.  Disable internal compass.
    Check mag field: x, max y, min z                        Compass field strength is too large or small        Relocate vehicle away from metal in the environment.  Move compass away from metal in the frame.  repeat :ref:`compass calibration <common-compass-calibration-in-mission-planner>`.  Disable internal compass.
    Chute has no channel                                    Parachute misconfigured                             Parachute is controlled using PWM but no servo output function has been configured (e.g. need SERVOx_FUNCTION = 27). see :ref:`Parachute setup instructions <common-parachute>`
    Chute has no relay                                      Parachute misconfigured                             Parachute is controlled via Relay but no relay output has been configured. see :ref:`Parachute setup instructions <common-parachute>`
    Chute is released                                       Parachute has been released                         Reboot autopilot
    Compass calibrated requires reboot                      Autopilot must be rebooted after compass cal        Reboot autopilot
    Compass calibration running                             Compass calibration is running                      Complete or cancel the :ref:`compass calibration <common-compass-calibration-in-mission-planner>`
    Compass not healthy                                     At least one compass is not providing data          Check compass's connection to autopilot and :ref:`configuration <common-positioning-landing-page>`
    Compass offsets too high                                Compass offset params are too large                 Relocate compass away from metal in the frame and repeat :ref:`compass calibration <common-compass-calibration-in-mission-planner>`.  Disable internal compass.  Increase :ref:`COMPASS_OFFS_MAX <COMPASS_OFFS_MAX>`.
    Compasses inconsistent                                  Two compasses angles or field strength disagree     Check compass orientations (e.g. :ref:`COMPASS_ORIENT <COMPASS_ORIENT>`). Move compass away from metal in the frame.  repeat :ref:`compass calibration <common-compass-calibration-in-mission-planner>`.  Disable internal compass.
    Dijkstra OA requires reboot                             Object avoidance config change requires reboot      Reboot autopilot.  See :ref:`Object Avoidance configuration<common-object-avoidance-landing-page>`
    Disarm Switch on                                        Disarm auxiliary switch is in the high position     Move Disarm switch to the low position or check :ref:`auxiliary functions <common-auxiliary-functions>` setup
    Downloading logs                                        Vehicle cannot be armed while logs are downloading  Wait until logs are downloaded, cancel download or reboot autopilot
    DroneCAN: Duplicate Node x../y!                         DroneCAN sees same node ids used by two devices     Clear DroneCAN DNS server by setting :ref:`CAN_D1_UC_OPTION<CAN_D1_UC_OPTION>` = 1 and reboot
    DroneCAN: Failed to access storage!                     Possible hardware issue                             Reboot autopilot
    DroneCAN: Failed to add Node x!                         DroneCAN could not init connection to a device      Check sensor's physical connection and power supply
    DroneCAN: Node x unhealthy!                             A DroneCAN device is not providing data             Check sensor's physical connection and power supply
    Duplicate Aux Switch Options                            Two auxiliary function switches for same feature    Check :ref:`auxiliary function<common-auxiliary-functions>` setup.  Check for :ref:`RCx_OPTION<RC1_OPTION>` parameters with same values
    EKF3 Roll/Pitch inconsistent by x degs                  Roll or Pitch lean angle estimates are inconsistent Normally due to EKF3 not getting good enough GPS accuracy, but could be due to other sensors producing errors. Go outdoors, wait or reboot autopilot.
    EKF3 waiting for GPS config data                        automatic GPS configuration has not completed       Check GPS connection and configuration especially if using DroneCAN GPS
    EKF3 Yaw inconsistent by x degs                         Yaw angle estimates are inconsistent                Wait or reboot autopilot
    Failed to open mission.stg                              Failed to load mission from SD Card                 Check SD card.  Try to re-save mission to SD card
    Fence requires position                                 If fences are enabled, position estimate required   Wait or move vehicle to a location with a clear view of the sky.  Reduce sources of radio-frequency interference
    FENCE_ALT_MAX < FENCE_ALT_MIN                           FENCE_ALT_MAX must be greater than FENCE_ALT_MIN    Increase :ref:`FENCE_ALT_MAX<FENCE_ALT_MAX>` or lower :ref:`FENCE_ALT_MIN<FENCE_ALT_MIN>`
    FENCE_MARGIN is less than FENCE_RADIUS                  FENCE_MARGIN must be larger than FENCE_RADIUS       Increase :ref:`FENCE_RADIUS<FENCE_RADIUS>` or reduce :ref:`FENCE_MARGIN<FENCE_MARGIN>`
    FENCE_MARGIN too big                                    FENCE_ALT_MAX - FENCE_ALT_MIN < 2x FENCE_MARGIN     Decrease :ref:`FENCE_MARGIN<FENCE_MARGIN>` or increase difference between :ref:`FENCE_ALT_MAX<FENCE_ALT_MAX>` and :ref:`FENCE_ALT_MIN<FENCE_ALT_MIN>`
    Fences enabled, but none selected                       Fences are enabled but none are defined             Disable some or all fences using :ref:`FENCE_ENABLE<FENCE_ENABLE>` or :ref:`FENCE_TYPE<FENCE_TYPE>` or define the missing fences
    Fences invalid                                          Polygon fence is invalid                            Check polygon fence does not have overlapping lines
    FETtec: Invalid motor mask                              FETtec misconfiguration                             See :ref:`FETtec configuration<common-fettec-onewire>`
    FETtec: Invalid pole count x                            FETtec misconfiguration                             See :ref:`FETtec configuration<common-fettec-onewire>`
    FETtec: No uart                                         FETtec misconfiguration                             See :ref:`FETtec configuration<common-fettec-onewire>`
    FETtec: Not initialised                                 FETtec ESCs are not communicating with autopilot    See :ref:`FETtec configuration<common-fettec-onewire>`
    FETtec: x of y ESCs are not running                     FETtec ESCs are not spinning                        See :ref:`FETtec configuration<common-fettec-onewire>`
    FETtec: x of y ESCs are not sending telem               FETtec ESCs are not communicating with autopilot    See :ref:`FETtec configuration<common-fettec-onewire>`
    FFT calibrating noise                                   FFT Harmonic Notch analysis has not completed       Wait until :ref:`In-Flight FFT analysis<common-imu-fft>` completes
    FFT config MAXHZ xHz > yHz                              FFT Harmonic Notch misconfiguration                 See :ref:`In-Flight FFT Harmonic Notch Setup<common-imu-fft>` completes
    FFT self-test failed, max error Hz                      FFT Harmonic Notch failed                           See :ref:`In-Flight FFT Harmonic Notch Setup<common-imu-fft>` completes
    FFT still analyzing                                     FFT Harmonic Notch analysis has not completed       Wait until :ref:`In-Flight FFT analysis<common-imu-fft>` completes
    FFT: calibrated xHz/xHz/xHz                             FFT Harmonic Notch issue                            See :ref:`In-Flight FFT Harmonic Notch Setup<common-imu-fft>` completes
    FFT: resolution is xHz, increase length                 FFT Harmonic Notch misconfiguration                 See :ref:`In-Flight FFT Harmonic Notch Setup<common-imu-fft>` completes
    Generator: Not healthy                                  Generator is not communicating with autopilot       Check :ref:`generator configuration <common-generators>`
    Generator: No backend driver                            Firmware does not include seelected generator       Build version of firmware with desired generator using custom.ardupilot.org
    GPS and AHRS differ by Xm                               GPS and EKF positions differ by at least 10m        Wait until GPS quality improves.  Move vehicle to a location with a clear view of the sky.  Reduce sources of radio-frequency interference
    GPS blending unhealthy                                  At least one GPS is not providing good data         Move vehicle to a location with a clear view of the sky.  Reduce sources of radio-frequency interference. Check :ref:`GPS blending configuration <common-gps-blending>`
    GPS Node x not set as instance y                        DroneCan GPS configuration error                    Check :ref:`GPS1_CAN_NODEID<GPS1_CAN_NODEID>` and :ref:`GPS2_CAN_NODEID<GPS2_CAN_NODEID>`
    GPS positions differ by Xm                              Two GPSs reported positions differ by 50m or more   Wait until GPS quality improves.  Move vehicle to a location with a clear view of the sky.  Reduce sources of radio-frequency interference
    GPS x still configuring this GPS                        Automatic GPS configuration has not completed       Wait until configuration completes.  Check GPS connection and configuration especially if using DroneCAN GPS
    GPS x: Bad fix                                          GPS does not have a good lock                       Move vehicle to a location with a clear view of the sky.  Reduce sources of radio-frequency interference
    GPS x: not healthy                                      GPS is not providing data                           Check GPSs physical connection to autopilot and  :ref:`configuration <common-positioning-landing-page>`
    GPS x: primary but TYPE 0                               Primary GPS has not been configured                 Check :ref:`GPS_PRIMARY<GPS_PRIMARY>` and confirm corresponding :ref:`GPS1_TYPE<GPS1_TYPE>` or :ref:`GPS2_TYPE<GPS2_TYPE>` matches type of GPS used
    GPS x: was not found                                    GPS disconnected or incorrectly configured          Check GPSs physical connection to autopilot and  :ref:`configuration <common-positioning-landing-page>`
    GPSx yaw not available                                  GPS-for-yaw configured but not working              Move to location with better GPS reception.  Check :ref:`GPS-for-yaw <common-gps-for-yaw>` configuration
    Gyros inconsistent                                      Two gyros are inconsistent by at least 5 deg/sec    Reboot autopilot and hold vehicle still until gyro calibration completes.  Allow autopilot to warm-up and reboot.  If failure continues replace autopilot
    Gyros not calibrated                                    Gyro calibration normally run at startup failed     Reboot autopilot and hold vehicle still until gyro calibration completes
    Gyros not healthy                                       At least one gyro is not providing data             Reboot autopilot.  If failure continues replace autopilot
    Hardware safety switch                                  Hardware safety switch has not been pushed          Push safety switch (normally on top of GPS) or disable by setting :ref:`BRD_SAFETY_DEFLT<BRD_SAFETY_DEFLT>` to zero and reboot autopilot
    heater temp low (x < 45)                                Board heater temp is below BRD_HEAT_TARG            Wait for board to heat up. Target temperature can be adjust using :ref:`BRD_HEAT_TARG<BRD_HEAT_TARG>`
    In OSD menu                                             OSD is being configured                             Complete OSD configuration.  Check :ref:`OSD configuration <common-osd-overview>`
    Internal errors 0x%x l:%u %s                            An internal error has occurred                      Reboot the autopilot. Report error to the development team
    Invalid FENCE_ALT_MAX value                             FENCE_ALT_MAX must be positive                      Increase :ref:`FENCE_ALT_MAX<FENCE_ALT_MAX>`
    Invalid FENCE_ALT_MIN value                             FENCE_ALT_MIN must be higher than -100              Increase :ref:`FENCE_ALT_MIN<FENCE_ALT_MIN>`
    Invalid FENCE_MARGIN value                              FENCE_MARGIN must be positive                       Increase :ref:`FENCE_MARGIN<FENCE_MARGIN>`
    Invalid FENCE_RADIUS value                              FENCE_RADIUS must be positive                       Increase :ref:`FENCE_RADIUS<FENCE_RADIUS>`
    Logging failed                                          Logs could not be written.  Maybe hardware failure  Reboot autopilot.  Replace autopilot
    Logging not started                                     Logs could not be written.  Maybe hardware failure  Reboot autopilot.  Replace autopilot
    Main loop slow (xHz < 400Hz)                            Autopilot's CPU is overloaded                       Wait to see if the error is temporary. Disable features or replace with a higher powered autopilot. Reduce :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>`
    Margin is less than inclusion circle radius             A circular fence has radius below FENCE_MARGIN      Increase the size of the circular fence involved or decrease :ref:`FENCE_MARGIN<FENCE_MARGIN>`
    memory low for auxiliary authorisation                  Autopilot has run out of memory                     Disable features or replace with a higher powered autopilot
    Missing mission item: do land start                     Auto mission needs a DO_LAND_START command          Add a DO_LAND_START command to the mission or adjust the :ref:`ARMING_MIS_ITEMS<ARMING_MIS_ITEMS>` parameter
    Missing mission item: land                              Auto mission needs a LAND command                   Add a LAND command to the mission or adjust the :ref:`ARMING_MIS_ITEMS<ARMING_MIS_ITEMS>` parameter
    Missing mission item: RTL                               Auto mission needs an RTL command                   Add an RTL command to the mission or adjust the :ref:`ARMING_MIS_ITEMS<ARMING_MIS_ITEMS>` parameter
    Missing mission item: takeoff                           Auto mission needs a TAKEOFF command                Add a TAKEOFF command to the mission or adjust the :ref:`ARMING_MIS_ITEMS<ARMING_MIS_ITEMS>` parameter
    Missing mission item: vtol land                         Auto mission needs a VTOL_LAND command              Add a VTOL_LAND command to the mission or adjust the :ref:`ARMING_MIS_ITEMS<ARMING_MIS_ITEMS>` parameter
    Missing mission item: vtol takeoff                      Auto mission needs a VTOL_TAKEOFF command           Add a VTOL_TAKEOFF command to the mission or adjust the :ref:`ARMING_MIS_ITEMS<ARMING_MIS_ITEMS>` parameter
    Mode channel and RCx_OPTION conflict                    RC flight mode switch also used for an aux function Change FLTMODE_CH (or MODE_CH for Rover) or RCx_OPTION to remove conflict
    Mode requires mission                                   Attempting to arm in Auto mode but no mission       Arm in an other mode or create and upload an Auto mission
    Motors Emergency Stopped                                Motors emergency stopped                            Release emergency stop. See :ref:`auxiliary functions <common-auxiliary-functions>`
    Mount: check TYPE                                       Mount (aka camera gimbal) misconfiguration          Check :ref:`MNT1_TYPE<MNT1_TYPE>` is valid. Check :ref:`Gimbal configuration<common-cameras-and-gimbals>`
    Mount: not healthy                                      Mount is not communicating with autopilot           Check physical connection between autopilot and gimbal and check :ref:`Gimbal configuration<common-cameras-and-gimbals>`
    Multiple SERIAL ports configured for RC input           RC misconfiguration                                 see :ref:`Multiple Radio Control Receivers <common-multiple-rx>`
    No mission library present                              Auto mission feature has been disabled              `Custom build server <https://custom.ardupilot.org/>`__ was probably used to produce the firmware without auto missions.  Rebuild with auto missions enabled
    No rally library present                                Rally point feature has been disabled               `Custom build server <https://custom.ardupilot.org/>`__ was probably used to produce the firmware without rally point.  Rebuild with Rally points included
    No SD card                                              SD card is corrupted or missing                     Format or replace SD card
    No sufficiently close rally point located               Rally points are further than RALLY_LIMIT_KM        Move :ref:`rally points <common-rally-points>` closer to vehicle's current location or increase :ref:`RALLY_LIMIT_KM<RALLY_LIMIT_KM>`
    OA requires reboot                                      Object avoidance config change requires reboot      Reboot autopilot.  See :ref:`Object Avoidance configuration<common-object-avoidance-landing-page>`
    OpenDroneID: ARM_STATUS not available                   OpenDroneID misconfiguration                        see :ref:`Remote ID configuration<common-remoteid>`
    OpenDroneID: operator location must be set              Operator location is not available                  see :ref:`Remote ID configuration<common-remoteid>`
    OpenDroneID: SYSTEM not available                       OpenDroneID misconfiguration                        see :ref:`Remote ID configuration<common-remoteid>`
    OpenDroneID: UA_TYPE required in BasicID                OpenDroneID misconfiguration                        see :ref:`Remote ID configuration<common-remoteid>`
    OSD_TYPE2 not compatible with first OSD                 OSD1 and OSD2 configurations are incompatible       Disable 2nd OSD (set :ref:`OSD_TYPE2<OSD_TYPE2>` to zero) or check :ref:`OSD configuration <common-osd-overview>`
    Param storage failed                                    Eeprom hardware failure                             Check power supply or replace autopilot
    PiccoloCAN: Servo x not detected                        PiccoloCAN misconfiguration or servo issue          Check :ref:`Currawong Velocity ESC setup instructions <common-velocity-can-escs>`
    Pin x disabled (ISR flood)                              Sensor connected to GPIO pin is rapidly changing    Check sensor attached to specified pin
    Pitch (RCx) is not neutral                              RC transmitter's pitch stick is not centered        Move RC pitch stick to center or repeat :ref:`radio calibration <common-radio-control-calibration>`
    Pitch radio max too low                                 RC pitch channel max below 1700                     Repeat the :ref:`radio calibration <common-radio-control-calibration>` procedure or increase :ref:`RC2_MAX<RC2_MAX>` above 1700
    Pitch radio min too high                                RC pitch channel min above 1300                     Repeat the :ref:`radio calibration <common-radio-control-calibration>` procedure or reduce :ref:`RC2_MIN<RC1_MIN>` below 1300
    PRXx: No Data                                           Proximity sensor is not providing data              Check proximity sensor physical connection and :ref:`configuration <common-proximity-landingpage>`
    PRXx: Not Connected                                     Proximity sensor is not providing data              Check proximity sensor physical connection and :ref:`configuration <common-proximity-landingpage>`
    Radio failsafe on                                       RC failsafe has triggered                           Turn on RC transmitter or check RC failsafe configuration
    Rangefinder x: Not Connected                            Rangefinder is not providing data                   Check rangefinder's physical connection to autopilot and :ref:`configuration <common-rangefinder-landingpage>`
    Rangefinder x: Not Detected                             Rangefinder is not providing data                   Check rangefinder's physical connection to autopilot and :ref:`configuration <common-rangefinder-landingpage>`
    RC calibrating                                          RC calibration is in progress                       Complete the :ref:`radio calibration <common-radio-control-calibration>` procedure
    RC not calibrated                                       RC calibration has not been done                    Complete the :ref:`radio calibration <common-radio-control-calibration>`. :ref:`RC3_MIN<RC3_MIN>` and :ref:`RC3_MAX<RC3_MAX>` must have been changed from their default values (1100 and 1900), and for channels 1 to 4, MIN value must be 1300 or less, and MAX value 1700 or more.
    RC not found                                            RC failsafe enabled but no RC signal                Turn on RC transmitter or check RC transmitters connection to autopilot. If operating with only a GCS, see :ref:`common-gcs-only-operation`
    RCx_MAX is less than RCx_TRIM                           RC misconfiguration                                 Adjust RCx_TRIM to be lower than RCx_MAX or repeat :ref:`radio calibration <common-radio-control-calibration>`
    RCx_MIN is greater than RCx_TRIM                        RC misconfiguration                                 Adjust RCx_TRIM to be higher than RCx_MIN or repeat :ref:`radio calibration <common-radio-control-calibration>`
    RELAYx_PIN=y invalid                                    Relay misconfigured                                 RELAYx_PIN is set to an invalid value. Check :ref:`Relay setup instructions <common-relay>`
    RELAYx_PIN=y, set SERVx_FUNCTION=-1                     Relay misconfigured                                 Set SERVOx_FUNCTION to -1
    RNGFNDx_PIN not set                                     Rangefinder misconfigured                           Set RNGFNDx_PIN to a non-zero value.  See :ref:`rangefinder configuration <common-rangefinder-landingpage>`
    RNGFNDx_PIN=y invalid                                   Rangefinder misconfigured                           RNGFNDx_PIN is set to an invalid value.  Check :ref:`rangefinder configuration <common-rangefinder-landingpage>`
    RNGFNDx_PIN=y, set SERVOx_FUNCTION=-1                   Rangefinder misconfigured                           Set SERVOx_FUNCTION to -1
    Roll (RCx) is not neutral                               RC transmitter's roll stick is not centered         Move RC roll stick to center or repeat :ref:`radio calibration <common-radio-control-calibration>`
    Roll radio max too low                                  RC roll channel max below 1700                      Repeat the :ref:`radio calibration <common-radio-control-calibration>` procedure or increase :ref:`RC1_MAX<RC1_MAX>` above 1700
    Roll radio min too high                                 RC roll channel min above 1300                      Repeat the :ref:`radio calibration <common-radio-control-calibration>` procedure or reduce :ref:`RC1_MIN<RC1_MIN>` below 1300
    RPMx_PIN not set                                        RPM sensor misconfigured                            Check RPMx_PIN value. Check :ref:`RPM setup instructions <common-rpm>`
    RPMx_PIN=y invalid                                      RPM sensor misconfigured                            RPMx_PIN is set to an invalid value. Check :ref:`RPM setup instructions <common-rpm>`
    RPMx_PIN=y, set SERVOx_FUNCTION=-1                      RPM sensor misconfigured                            Set SERVOz_FUNCTION to -1
    Same Node Id x set for multiple GPS                     DroneCan GPS configuration error                    Check :ref:`GPS1_CAN_NODEID<GPS1_CAN_NODEID>` and :ref:`GPS2_CAN_NODEID<GPS2_CAN_NODEID>` are different.  Set one to zero and reboot autopilot
    Same rfnd on different CAN ports                        Two rangefinders appearing on different CAN ports   Check USD1, TOFSensP, NanoRadar or Benewake setup instructions
    Scripting: loaded CRC incorrect want: x                 Script has incorrect CRC                            Replace Lua script with expected version
    Scripting: running CRC incorrect want: x                Script has incorrect CRC                            Replace Lua script with expected version
    Scripting: xxx failed to start                          A Lua script failed to start                        Autopilot out of memory or Lua script misconfiguration. See :ref:`Lua scripts<common-lua-scripts>`
    Scripting: xxx out of memory                            A Lua script ran out of memory                      Increase :ref:`SCR_HEAP_SIZE<SCR_HEAP_SIZE>` or check :ref:`Lua script configuration<common-lua-scripts>`
    Servo voltage to low (Xv < 4.3v)                        Servo rail voltage below 4.3V                       Check power supply to rear servo rail
    SERVOx_FUNCTION=y on disabled channel                   PWM output misconfigured                            SERVOx_FUNCTION is set for a servo output that has been disabled. See :ref:`BLHeli setup<common-blheli32-passthru>`
    SERVOx_MAX is less than SERVOx_TRIM                     PWM output misconfigured                            Set SERVOx_TRIM to be lower than SERVOx_MAX
    SERVOx_MIN is greater than SERVOx_TRIM                  PWM output misconfigured                            Set SERVOx_TRIM to be higher than SERVOx_MIN
    temperature cal running                                 Temperature calibration is running                  Wait until :ref:`temp calibration <common-imutempcal>` completes or reboot autopilot
    terrain disabled                                        Auto mission uses terrain but terrain disabled      Enable the terrain database (set :ref:`TERRAIN_ENABLE<TERRAIN_ENABLE>` = 1) or remove auto mission items that use terrain altitudes.  For Copters also check RTL_ALT_TYPE.
    Terrain out of memory                                   Autopilot has run out of memory                     Disable features or replace with a higher powered autopilot
    Throttle (RCx) is not neutral                           RC transmitter's throttle stick is too high         Lower throttle stick or repeat :ref:`radio calibration <common-radio-control-calibration>`
    Throttle radio max too low                              RC throttle channel max below 1700                  Repeat the :ref:`radio calibration <common-radio-control-calibration>` procedure or increase :ref:`RC2_MAX<RC2_MAX>` above 1700
    Throttle radio min too high                             RC throttle channel min above 1300                  Repeat the :ref:`radio calibration <common-radio-control-calibration>` procedure or reduce :ref:`RC1_MIN<RC1_MIN>` below 1300
    Too many auxiliary authorisers                          More than 3 external systems controlling arming     Check external authorisation system
    vehicle outside fence                                   Vehicle is outside the fence                        Move vehicle within the fence
    VisOdom: not healthy                                    VisualOdometry sensor is not providing data         Check visual odometry physical connection and :ref:`configuration<common-non-gps-navigation-landing-page>`
    VisOdom: out of memory                                  Autopilot has run out of memory                     Disable features or replace with a higher powered autopilot
    VTOL Fwd Throttle iz not zero                           RC transmitter's VTOL Fwd throttle stick is high    Lower VTOL Fwd throttle stick or repeat :ref:`radio calibration <common-radio-control-calibration>`
    waiting for terrain data                                Waiting for GCS to provide required terrain data    Wait or move to location with better GPS reception
    Yaw (RCx) is not neutral                                RC transmitter's yaw stick is not centered          Move RC yaw stick to center or repeat :ref:`radio calibration <common-radio-control-calibration>`
    Yaw radio max too low                                   RC yaw channel max below 1700                       Repeat the :ref:`radio calibration <common-radio-control-calibration>` procedure or increase :ref:`RC2_MAX<RC2_MAX>` above 1700
    Yaw radio min too high                                  RC yaw channel min above 1300                       Repeat the :ref:`radio calibration <common-radio-control-calibration>` procedure or reduce :ref:`RC1_MIN<RC1_MIN>` below 1300
    ======================================================= =================================================== ====================================================

[site wiki="copter"]

.. table:: Pre-Arm Failure Messages (Copter/Heli Only)
    :widths: auto

    ======================================================= =================================================== ====================================================
    Message                                                 Cause                                               Solution
    ======================================================= =================================================== ====================================================
    ADSB threat detected                                    ADSB failsafe.  Manned vehicles nearby              See :ref:`ADSB configuration<common-ads-b-receiver>`
    AHRS not healthy                                        AHRS/EKF is not yet ready                           Wait.  Reboot autopilot
    Altitude disparity                                      Barometer and EKF altitudes differ by at least 1m   Wait for EKF altitude to stabilise.  Reboot autopilot
    Auto mode not armable                                   Vehicle cannot be armed in Auto mode                Switch to another mode (like Loiter) or set :ref:`RTL_OPTIONS<RTL_OPTIONS>` = 3. See :ref:`Auto mode<auto-mode>`
    Bad parameter: ATC_ANG_PIT_P must be > 0                Attitude controller misconfiguration                Increase specified parameter value to be above zero. See :ref:`Tuning Process Instructions<tuning-process-instructions>`
    Bad parameter: PSC_POSXY_P must be > 0                  Position controller misconfiguration                Increase specified parameter value to be above zero. See :ref:`Tuning Process Instructions<tuning-process-instructions>`
    Battery failsafe                                        Battery failsafe has triggered                      Plug in battery and check its voltage and capacity.  See ref:`battery failsafe configuration <failsafe-battery>`
    Check ACRO_BAL_ROLL/PITCH                               ACRO_BAL_ROLL or ACRO_BAL_PITCH too high            Reduce :ref:`ACRO_BAL_ROLL<ACRO_BAL_ROLL>` below :ref:`ATC_ANG_RLL_P<ATC_ANG_RLL_P>` and/or :ref:`ACRO_BAL_PITCH<ACRO_BAL_PITCH>` below :ref:`ATC_ANG_PIT_P<ATC_ANG_RLL_P>`.  See :ref:`Acro mode<acro-mode>`
    Check ANGLE_MAX                                         ANGLE_MAX set too high                              Reduce :ref:`ANGLE_MAX<ANGLE_MAX>` to 8000 (e.g. 80 degrees) or lower
    Check FS_THR_VALUE                                      RC failsafe misconfiguration                        Set :ref:`FS_THR_VALUE<FS_THR_VALUE>` between 910 and RC throttle's min (e.g :ref:`RC3_MIN<RC3_MIN>`. See ref:`battery failsafe configuration <failsafe-battery>`
    Check PILOT_SPEED_UP                                    PILOT_SPEED_UP set too low                          Increase :ref:`PILOT_SPEED_UP<PILOT_SPEED_UP>` to a positive number (e.g. 100 = 1m/s).  See :ref:`AltHold mode<altholdmode>`
    Collective below failsafe (TradHeli only)               RC collective input is below FS_THR_VALUE           Turn on RC transmitter or check :ref:`FS_THR_VALUE<FS_THR_VALUE>`.  Check :ref:`RC failsafe setup<radio-failsafe>`
    EKF attitude is bad                                     EKF does not have a good attitude estimate          Wait for EKF attitude to stabilize.  Reboot autopilot.  Replace autopilot
    EKF compass variance                                    Compass direction appears incorrect                 Relocate vehicle away from metal in the environment.  Move compass away from metal in the frame.  repeat :ref:`compass calibration <common-compass-calibration-in-mission-planner>`.  Disable internal compass.
    EKF height variance                                     Barometer values are unstable or high vibration     Wait.  :ref:`Measure vibration <common-measuring-vibration>` and add :ref:`vibration isolation <common-vibration-damping>`
    EKF position variance                                   GPS position is unstable                            Wait.  If indoors, go outside.  Eliminate radio-frequency sources that could interfere with the GPS
    EKF velocity variance                                   GPS or optical flow velocities are unstable         Wait.  If indoors, go outside.  Eliminate radio-frequency sources that could interfere with the GPS.  Check :ref:`optical flow calibration <common-optical-flow-sensor-setup>`
    Fence enabled, need position estimate                   Fence is enabled so need a position estimate        Wait.  If indoors, go outside.  Ensure compass and accelerometer calibrations have been completed.  Eliminate radio-frequency sources that could interfere with the GPS. See :ref:`Fence configuration<common-geofencing-landing-page>`
    FS_GCS_ENABLE=2 removed, see FS_OPTIONS                 GCS failsafe misconfiguration                       Set :ref:`FS_GCS_ENABLE<FS_GCS_ENABLE>` = 1 and check :ref:`FS_OPTIONS<FS_OPTIONS>` parameter. See ref:`GCS Failsafe configuration<gcs-failsafe>`
    GCS failsafe on                                         GCS failsafe has triggered                          Check telemetry connection. See ref:`GCS Failsafe configuration<gcs-failsafe>`
    GPS glitching                                           GPS position is unstable                            Wait.  If indoors, go outside.  Eliminate radio-frequency sources that could interfere with the GPS
    High GPS HDOP                                           GPS horizontal quality too low                      Wait or move to location with better GPS reception. You may raise :ref:`GPS_HDOP_GOOD<GPS_HDOP_GOOD>` but this is rarely a good idea
    Home too far from EKF origin                            Home is more than 50km from EKF origin              Reboot autopilot to reset EKF origin to current Location
    Interlock/E-Stop Conflict (TradHeli only)               Incompatible auxiliary function switch configured   Remove Interlock, E-Stop or Emergency Stop from :ref:`auxiliary function<common-auxiliary-functions>` setup
    Invalid MultiCopter FRAME_CLASS                         FRAME_CLASS parameter misconfigured                 Multicopter firmware loaded but :ref:`FRAME_CLASS<FRAME_CLASS>` set to helicopter.  Load helicopter firmware or change :ref:`FRAME_CLASS<FRAME_CLASS>`
    Inverted flight option not supported                    Inverted flight auxiliary function not supported    Remove :ref:`auxiliary function<common-auxiliary-functions>` switch for inverted flight
    Leaning                                                 Vehicle is leaning more than ANGLE_MAX              Level vehicle
    Motor Interlock Enabled                                 Motor Interlock in middle or high position          Move motor interlock :ref:`auxiliary function<common-auxiliary-functions>` switch to low position
    Motor Interlock not configured                          Helicopters require motor interlock be configured   Enable the motor interlock :ref:`auxiliary function<common-auxiliary-functions>` switch
    Motors: Check frame class and type                      Unknown or misconfigured frame class or type        Enter valid frame class and/or type
    Motors: Check MOT_PWM_MIN and MOT_PWM_MAX               MOT_PWM_MIN or MOT_PWM_MAX misconfigured            Set :ref:`MOT_PWM_MIN<MOT_PWM_MIN>` = 1000 and :ref:`MOT_PWM_MAX<MOT_PWM_MAX>` = 2000 and repeat the :ref:`ESC calibration <esc-calibration>`
    Motors: MOT_SPIN_ARM > MOT_SPIN_MIN                     MOT_SPIN_ARM is too high or MOT_SPIN_MIN is too low Reducse :ref:`MOT_SPIN_ARM<MOT_SPIN_ARM>` to below :ref:`MOT_SPIN_MIN<MOT_SPIN_MIN>`.  See :ref:`Seting motor range<set-motor-range>`
    Motors: MOT_SPIN_MIN too high x > 0.3                   MOT_SPIN_MIN parameter value is too high            Reduce :ref:`MOT_SPIN_MIN<MOT_SPIN_MIN>` to below 0.3.  See :ref:`Setting motor range<set-motor-range>`
    Motors: no SERVOx_FUNCTION set to MotorX                At least one motor output has not been configured   Check SERVOx_FUNCTION values for "Motor1", "Motor2", etc.  Check :ref:`ESC and motor configuration <connect-escs-and-motors>`
    Need Alt Estimate                                       EKF has not yet calculated the altitude             Wait.  Allow autopilot to heat up.  Ensure :ref:`accelerometer calibration <common-accelerometer-calibration>` has been done.
    Need Position Estimate                                  EKF does not have a position estimate               Wait.  If indoors, go outside.  Ensure compass and accelerometer calibrations have been completed.  Eliminate radio-frequency sources that could interfere with the GPS
    Proximity x deg, ym (want > Zm)                         Obstacles too close to vehicle                      Move obstacles away from vehicle or check sensor.  See :ref:`proximity sensor configuration <common-proximity-landingpage>`
    RTL mode not armable                                    Vehicle cannot be armed in RTL mode                 Switch to another flight mode
    RTL_ALT_TYPE is above-terrain but no rangefinder        RTL uses rangefinder but rangefinder unavailable    Check :ref:`rangefinder configuration<common-rangefinder-landingpage>` including RNGFNDx_ORIENT=251
    RTL_ALT_TYPE is above-terrain but no terrain data       RTL uses terrain but Terrain database unavailable   Set :ref:`TERRAIN_ENABLE<TERRAIN_ENABLE>` = 1.  See :ref:`Terrain Following<terrain-following>`
    RTL_ALT_TYPE is above-terrain but RTL_ALT>RNGFND_MAX_CM RTL return altitude above rangefinder range         Reduce :ref:`RTL_ALT<RTL_ALT>` to less than RNGFNDx_MAX_CM. See :ref:`Terrain Following<terrain-following>`
    Safety Switch                                           Hardware safety switch has not been pushed          Push safety switch (normally on top of GPS) or disable by setting :ref:`BRD_SAFETY_DEFLT<BRD_SAFETY_DEFLT>` to zero and reboot autopilot
    Throttle below failsafe                                 RC throttle input is below FS_THR_VALUE             Turn on RC transmitter or check :ref:`FS_THR_VALUE<FS_THR_VALUE>`.  Check :ref:`RC failsafe setup<radio-failsafe>`
    Vehicle too far from EKF origin                         Vehicle is more than 50km from EKF origin           Reboot autopilot to reset EKF origin to current Location
    winch unhealthy                                         Winch is not communicating with autopilot           Check winch's physical connection and :ref:`configuration <common-daiwa-winch>`
    ======================================================= =================================================== ====================================================
[/site]
[site wiki="plane"]

.. table:: Pre-Arm Failure Messages (Plane Only)
    :widths: auto

    ======================================================= =================================================== ====================================================
    Message                                                 Cause                                               Solution
    ======================================================= =================================================== ====================================================
    ADSB threat detected                                    ADSB failsafe.  Manned vehicles nearby              See :ref:`ADSB configuration<common-ads-b-receiver>`
    AHRS not healthy                                        AHRS/EKF is not yet ready                           Wait.  Reboot autopilotFS_LONG_TIMEOUT < FS_SHORT_TIMEOUT
    AIRSPEED_MIN too low x<5 m/s                            Parameter set too low, under 5m/s                   Raise to at least 20% above stall speed
    Bad parameter: ATC_ANG_PIT_P must be > 0                Attitude controller misconfiguration                Increase specified parameter value to be above zero. See :ref:`Tuning Process Instructions<tuning-process-instructions>`
    Bad parameter: PSC_POSXY_P must be > 0                  Position controller misconfiguration                Increase specified parameter value to be above zero. See :ref:`Tuning Process Instructions<tuning-process-instructions>`
    Check Q_ANGLE_MAX                                       Set above 80 degrees                                Reduce :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>` below 80; 30 degrees is typical
    In landing sequence                                     Trying to arm while still in landing sequence       Reset mission;change to mission item not in a landing sequence
    Invalid THR_FS_VALUE for reversed throttle input        THR_FS_VALUE pwm is not ABOVE the max throttle      Set :ref:`THR_FS_VALUE<THR_FS_VALUE>` above throttle maximum pwm
    ROLL_LIMIT_DEG too small x                              Parameter set under 3 degrees                       Increase, 45 deg recommended minimum for adequate control
    PTCH_LIM_MAX_DEG too small x                            Parameter set under 3 degrees                       Increase, 45 deg recommended minimum for adequate control
    PTCH_LIM_MIN_DEG too large x                            Parameter set over 3 degrees                        Increase, 45 deg recommended minimum for adequate control
    Mode not armable                                        Cannot arm from this mode                           Change Mode
    Mode not QMODE                                          Q_OPTION set to prevent arming except in QMODE/AUTO Change Mode or reset :ref:`Q_OPTIONS<Q_OPTIONS>` bit 18
    Motors: Check frame class and type                      Unknown or misconfigured frame class or type        Enter valid frame class and/or type
    Motors: Check MOT_PWM_MIN and MOT_PWM_MAX               MOT_PWM_MIN or MOT_PWM_MAX misconfigured            Set :ref:`MOT_PWM_MIN<MOT_PWM_MIN>` = 1000 and :ref:`MOT_PWM_MAX<MOT_PWM_MAX>` = 2000 and repeat the :ref:`ESC calibration <esc-calibration>`
    Motors: MOT_SPIN_ARM > MOT_SPIN_MIN                     MOT_SPIN_ARM is too high or MOT_SPIN_MIN is too low Reducse :ref:`MOT_SPIN_ARM<MOT_SPIN_ARM>` to below :ref:`MOT_SPIN_MIN<MOT_SPIN_MIN>`.  See :ref:`Seting motor range<set-motor-range>`
    Motors: MOT_SPIN_MIN too high x > 0.3                   MOT_SPIN_MIN parameter value is too high            Reduce :ref:`MOT_SPIN_MIN<MOT_SPIN_MIN>` to below 0.3.  See :ref:`Setting motor range<set-motor-range>`
    Motors: no SERVOx_FUNCTION set to MotorX                At least one motor output has not been configured   Check SERVOx_FUNCTION values for "Motor1", "Motor2", etc.  Check :ref:`ESC and motor configuration <connect-escs-and-motors>`
    Q_ASSIST_SPEED is not set                               Q_ASSIST_SPEED has not been set                     Set :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>`, See :ref:`assisted_fixed_wing_flight`
    Quadplane enabled but not running                       Q_ENABLE is set, but QuadPlane code has not started Reboot
    quadplane needs SCHED_LOOP_RATE >= 100                  Quadplane needs faster loop times for performance   Increase :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>`; 300 is typical
    set Q_FWD_THR_USE to 0                                  Trying to use FWD Throttle on Tailsitter            Set :ref:`Q_FWD_THR_USE<Q_FWD_THR_USE>` = 0 on tailsitters
    set TAILSIT_ENABLE 0 or TILT_ENABLE 0                   Cannot have simultaneous tiltrotor and tailsitter   Pick one to match your vehicle configuration and reboot
    tailsitter setup not complete, reboot                   Enabled tiltrotor but have not rebooted yet         Reboot
    tiltrotor setup not complete, reboot                    Enabled tailsitter but have not rebooted yet        Reboot
    Throttle trim not near center stick %x                  RC trim for centered throttle stick use incorrect   Set throttle channels RC trim to center position (idle) if :ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>` bit 10 is set.
    Waiting for RC                                          RC failsafe enabled but no RC signal                Turn on RC transmitter or check RC transmitters connection to autopilot. If operating with only a GCS, see :ref:`common-gcs-only-operation`
    ======================================================= =================================================== ====================================================
[/site]
Other Failure messages
======================

Failsafes:
----------

Any failsafe (RC, Battery, GCS,etc.) will display a message and prevent arming.


Barometer failures:
-------------------

**Baro not healthy** : the barometer sensor is reporting that it is
unhealthy which is normally a sign of a hardware failure.

**Alt disparity** : the barometer altitude disagrees with the inertial
navigation (i.e. Baro + Accelerometer) altitude estimate by more than 1
meters.  This message is normally short-lived and can occur when the
autopilot is first plugged in or if it receives a hard jolt
(i.e. dropped suddenly).  If it does not clear the :ref:`accelerometers may need to be calibrated <common-accelerometer-calibration>` or there may
be a barometer hardware issue.

Compass failures:
-----------------

**Compass not healthy** : the compass sensor is reporting that it is
unhealthy which is a sign of a hardware failure.

**Compass not calibrated** : the :ref:`compass(es) has not been calibrated <common-compass-calibration-in-mission-planner>`.  the
``COMPASS_OFS_X, _Y, _Z`` parameters are zero or the number or type of
compasses connected has been changed since the last compass calibration
was performed.

**Compass offsets too high** : the primary compass's offsets length
(i.e. sqrt(x^2+y^2+z^2)) are larger than 500.  This can be caused by
metal objects being placed too close to the compass.  If only an
internal compass is being used (not recommended), it may simply be the
metal in the board that is causing the large offsets and this may not
actually be a problem in which case you may wish to disable the compass
check.

**Check mag field** : This can result from failing two different checks. First, the sensed magnetic field in the area is 35%
higher or  lower than the expected value.  The expected length is 530 so
it's > 874 or < 185. Also, besides this rough check, when the vehicle's position has been obtained (GPS lock), another check against the field strength predicted using the internal World Magnetic Field database is done with much tighter limits. If you are failing this, either the :ref:`compass calibration <common-compass-calibration-in-mission-planner>` has not calculated good offsets and calibration should be repeated, or your vehicle is near a large metallic or magnetic disturbance and will need to be relocated.

**Compasses inconsistent** : the internal and external compasses are
pointing in different directions (off by >45 degrees).  This is normally
caused by the external compasses orientation (i.e. :ref:`COMPASS_ORIENT<COMPASS_ORIENT>`
parameter) being set incorrectly.

GPS related failures:
---------------------

**GPS Glitch** : the :ref:`GPS is glitching <gps-failsafe-glitch-protection>` and the vehicle
is in a flight mode that requires GPS (i.e. Loiter, PosHold, etc) and/or
the :ref:`cylindrical fence <common-ac2_simple_geofence>` is enabled.

**Need 3D Fix** : the GPS does not have a 3D fix and the vehicle is in a
flight mode that requires the GPS and/or the :ref:`cylindrical fence <common-ac2_simple_geofence>` is enabled.

**Bad Velocity** : the vehicle's velocity (according to inertial
navigation system) is above 50cm/s.  Issues that could lead to this
include the vehicle actually moving or being dropped, bad accelerometer
calibration, GPS updating at below the expected 5hz.

**High GPS HDOP** : the GPS's HDOP value (a measure of the position
accuracy) is above 2.0 and the vehicle is in a flight mode that requires
GPS and/or the :ref:`cylindrical fence <common-ac2_simple_geofence>` is enabled. 
This may be resolved by simply waiting a few minutes, moving to a
location with a better view of the sky or checking sources of GPS
interference (i.e. FPV equipment) are moved further from the GPS. 
Alternatively the check can be relaxed by increasing the :ref:`GPS_HDOP_GOOD<GPS_HDOP_GOOD>`
parameter to 2.2 or 2.5.  Worst case the pilot may disable the fence and
take-off in a mode that does not require the GPS (i.e. Stabilize,
AltHold) and switch into Loiter after arming but this is not
recommended.

Note: the GPS HDOP can be readily viewed through the Mission Planner's
Quick tab as shown below.

.. image:: ../../../images/MP_QuicHDOP.jpg
    :target: ../_images/MP_QuicHDOP.jpg

INS checks (i.e. Acclerometer and Gyro checks):
-----------------------------------------------

**INS not calibrated**: some or all of the accelerometer's offsets are
zero.  The :ref:`accelerometers need to be calibrated <common-accelerometer-calibration>`.

**Accels not healthy**: one of the accelerometers is reporting it is not
healthy which could be a hardware issue.  This can also occur
immediately after a firmware update before the board has been restarted.

**Accels inconsistent**: the accelerometers are reporting accelerations
which are different by at least 1m/s/s.  The :ref:`accelerometers need to be re-calibrated <common-accelerometer-calibration>` or there is a
hardware issue.

**Gyros not healthy**: one of the gyroscopes is reporting it is
unhealthy which is likely a hardware issue.  This can also occur
immediately after a firmware update before the board has been restarted.

**Gyro cal failed**: the gyro calibration failed to capture offsets. 
This is most often caused by the vehicle being moved during the gyro
calibration (when red and blue lights are flashing) in which case
unplugging the battery and plugging it in again while being careful not
to jostle the vehicle will likely resolve the issue.  Sensors hardware
failures (i.e. spikes) can also cause this failure.

**Gyros inconsistent**: two gyroscopes are reporting vehicle rotation
rates that differ by more than 20deg/sec.  This is likely a hardware
failure or caused by a bad gyro calibration.

Board Voltage checks:
---------------------

**Check Board Voltage**: the board's internal voltage is below 4.3 Volts
or above 5.8 Volts.

If powered through a USB cable (i.e. while on the bench) this can be
caused by the desktop computer being unable to provide sufficient
current to the autopilot - try replacing the USB cable.

If powered from a battery this is a serious problem and the power system
(i.e. Power Module, battery, etc) should be carefully checked before
flying.

Parameter checks:
-----------------

**Ch7&Ch8 Opt cannot be same**: :ref:`Auxiliary Function Switches <channel-7-and-8-options>` are set to the same option which is not permitted because it could lead to confusion.

**Check FS_THR_VALUE**: the :ref:`radio failsafe pwm value <radio-failsafe>` has been set too close to the throttle channels (i.e. ch3) minimum.

**Check ANGLE_MAX**: the :ref:`ANGLE_MAX<ANGLE_MAX>` parameter which controls the
vehicle's maximum lean angle has been set below 10 degrees (i.e. 1000)
or above 80 degrees (i.e. 8000).

**ACRO_BAL_ROLL/PITCH**: the :ref:`ACRO_BAL_ROLL<ACRO_BAL_ROLL>` parameter is higher than
the Stabilize Roll P and/or :ref:`ACRO_BAL_PITCH<ACRO_BAL_PITCH>` parameter is higher than
the Stabilize Pitch P value.  This could lead to the pilot being unable
to control the lean angle in ACRO mode because the :ref:`Acro Trainer stabilization <acro-mode_acro_trainer>` would overpower the pilot's
input.

Battery/Power Monitor:
----------------------

If a power monitor voltage is below its failsafe low or critical voltages or failsafe remaining capacity low or critical set points, this check will fail and indicate which set point it is below. It will also fail if these set points are inverted, ie critical point is higher than low point. See :ref:`failsafe-battery` for Copter, :ref:`apms-failsafe-function` for Plane, or :ref:`rover-failsafes` for Rover for more information on these.

In addition, minimum arming voltage and remaining capacity parameters for each battery/power monitor can be set, for example :ref:`BATT_ARM_VOLT<BATT_ARM_VOLT>` and :ref:`BATT_ARM_MAH<BATT_ARM_MAH>` for the first battery, to provide a check that the battery is not only above failsafe levels, but also has enough capacity for operation.

Airspeed:
---------

If an airspeed sensor is configured, and it is not providing a reading or failed to calibrate, this check will fail.

**Airspeed not healthy**

Logging:
--------

**Logging failed**: Logging pre-armed was enabled but failed to write to the log.

**No SD Card**: Logging is enabled, but no SD card is detected.

Safety Switch:
--------------

**Hardware safety switch**: Hardware safety switch has not been pushed.

System:
-------

**Param storage failed**: A check of reading the parameter storage area failed.

**Internal errors (0xx)**: An internal error has occurred. Report to ArduPilot development team `here <https://github.com/ArduPilot/ardupilot/issues/15916>`_

**KDECAN Failed**: KDECAN system failure.

**DroneCAN Failed**: DroneCAN system failure.

Mission:
--------

See :ref:`ARMING_MIS_ITEMS<ARMING_MIS_ITEMS>`

**No mission library present**: Mission checking is enabled, but no mission is loaded.

**No rally library present**: Rally point checking is enabled, but no rally points loaded.

**Missing mission item: xxxx**: A required mission items is missing.


Rangefinder:
------------

IF a rangefinder has been configured, a reporting error has occurred.

Disabling the Pre-arm Safety Check
==================================

.. warning:: Disabling pre-arm safety checks is not recommended. The cause of the pre-arm failure should be corrected before operation of the vehicle if at all possible. If you are confident that the pre-arm check failure is not a real problem, it is possible to disable a failing check.

Arming checks can be individually disabled by setting the :ref:`ARMING_CHECK<ARMING_CHECK>` parameter to something other than 1. Setting to 0 completely removes all pre-arm checks. For example, setting to  4 only checks that the GPS has lock.

This can also be configured using Mission Planner:

.. image:: ../../../images/MP_PreArmCheckDisable.png
    :target: ../_images/MP_PreArmCheckDisable.png


-  Connecting your Autopilot to the Mission Planner
-  Go to Mission Planner's Config/Tuning >> Standard Params screen
-  set the Arming Check drop-down to "Disabled" or one of the "Skip"
   options which more effectively skips the item causing the failure.
-  Push the "Write Params" button
