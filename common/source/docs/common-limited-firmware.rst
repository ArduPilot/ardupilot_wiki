.. _common-limited_firmware:

==========================================
Firmware Limitations on AutoPilot Hardware
==========================================

The ArduPilot firmware in some configurations exceeds 1 MB in size. Some autopilots may not have enough
flash memory to store the full firmware.

For the affected autopilots, a reduced firmware is generated. This firmware omits less-commonly used features
in order to reduce the firmware size to less than 1 MB.

The missing features are listed below. If you require any of these features, you can try to create a build with them in it (at the expense of other non-needed features) using the `Custom Firmware Build Server <https://custom.ardupilot.org>`__.

Common to all 1MB Boards
========================

Exclusions for: KakuteF4, Sparky2, OMNIBUSF7V2, KakuteF7, KakuteF7 Mini, older versions of the Pixhawk (with the RevA, RevY and Rev1 of the STM32F427 chip), BeastF4, F35Lightning, F4BY, MambaF405v2, MazzyStarDrone, OmnibusNanoV6, VRBrain-v51, VRBrain-v52, VRCore-v10, VRUBrain-v51, airbotf4, crazyflie2, mini-pix, revo-mini, skyviper, speedybeef4.

- ADSB
- EFI Controller Support
- External AHRS
- FETtec ESCs
- INA2XX Battery Monitors
- Plane Deep Stall Landing
- Torqeedo Motor Control
- MAVLink frame rate control from SD file
- INS Temperature Calibration
- SOLO Gimbal Support
- Visual Odometry
- Airspeed Drag Compensation
- Plus Code Location Support
- Object Avoidance using Proximity Sensors
- CRSF Text Display (Telemetry is included)
- Bootloader inclusion in main code base
- EKF2 (EKF3 only)
- LUA scripting support
- In-Flight FFT control of Harmonic Notch

Additional Exclusions
=====================

Additional Exclusions for: Sparky2, OMNIBUSF7V2, KakuteF7, KakuteF7 Mini and older versions of the Pixhawk (with the RevA, RevY and Rev1 of the STM32F427 chip)

   -  **Common to all vehicles**

      -  Aux function for testing IMU failover (KILL_IMU)
      -  LTM, MSP, CRSF, Spektrum, Devo and Hott telemetry formats
      -  Piccolo CAN
      -  Oreo LED lights
      -  NCP5623 LED lights
      -  NMEA output format
      -  Solo Gimbal
      -  In Flight FFT support
      -  MTK, SIRF GPS support
      -  AK09916 on ICM20948 compass
      -  Runcam Control
      -  External I2C barometers
      -  DLVR Airspeed sensors
      -  CAN Tester
      -  KDE CAN
      -  External AHRS
      -  Generator
      -  GPS moving baseline
      -  MSP Rangefinders
      -  Camera Mount Control
      -  OSD Parameter Editor
      -  OSD Scrolling Sidebars
      -  Button
      -  OAPathPlanner
       
   -  **Copter only**

      -  Sprayer
      -  Gripper
      -  RPM
      -  Guided, Follow, Sport, Guided_noGPS, SystemID, Zigzag and Autorotate modes
      -  Beacon
      -  Optical Flow

   -  **Plane Only**

      -  Gripper
      -  Soaring
      -  Landing Gear
      -  Qautotune mode
      -  Off-board Guided

   -  **Rover Only**

      -  N/A


   -  **Sub Only**

      -  N/A

Additional exclusions for: Matek F405, Matek F405-Wing/F405-SE, OmnibusF4/F4Pro

   -  SMBUS battery
   -  Parachute 
   -  Sprayer
   -  OAPathPlanner
   -  Generator
   -  Precision Landing
   -  Baro Wind Compensation *
   -  Gripper *

      * enabled in OmnibusF4/F4Pro

Also for MatekF405-Wing:

   -  Wind Compensation
   -  RunCam Control
   -  Spektrum Telem

Additional exclusions for: SuccexF4

   -  Parachute
   -  Sprayer


RAM Limitations
===============

There may be insufficient RAM available in some flight controllers to support all enabled firmware features. Some possible symptoms are:

- MAVFTP does not work. Parameter downloads to GCS are delayed starting while the GCS tries to establish a MAVFTP link which cannot be setup, and then proceed slowly to download parameters using the normal download method.
- Compass Calibration will not start
- Logging will not start
- Terrain downloading from GCS will not start

If this occurs, several possible options are available to allow temporary use of MAVFTP, download terrain tiles, and/or Compass Calibration. All require a reboot to take effect:

- Make sure IMU Batch Sampling (used for FFT analysis) is not running by setting :ref:`INS_LOG_BAT_MASK<INS_LOG_BAT_MASK>` = 0.
- Try temporarily disabling logging by setting :ref:`LOG_BACKEND_TYPE<LOG_BACKEND_TYPE>` to 0, then returning to 1 (default) after calibrating.
- Reduce the size of :ref:`LOG_FILE_BUFSIZE<LOG_FILE_BUFSIZE>` . However, lowering below 16KB can introduce small gaps in the log. This may be used temporarily to download terrain or calibrate compass.
- Disable Terrain Following temporarily by setting :ref:`TERRAIN_ENABLE<TERRAIN_ENABLE>` to 0.
- Disable SmartRTL on Copter by setting :ref:`SRTL_POINTS<SRTL_POINTS>` = 0.

[copywiki destination="plane,copter,rover,blimp"]
