.. _common-limited_firmware:

==========================================
Firmware Limitations on AutoPilot Hardware
==========================================

The ArduPilot firmware in some configurations exceeds 1 MB in size. Some autopilots may not have enough
flash memory to store the full firmware.

For the affected autopilots, a reduced firmware is generated. This firmware omits less-commonly used features
in order to reduce the firmware size to less than 1 MB.

The missing features are listed below.

.. note:: The new LUA scripting feature requires a 2MB board, so none of the boards below have that capability.

-  **Sparky2 and older versions of the Pixhawk (with the RevA, RevY and Rev1 of the STM32F427 chip)**

   -  **Common to all vehicles**

      -  Aux function for testing IMU failover (KILL_IMU)
      -  LTM, Devo and Hott telemetry formats
      -  Piccolo CAN
      -  Oreo LED lights
      -  NCP5623 LED lights
      -  NMEA output format
      -  Solo Gimbal
      -  DSP support
      -  MTK, SIRF GPS support
      -  EFI engine support
      -  AK09916 on ICM20948 compass
      -  Runcam
      -  External I2C barometers
      -  DLVR Airspeed sensors


   -  **Copter only**

      -  Sprayer
      -  Visual Odometry
      -  Gripper
      -  RPM
      -  ADSB
      -  Guided, Follow, Sport, SystemID, Zigzag and Autorotate modes
      -  Beacon
      -  OAPathPlanner
      -  Optical Flow


   -  **Plane Only**

      -  HIL
      -  Gripper
      -  Soaring
      -  Landing Gear
      -  Qautotune mode


   -  **Rover Only**

      -  N/A


   -  **Sub Only**

      -  N/A

-  **KakuteF7, KakuteF7 Mini, OmnibusF7V2, Matek F405, Matek F405-Wing/ F405-SE,OmnibusF4/ F4Pro** 

   -  SMBUS battery
   -  Parachute
   -  Sprayer


-  **SuccexF4**

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
