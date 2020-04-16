.. _common-limited_firmware:

==========================================
Firmware Limitations on AutoPilot Hardware
==========================================

The ArduPilot firmware in some configurations exceeds 1 MB in size. Some autopilots may not have enough
flash memory to store the full firmware.

For the affected autopilots, a reduced firmware is generated. This firmware omits less-commonly used features
in order to reduce the firmware size to less than 1 MB.

The missing features are listed below.


-  **KakuteF7, KakuteF7 Mini, OmnibusF7V2, sparky2 and older versions of the Pixhawk (with the RevA, RevY and Rev1 of the STM32F427 chip)**

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


-  **Matek F405**

   -  SMBUS battery
   -  Parachute
   -  Sprayer


-  **Matek F405-Wing**

   -  SMBUS battery
   -  Parachute
   -  Sprayer


-  **OmnibusF4 & F4Pro**

   -  SMBUS battery
   -  Parachute
   -  Sprayer


-  **SuccexF4**

   -  Parachute
   -  Sprayer
