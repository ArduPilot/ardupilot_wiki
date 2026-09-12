.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.8 release and is currently in master ("latest")

[copywiki destination="plane,copter,rover,blimp,sub"]

New Board Support
=================
- JPilot-C, see https://github.com/ArduPilot/ardupilot_wiki/pull/7567
- SparkNavi Blue , see https://github.com/ArduPilot/ardupilot_wiki/pull/7643
- PrinciploT H7 Pi , see https://github.com/ArduPilot/ardupilot_wiki/pull/7694
- GPILOT P1, see https://github.com/ArduPilot/ardupilot_wiki/pull/7877
- SIYI UniFC 6 PICO, see https://github.com/ArduPilot/ardupilot_wiki/pull/7877
- SimpliFly H7, see https://github.com/ArduPilot/ardupilot_wiki/pull/7915
- AET-H743-Air, see https://github.com/ArduPilot/ardupilot_wiki/pull/7918
- CUAV-X25-MEGA, see https://github.com/ArduPilot/ardupilot_wiki/pull/7944
- Agam MegH7, see https://github.com/ArduPilot/ardupilot_wiki/pull/7974
- FlyFishRC F405, see https://github.com/ArduPilot/ardupilot_wiki/pull/7985
- Lectron Pi5, see https://github.com/ArduPilot/ardupilot_wiki/pull/7976
- HGLRC H743 EVO, see https://github.com/ArduPilot/ardupilot_wiki/pull/8016
- NWBlue Pro H757, see https://github.com/ArduPilot/ardupilot_wiki/pull/8017
- Tustin MACH, see https://github.com/ArduPilot/ardupilot_wiki/pull/8029

New Peripheral Support
======================
- Trimble PX-1 GSOF AHRS, see https://github.com/ArduPilot/ardupilot_wiki/pull/7566
- Kebni SensAItion IMU and INS sensors, see https://github.com/ArduPilot/ardupilot_wiki/pull/7337
- YARI DroneCAN GNSSs, see https://github.com/ArduPilot/ardupilot_wiki/pull/7785
- RPLidarS2, see https://github.com/ArduPilot/ardupilot_wiki/pull/7787
- LightwareGRF-250 I2C, see https://github.com/ArduPilot/ardupilot_wiki/pull/7795
- Aeron Systems PLX3 INS, see https://github.com/ArduPilot/ardupilot_wiki/pull/7750
- HC Robotics HCR-523 DroneCAN GPS/Compass, see https://github.com/ArduPilot/ardupilot_wiki/pull/7947

New Features
============

- Option to clear GCS RC overrides on RC stick input, see https://github.com/ArduPilot/ardupilot_wiki/pull/7880
- Accel and gyro consistency pre-arm checks now run concurrently, see https://github.com/ArduPilot/ardupilot_wiki/pull/7921
- EK3_OPTIONS bits for optical flow (terrain alt above rangefinder range, AGL Kalman filter for flow scaling), see https://github.com/ArduPilot/ardupilot_wiki/pull/7962
- MAV_CMD_DO_SET_MISSION_CURRENT can now reset DO_JUMP repeat counters without changing the current mission item, see https://github.com/ArduPilot/ardupilot_wiki/pull/7982
- Read-only MAVn_DEVID parameter identifying which port each MAVLink channel's parameter group belongs to, see https://github.com/ArduPilot/ardupilot_wiki/pull/8051
- I2C TFmini Plus lidar can now be powered down above the RNGFNDx_PWRRNG height above terrain, see https://github.com/ArduPilot/ardupilot_wiki/pull/8055
- ARSPD_TYPE = 20 allows a LUA script to provide the airspeed or differential pressure, see https://github.com/ArduPilot/ardupilot_wiki/pull/8057

[site wiki="plane"]
- Rangefinder engagement distance, see https://github.com/ArduPilot/ardupilot_wiki/pull/7559
- QRTL approach now descends gradually to RTL_ALTITUDE instead of stepping down to it, see https://github.com/ArduPilot/ardupilot_wiki/pull/8031
- Weathervaning is now active while navigating between VTOL waypoints in AUTO, see https://github.com/ArduPilot/ardupilot_wiki/pull/8058
- Q_RTL_PAUSE_TIME pauses above the landing point before the final VTOL descent, defaulting to 5s on tailsitters, see https://github.com/ArduPilot/ardupilot_wiki/pull/8059
- Automatic flap speed schedule reworked: target airspeed is used without needing an airspeed sensor, with AIRSPEED_CRUISE as the fallback, see https://github.com/ArduPilot/ardupilot_wiki/pull/8060
[/site]
[site wiki="copter"]
- Ability to abort flip using aux switch low, see https://github.com/ArduPilot/ardupilot_wiki/pull/7759
- Pilot yaw input now also gates during landing (LAND_REPOSITION), see https://github.com/ArduPilot/ardupilot_wiki/pull/7879
- MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET mission command (point gimbal at next waypoint with an offset), see https://github.com/ArduPilot/ardupilot_wiki/pull/7920
- Simple/Super Simple mode fix: rotation now correctly applied in more flight modes, including Drift mode and during Precision Landing reposition, see https://github.com/ArduPilot/ardupilot_wiki/pull/7967
- Tradheli: DDFP tail rotor moved to the RSC controller with independent H_TAIL_RAMP_TIME, see https://github.com/ArduPilot/ardupilot_wiki/pull/7979
- Flip mode rotation rate is now set by FLIP_RATE, with the flip timeout derived from it, see https://github.com/ArduPilot/ardupilot_wiki/pull/8049
[/site]
[site wiki="rover"]

[/site]
[site wiki="sub"]
- Remote (MAVLink) leak detection, see https://github.com/ArduPilot/ardupilot_wiki/pull/7593
- GUIDED mode accepts an acceleration target alongside position and velocity, see https://github.com/ArduPilot/ardupilot_wiki/pull/8053
[/site]
