.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.8 release and is currently in master ("latest")

[copywiki destination="plane,copter,rover,blimp,sub"]

New Board Support
=================
- JPilot-C, see https://github.com/ArduPilot/ardupilot_wiki/pull/7567
- VUAV-TinyV7, see https://github.com/ArduPilot/ardupilot_wiki/pull/7630
- SparkNavi Blue , see https://github.com/ArduPilot/ardupilot_wiki/pull/7643
- PilotGaeaSH7V1-bdshot, see https://github.com/ArduPilot/ardupilot_wiki/pull/7687
- PrinciploT H7 Pi , see https://github.com/ArduPilot/ardupilot_wiki/pull/7694
- SkyDroid-S3, see https://github.com/ArduPilot/ardupilot_wiki/pull/7791
- SaamPixV1_1, see https://github.com/ArduPilot/ardupilot_wiki/pull/7872
- CyberX-v10, see https://github.com/ArduPilot/ardupilot_wiki/pull/7872
- AMOV Flycore, see https://github.com/ArduPilot/ardupilot_wiki/pull/7872
- ORBITH743v2, see https://github.com/ArduPilot/ardupilot_wiki/pull/7872
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

[site wiki="plane"]
- Rangefinder engagement distance, see https://github.com/ArduPilot/ardupilot_wiki/pull/7559
[/site]
[site wiki="copter"]
- Ability to abort flip using aux switch low, see https://github.com/ArduPilot/ardupilot_wiki/pull/7759
- Pilot yaw input now also gates during landing (LAND_REPOSITION), see https://github.com/ArduPilot/ardupilot_wiki/pull/7879
- MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET mission command (point gimbal at next waypoint with an offset), see https://github.com/ArduPilot/ardupilot_wiki/pull/7920
- Simple/Super Simple mode fix: rotation now correctly applied in more flight modes, including Drift mode and during Precision Landing reposition, see https://github.com/ArduPilot/ardupilot_wiki/pull/7967
- Tradheli: DDFP tail rotor moved to the RSC controller with independent H_TAIL_RAMP_TIME, see https://github.com/ArduPilot/ardupilot_wiki/pull/7979
[/site]
[site wiki="rover"]

[/site]
[site wiki="sub"]
- Remote (MAVLink) leak detection, see https://github.com/ArduPilot/ardupilot_wiki/pull/7593
[/site]
