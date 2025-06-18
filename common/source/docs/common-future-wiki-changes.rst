.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.7 release and is currently in master ("latest")

[copywiki destination="plane,copter,rover,blimp,sub"]

New Board Support
=================
- AeroFox H7 , see https://github.com/ArduPilot/ardupilot_wiki/pull/6446
- GEPRC TAKER H7 BT, see https://github.com/ArduPilot/ardupilot_wiki/pull/6450
- MicoAir743-AIO, see https://github.com/ArduPilot/ardupilot_wiki/pull/6511
- CrazyF405 ELRS, see https://github.com/ArduPilot/ardupilot_wiki/pull/6513
- MicoAir743v2, see https://github.com/ArduPilot/ardupilot_wiki/pull/6525
- ARK FPV, see https://github.com/ArduPilot/ardupilot_wiki/pull/6560
- DroneerF405, see https://github.com/ArduPilot/ardupilot_wiki/pull/6567
- Updates to BETAFPVF405 family of boards, see https://github.com/ArduPilot/ardupilot_wiki/pull/6578
- ZeroOneX6-Air/Air+, see https://github.com/ArduPilot/ardupilot_wiki/pull/6616
- SULIGH7 Reference Design, see https://github.com/ArduPilot/ardupilot_wiki/pull/6637
- SpeedyBee F405 AIO, see https://github.com/ArduPilot/ardupilot_wiki/pull/6664
- StellarH7V2, see https://github.com/ArduPilot/ardupilot_wiki/pull/6681
- StellarF4, https://github.com/ArduPilot/ardupilot_wiki/pull/6702
- StellarF4V2, https://github.com/ArduPilot/ardupilot_wiki/pull/6700
- Lumineer LUXF765-NDAA, see https://github.com/ArduPilot/ardupilot_wiki/pull/6711
- NarinFC-H7, see https://github.com/ArduPilot/ardupilot_wiki/pull/6611
- BrotherHobby H743, see https://github.com/ArduPilot/ardupilot_wiki/pull/6792
- BrotherHobby F405v3, see https://github.com/ArduPilot/ardupilot_wiki/pull/6799
- TBS LUCIDH7 Wing, see https://github.com/ArduPilot/ardupilot_wiki/pull/6802
- PixSurveyA2-IND, see https://github.com/ArduPilot/ardupilot_wiki/pull/6811
- CORVON405V2.1, see https://github.com/ArduPilot/ardupilot_wiki/pull/6816
- BrahmaF4, see https://github.com/ArduPilot/ardupilot_wiki/pull/6820
- JHEMCUF405Pro, see https://github.com/ArduPilot/ardupilot_wiki/pull/6853
- CORVON743V1, see https://github.com/ArduPilot/ardupilot_wiki/pull/6891
- UAV DEV 743-UM982 ,see https://github.com/ArduPilot/ardupilot_wiki/pull/6897

New Peripheral Support
======================
- MakeFlyEasy POS3 Dronecan GPS/Compass, see https://github.com/ArduPilot/ardupilot_wiki/pull/6434
- UltraMotion CAN Servos, see https://github.com/ArduPilot/ardupilot_wiki/pull/6442
- Torqeedo TorqLink controlled motors, see https://github.com/ArduPilot/ardupilot_wiki/pull/6584
- Battery Tags, see https://github.com/ArduPilot/ardupilot_wiki/pull/6893
- VimDrones DroneCAN peripherals, see https://github.com/ArduPilot/ardupilot_wiki/pull/6906

New Features
============
- Option to change Mounts to Neutral on rc failsafe, see https://github.com/ArduPilot/ardupilot_wiki/pull/6430
- Auto-resizing scripting heap if needed at runtime, see https://github.com/ArduPilot/ardupilot_wiki/pull/6432
- New Airspeed sensor bootup skip cal option, see see https://github.com/ArduPilot/ardupilot_wiki/pull/6706
- Gyro rate pre-arm, see https://github.com/ArduPilot/ardupilot_wiki/pull/6776
- Add iNav fonts to DisplayPort for DJI Goggles 3/N3, see https://github.com/ArduPilot/ardupilot_wiki/pull/6794
- Fence breach warning, see https://github.com/ArduPilot/ardupilot_wiki/pull/6814
- Add JUMP count option, see https://github.com/ArduPilot/ardupilot_wiki/pull/6818
- Enhanced DroneCAN battery hotswap operation, see https://github.com/ArduPilot/ardupilot_wiki/pull/6904
- Initializing PreArm warning, see https://github.com/ArduPilot/ardupilot_wiki/pull/6908
- Add MAV_CMD_DO_SET_ROI_LOCATION/_NONE commands, see https://github.com/ArduPilot/ardupilot_wiki/pull/6915
- Add --embed option to waf firmware build command, see https://github.com/ArduPilot/ardupilot_wiki/pull/6921
- Stream rates moved to MAVx parameters, see https://github.com/ArduPilot/ardupilot_wiki/pull/6923
- Winch,Generator,and Beacons are no longer included in std firmware builds for autopilots <2048MB

[site wiki="plane"]
- QwikTune QuadPlane VTOL tuning, see https://github.com/ArduPilot/ardupilot_wiki/pull/6439
- DO_RETURN_PATH_START mission item, see https://github.com/ArduPilot/ardupilot_wiki/pull/6460
- Add some new TX tuning sets, see https://github.com/ArduPilot/ardupilot_wiki/pull/6553
- Add FWD_BAT_THR_CUT parameter, see https://github.com/ArduPilot/ardupilot_wiki/pull/6624
- Add AUTOLAND as battery failsafe and/or fence action, see https://github.com/ArduPilot/ardupilot_wiki/pull/6685
- Add VTOL-FW throttle smoothing parameter, see https://github.com/ArduPilot/ardupilot_wiki/pull/6902
[/site]
[site wiki="copter"]
- Fast Attitude loop option, see https://github.com/ArduPilot/ardupilot_wiki/pull/6470
- Selecting Throttle based notch results in a config error at boot, see https://github.com/ArduPilot/ardupilot_wiki/pull/6551
- Add option to require valid location before arming, see https://github.com/ArduPilot/ardupilot_wiki/pull/6600
- Changed AUTO_TRIM function, see https://github.com/ArduPilot/ardupilot_wiki/pull/6622
- Add ability to tune LOIT_SPEED from transmitter, see see https://github.com/ArduPilot/ardupilot_wiki/pull/6640
- Add EKF FS reporting only option, see https://github.com/ArduPilot/ardupilot_wiki/pull/6937
- Baro thrust compensation, see https://github.com/ArduPilot/ardupilot_wiki/pull/6687
- Correct AUTO NAV_LAND behavior, see https://github.com/ArduPilot/ardupilot_wiki/pull/6945
[/site]
[site wiki="rover"]
- Add option to require valid location before arming, see https://github.com/ArduPilot/ardupilot_wiki/pull/6600
- Add autoarmig option, see https://github.com/ArduPilot/ardupilot_wiki/pull/6878
- Add LOITER/HOLD failsafe action, see https://github.com/ArduPilot/ardupilot_wiki/pull/6911
- Add MTR_REV_DELAY, see https://github.com/ArduPilot/ardupilot_wiki/pull/6935
[/site]
