.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.6 release and is currently in master ("latest")

[copywiki destination="plane,copter,rover,blimp"]

New Board Support
=================

- MicoAir405v2/Mini, see https://github.com/ArduPilot/ardupilot_wiki/pull/5926
- MicoAir743, see https://github.com/ArduPilot/ardupilot_wiki/pull/5921
- MicoAir NxtPx4v2, see https://github.com/ArduPilot/ardupilot_wiki/pull/5928
- CSKY405 see https://github.com/ArduPilot/ardupilot_wiki/pull/6014
- iFlight Blitz H743 Pro see https://github.com/ArduPilot/ardupilot_wiki/pull/6007
- Flywoo F405 HD 1-2S see https://github.com/ArduPilot/ardupilot_wiki/pull/6025
- OrqaF405 see: https://github.com/ArduPilot/ardupilot_wiki/pull/6028
- 3DR Control Zero H7 OEM RevG, see https://github.com/ArduPilot/ardupilot_wiki/pull/6033
- CBUnmanned Stamp H743, see https://github.com/ArduPilot/ardupilot_wiki/pull/6037
- PixFlamingo-F767, see https://github.com/ArduPilot/ardupilot_wiki/pull/6048
- JHEMCU-H743HD, see https://github.com/ArduPilot/ardupilot_wiki/pull/6066
- iflight Thunder H7, see https://github.com/ArduPilot/ardupilot_wiki/pull/6105
- LongbowF405WING, see https://github.com/ArduPilot/ardupilot_wiki/pull/6106

New Peripheral Support
======================

- Ainstein LR-D1 Long Range Radar Altimeter see https://github.com/ArduPilot/ardupilot_wiki/pull/5930
- NanoRadar MR72 Object Avoidance Radar see https://github.com/ArduPilot/ardupilot_wiki/pull/5938
- DroneCAN temperature sensors, see https://github.com/ArduPilot/ardupilot_wiki/pull/6079

New Features
============


- Full parsing of RTCM stream options for special cases, see https://github.com/ArduPilot/ardupilot_wiki/pull/5924
- Crash dump pre-arm, see https://github.com/ArduPilot/ardupilot_wiki/pull/5920
- Add ability to store fence list on SD card, see https://github.com/ArduPilot/ardupilot_wiki/pull/5967
- Fix MambaH743 serial defaults, see https://github.com/ArduPilot/ardupilot_wiki/pull/6021
- Add AUX switch to disable RF in mount, see https://github.com/ArduPilot/ardupilot_wiki/pull/5974
- Add mask to select which ESCs are used in a given ESC Battery Monitor, see https://github.com/ArduPilot/ardupilot_wiki/pull/6023
- Add EKF3 option to help mediate operation in GPS jamming environments, see https://github.com/ArduPilot/ardupilot_wiki/pull/6069
- Add hires DroneCAN magnetometer array info, see https://github.com/ArduPilot/ardupilot_wiki/pull/6071
- New Notch Filter Option, see https://github.com/ArduPilot/ardupilot_wiki/pull/6108
- Add Mount2 retract aux switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/6111
- Add EKF3 option to align ExternalNAV with OpticalFlow, see https://github.com/ArduPilot/ardupilot_wiki/pull/6124
[site wiki="plane"]
- New parachute option, see https://github.com/ArduPilot/ardupilot_wiki/pull/5925
- Option to report airspeed sensor offset calibration to gcs during boot, see https://github.com/ArduPilot/ardupilot_wiki/pull/5913
- Switch to QLAND mode if  a failsafe occurs during VTOL takeoffs, see https://github.com/ArduPilot/ardupilot_wiki/pull/5941
- Add ability to change the ON output voltage polarity for a Relay, see https://github.com/ArduPilot/ardupilot_wiki/pull/5950
- Add TECS option for steep descents for low drag vehicles, see https://github.com/ArduPilot/ardupilot_wiki/pull/5956
- Add Q_BCK_PIT_LIM parameter, see https://github.com/ArduPilot/ardupilot_wiki/pull/5962
[/site]
[site wiki="copter"]
- Mission pausing via RC switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/5919
- Copter RTL Alt minimum reduced to 30cm, see https://github.com/ArduPilot/ardupilot_wiki/pull/5915
- Option to require valid position before arming, in all modes. See https://github.com/ArduPilot/ardupilot_wiki/pull/6087
[/site]
[site wiki="rover"]
- Mission pausing via RC switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/5919
- Renamed Torqueedo Motor params and allowed for two motors, see - Mission pausing via RC switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/5919
- Add Omni3 Mecanum frame, see https://github.com/ArduPilot/ardupilot_wiki/pull/6130
[/site]
