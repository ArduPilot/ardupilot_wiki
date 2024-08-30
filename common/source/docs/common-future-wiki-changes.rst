.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.6 release and is currently in master ("latest")

[copywiki destination="plane,copter,rover,blimp"]

New Board Support
=================

- MicoAir743, see https://github.com/ArduPilot/ardupilot_wiki/pull/5921
- MicoAir NxtPx4v2, see https://github.com/ArduPilot/ardupilot_wiki/pull/5928
- CSKY405 see https://github.com/ArduPilot/ardupilot_wiki/pull/6014
- iFlight Blitz H743 Pro see https://github.com/ArduPilot/ardupilot_wiki/pull/6007
- Flywoo F405 HD 1-2S see https://github.com/ArduPilot/ardupilot_wiki/pull/6025
- CBUnmanned Stamp H743, see https://github.com/ArduPilot/ardupilot_wiki/pull/6037
- JHEMCU-H743HD, see https://github.com/ArduPilot/ardupilot_wiki/pull/6066
- LongbowF405WING, see https://github.com/ArduPilot/ardupilot_wiki/pull/6106
- Holybro KakuteF4-Wing , see https://github.com/ArduPilot/ardupilot_wiki/pull/6176
- GEPRC Taker F745, see https://github.com/ArduPilot/ardupilot_wiki/pull/6184
- Flywoo H743 Pro, see https://github.com/ArduPilot/ardupilot_wiki/pull/6186

New Peripheral Support
======================

- Ainstein LR-D1 Long Range Radar Altimeter see https://github.com/ArduPilot/ardupilot_wiki/pull/5930
- NanoRadar MR72 Object Avoidance Radar see https://github.com/ArduPilot/ardupilot_wiki/pull/5938
- DroneCAN temperature sensors, see https://github.com/ArduPilot/ardupilot_wiki/pull/6079

New Features
============

- Crash dump pre-arm, see https://github.com/ArduPilot/ardupilot_wiki/pull/5920
- Fix MambaH743 serial defaults, see https://github.com/ArduPilot/ardupilot_wiki/pull/6021
- Add AUX switch to disable RF in mount, see https://github.com/ArduPilot/ardupilot_wiki/pull/5974
- Add mask to select which ESCs are used in a given ESC Battery Monitor, see https://github.com/ArduPilot/ardupilot_wiki/pull/6023
- Add EKF3 option to help mediate operation in GPS jamming environments, see https://github.com/ArduPilot/ardupilot_wiki/pull/6069
- Add hires DroneCAN magnetometer array info, see https://github.com/ArduPilot/ardupilot_wiki/pull/6071
- New Notch Filter Option, see https://github.com/ArduPilot/ardupilot_wiki/pull/6213
- Add Mount2 retract aux switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/6090
- Add EKF3 option to align ExternalNAV with OpticalFlow, see https://github.com/ArduPilot/ardupilot_wiki/pull/6124
- iBUS Telemetry, see https://github.com/ArduPilot/ardupilot_wiki/pull/6154
- More RPM sensors allowed, see https://github.com/ArduPilot/ardupilot_wiki/pull/6182
- Per motor throttle based notch filters, see https://github.com/ArduPilot/ardupilot_wiki/pull/6214
- New Temp sensor logging option, see https://github.com/ArduPilot/ardupilot_wiki/pull/6197
- Single board LED notify capability, see https://github.com/ArduPilot/ardupilot_wiki/pull/6221
- Option to set reverse cranking direction for ICE starter, see https://github.com/ArduPilot/ardupilot_wiki/pull/6230

[site wiki="plane"]
- New parachute option, see https://github.com/ArduPilot/ardupilot_wiki/pull/5925
- Option to report airspeed sensor offset calibration to gcs during boot, see https://github.com/ArduPilot/ardupilot_wiki/pull/5913
- Switch to QLAND mode if  a failsafe occurs during VTOL takeoffs, see https://github.com/ArduPilot/ardupilot_wiki/pull/5941
- Add ability to change the ON output voltage polarity for a Relay, see https://github.com/ArduPilot/ardupilot_wiki/pull/5950
- Add Q_BCK_PIT_LIM parameter, see https://github.com/ArduPilot/ardupilot_wiki/pull/5962
- Add AIRSPEED_STALL parameter, see https://github.com/ArduPilot/ardupilot_wiki/pull/6147
- Add AUX switch to test completed Autotune gains in any flight mode, see https://github.com/ArduPilot/ardupilot_wiki/pull/6194
[/site]
[site wiki="copter"]
- Mission pausing via RC switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/5919
- Copter RTL Alt minimum reduced to 30cm, see https://github.com/ArduPilot/ardupilot_wiki/pull/5915
- Option to require valid position before arming, in all modes. See https://github.com/ArduPilot/ardupilot_wiki/pull/6087
- Add AUX switch to test completed Autotune gains in any flight mode, see https://github.com/ArduPilot/ardupilot_wiki/pull/6194
[/site]
[site wiki="rover"]
- Mission pausing via RC switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/5919
- Renamed Torqueedo Motor params and allowed for two motors, see - Mission pausing via RC switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/5919
- Add Omni3 Mecanum frame, see https://github.com/ArduPilot/ardupilot_wiki/pull/6130
[/site]
