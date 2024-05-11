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

New Peripheral Support
======================

- Ainstein LR-D1 Long Range Radar Altimeter see https://github.com/ArduPilot/ardupilot_wiki/pull/5930
- NanoRadar MR72 Object Avoidance Radar see https://github.com/ArduPilot/ardupilot_wiki/pull/5938

New Features
============


- Full parsing of RTCM stream options for special cases, see https://github.com/ArduPilot/ardupilot_wiki/pull/5924
- Crash dump pre-arm, see https://github.com/ArduPilot/ardupilot_wiki/pull/5920
- Correct the motor GPIO and grouping for the AocodaH743Dual autopilot, see https://github.com/ArduPilot/ardupilot_wiki/pull/5896
- Add ability to store fence list on SD card, see https://github.com/ArduPilot/ardupilot_wiki/pull/5967
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
[/site]
[site wiki="rover"]
- Mission pausing via RC switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/5919
- Renamed Torqueedo Motor params and allowed for two motors, see - Mission pausing via RC switch, see https://github.com/ArduPilot/ardupilot_wiki/pull/5919
[/site]
