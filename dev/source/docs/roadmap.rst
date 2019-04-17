.. _roadmap:
    
=====================
RoadMap for 2019/2020
=====================

   .. image:: ../images/roadmap-topimage.jpg
       :width: 40%

This roadmap shows the direction of the ArduPilot team in 2019 and beyond.  The purpose of this roadmap
is not to guarantee exactly when features will be added but instead to help the team, `Partners <http://ardupilot.org/about/Partners>`__
and independent developers to spot areas for cooperation.  There will also undoubtedly be developments that are not on this list.

The main point of contact for each area is provided so that those looking to join in or sponsor the development
know who to contact (see :ref:`Contact Us <common-contact-us>` for a list of ways to contact the devs).

Hardware & OS support (Tridge / Sid)
-------------------------------------------

- software support for new CAN hardware

Scripting (MichaelDB / Tridge)
------------------------------

- add Lua scripting to ArduPilot (`video <https://www.youtube.com/watch?v=ZUNOZMxOwsI>`_ from 2019 un-conference)
- add new plane flight modes and mission items using Lua scripting

EKF (PaulR / Randy)
-------------------

- Robust yaw estimator for Copters
- SLAM integration for position estimation

Plane (Tridge / Mark Whitehorn / MichaelDB)
-------------------------------------------

- VTOL control improvements with a focus on better tailsitter support
- support for dual-airspeed with estimation based failover
- architecture update to align with Rover/Copter
- DeepStall accuracy improvements (`DeepStall video <https://youtu.be/XuSl9Io93aQ?t=1820>`__)

Copter/Rover Non-GPS Navigation, Avoidance and Path Planning (Randy / Patrick Poirier)
--------------------------------------------------------------------------------------

- Add support for Intel RealSense T265, D435 (ROS & non-ROS)
- Back away from objects
- Proximity data (i.e. obstacles) recorded in Earth coordinates
- On-board Path Planning around obstacles
- Improve Off-Board SLAM + Object Avoidance (`dev wiki link <http://ardupilot.org/dev/docs/ros-cartographer-slam.html>`__)

Copter (Randy / Leonard / Tridge)
---------------------------------

- ESC feedback handling
- Motor Mixers

  - sub unity limits on Roll, Pitch and Yaw
  - custom motor mixers

- Flight Mode Improvements

  - System identification mode
  - 4kHz+ loop rate PIDs
  - Stand-By mode

- Attitude controller

  - separate FF and PID input for correct scaling
  - rate loop updates
  - enable rate loop saturation from external sources
  - quaternion error limits based on output saturation
  - quaternion error limits defined by user
  - SI unit input

- Position Controller

  - update Z to XY feed forward architecture
  - include position error limits based on velocity saturation
  - include velocity error limits based on accel saturation
  - handle EKF reset correctly

- Waypoint Navigation

  - Loiter: add support for slippery airframes
  - Auto: navigation rewrite for new pos control
  - Auto: add support for advanced corner support
  - Auto: add support for S-Curves (jerk limited navigation)

- Autotune

  - update for large aircraft
  - fix over angle P issue
  - enable frame specific or controller specific tuning tests
  - helicopter autotune
  - add AltHold tuning
  - chirp, stepped CW, or BPSK excitations

Documentation (Randy / PierreK)
-------------------------------

- Complete MAVLink interface section of developer wiki

Other (names listed below)
--------------------------

- Two stage failsafe for Plane, Copter, Rover, Sub (Michael DB)
- FrSky Sensors to act as battery monitor, airspeed sensor, etc (Chris)
- Black Magic Cinema Camera Pocket 4K control via bluetooth BLE interface for full camera control (Chris)
- Improved detection of a failed airspeed sensor (using synthetic airspeed, wind estimation and through a KF) (Chris)
- 3G/LTE telemetry (Randy)
- APSync to support connecting to external Wifi access point (Randy/Peter)

Organisational (Randy / Tridge)
-------------------------------

- find Wiki maintainer
- increase develop team capacity by reducing barriers to entry through documentation and tools
- assist Ready-To-Fly manufacturers get their products to market

=======================
Items completed in 2018
=======================

Below is a list of Roadmap items completed in 2018.  There were many more projects completed as well!

- ChibiOS support all existing flight boards and features (`ChibiOS video <https://www.youtube.com/watch?v=y2KCB0a3xMg>`_) -- **COMPLETE!**
- port ArduPilot to a wide range of F4 and F7 based flight boards, including boards with integrated OSD and boards in small RTF racing copters -- **COMPLETE!**
- follow mode (`follow mode video <https://www.youtube.com/watch?v=uiJURjgP460>`_) -- **COMPLETE!**
- add spool state handling -- **COMPLETE!**
- support balance bots -- **COMPLETE!**
- pivot turn improvements -- **COMPLETE!**
- active loiter / boat thruster -- **COMPLETE!**
- bring heli mixers up to date -- **COMPLETE!**
- code coverage analyser -- **COMPLETE!**
- find Antenna Tracker maintainer -- **COMPLETE!**
- find Trad Heli maintainer -- **COMPLETE!**