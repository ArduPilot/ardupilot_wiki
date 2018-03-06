.. _roadmap:
    
=====================
RoadMap for 2018/2019
=====================

   .. image:: ../images/roadmap-topimage.jpg
       :width: 40%

This roadmap shows the direction of the ArduPilot team in 2018 and beyond.  The purpose of this roadmap
is not to guarantee exactly when features will be added but instead to help the team, `Partners <http://ardupilot.org/about/Partners>`__
and independent developers to spot areas for cooperation.  There will also undoubtedly be developments that are not on this list.

The main point of contact for each area is provided so that those looking to join in or sponsor the development
know who to contact (see :ref:`Contact Us <common-contact-us>` for a list of ways to contact the devs).

Hardware & OS support (Tridge / Sid / TomP)
------------------------------------------

- ChibiOS support all existing flight boards and features (`video <https://www.youtube.com/watch?v=y2KCB0a3xMg>`_)
- port ArduPilot to a wide range of F4 and F7 based flight boards, including boards with integrated OSD and boards in small RTF racing copters
- software support for new CAN hardware

Scripting (MichaelDB / Tridge)
------------------------------

- add Lua scripting to ArduPilot (`video <https://www.youtube.com/watch?v=nc8IwU2G1rY>`_)
- add new plane flight modes and mission items using Lua scripting

EKF (PaulR / Randy)
-------------------

- SLAM integration for position estimation
- Multicopter wind estimation
- Multicopter improved baro disturbance compensation
- RW robust yaw estimator

Plane (Tridge / Leonard / Peter / TomP)
---------------------------------------

- VTOL control improvements with a focus on better tailsitter support
- support for dual-airspeed with estimation based failover
- architecture update to align with Rover/Copter
- DeepStall accuracy improvements (`video <https://youtu.be/XuSl9Io93aQ?t=1820>`_)

Copter (Randy / Leonard / Tridge)
---------------------------------

- Object Avoidance

  - support backing away from objects
  - support SLAM for object avoidance

- Motor Mixers

  - bring heli mixers up to date
  - ESC feedback handling
  - sub unity limits on Roll, Pitch and Yaw
  - custom motor mixers

- Flight Mode Improvements

  - 4kHz+ loop rate PIDs
  - follow mode (`video <https://www.youtube.com/watch?v=uiJURjgP460>`_)
  - add spool state handling
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

Rover (Randy / Grant)
---------------------

- support balance bots
- pivot turn improvements
- active loiter / boat thruster
- backaway from objects

APSync (Randy / PeterB)
-----------------------

- Basler camera support
- allow connecting to external wifi access point
- 3G/LTE telemetry

Documentation (Randy / PierreK)
-------------------------------

- improve developer wiki
- more ROS and gazebo tutorials

Dev Tools (Buzz)
----------------

- code coverage analyser

Other (Randy / Peter / Tridge / MichaelDB)
------------------------------

- AP_Telemetry library to include json, mavlink, frsky protocols
- two stage failsafe for Plane, Copter, Rover, Sub

Organisational (Randy / Tridge)
-------------------------------

- find Wiki maintainer
- find Antenna Tracker maintainer
- find Trad Heli maintainer
- increase develop team capacity by reducing barriers to entry through documentation and tools
- improve developer funding by clarifying roadmap and help companies find developer support
- assist Ready-To-Fly manufacturers get their products to market
