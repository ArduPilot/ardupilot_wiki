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

Hardware & OS support (Tridge)
------------------------------

- CAN ecosystem ramp-up
- Add FDCAN support
- Sensor re-ordering
- Increase max number of sensors
- Log synthetic airpseed
- Bi-directional D-shot support

Scripting (MichaelDB / Tridge)
------------------------------

- add Lua scripting to ArduPilot (`video <https://www.youtube.com/watch?v=ZUNOZMxOwsI>`_ from 2019 un-conference)
- add new plane flight modes and mission items using Lua scripting

EKF (PaulR / Randy)
-------------------

- Robust yaw estimator for Copters
- SLAM integration for position estimation

Plane (Tridge)
--------------

- Takeoff mode
- Automatic Taxiing
- Increase QuadPlane landing approach options
- Plane architectural improvements (aka "the onion")

Non-GPS Navigation, Avoidance and Path Planning (Randy / Patrick Poirier)
-------------------------------------------------------------------------

- Add support for Intel RealSense T265, D435 (ROS & non-ROS)
- Copter & Rover back away from objects
- Proximity data (i.e. obstacles) recorded in Earth coordinates
- On-board Path Planning around obstacles
- Improve Off-Board SLAM + Object Avoidance (`dev wiki link <http://ardupilot.org/dev/docs/ros-cartographer-slam.html>`__)

Copter (Randy / Leonard / Tridge)
---------------------------------

- ESC feedback handling
- Flight Mode Improvements

  - System identification mode
  - 4kHz+ loop rate PIDs
  - Stand-By mode

- Attitude controller

  - Separate FF and PID input for correct scaling
  - Rate loop updates
  - Enable rate loop saturation from external sources
  - SI unit input

- Position Controller

  - Update Z to XY feed forward architecture
  - Include position error limits based on velocity saturation
  - Include velocity error limits based on accel saturation
  - Handle EKF reset for position correctly

- AutoTune

  - Add tuning type to PID object

Rover & Boat (Randy / Leonard / Peter)
--------------------------------------

- S-Curve navigation (video from 2019 un-conference)
- Underwater mapping with scanning sonar
- Precision Docking (using marker on shore)
- 3G/LTE telemetry (Randy)
- APSync to support connecting to external Wifi access point (Randy/Peter)

Documentation (Randy / PierreK)
-------------------------------

- Complete MAVLink interface section of developer wiki

ChrisB
------

- FrSky Sensors to act as battery monitor, airspeed sensor, etc (ChrisB)
- Black Magic Cinema Camera Pocket 4K control via bluetooth BLE interface for full camera control (ChrisB)
- Improved detection of a failed airspeed sensor (using synthetic airspeed, wind estimation and through a KF) (ChrisB)

Organisational (Randy / Tridge)
-------------------------------

- Find Wiki maintainer
- Assist Ready-To-Fly manufacturers get their products to market

=======================
Items completed in 2018
=======================

Below is a list of Roadmap items completed in 2018.  There were many more projects completed as well!

- ChibiOS support all existing flight boards and features (`ChibiOS video <https://www.youtube.com/watch?v=y2KCB0a3xMg>`_) -- **COMPLETE!**
- port ArduPilot to a wide range of F4 and F7 based flight boards, including boards with integrated OSD and boards in small RTF racing copters -- **COMPLETE!**
- follow mode (`follow mode video <https://www.youtube.com/watch?v=uiJURjgP460>`_) -- **COMPLETE!**
- Two stage battery failsafe for Plane, Copter, Rover, Sub (Michael DB) -- **COMPLETE!**
- add spool state handling -- **COMPLETE!**
- support balance bots -- **COMPLETE!**
- pivot turn improvements -- **COMPLETE!**
- active loiter / boat thruster -- **COMPLETE!**
- bring heli mixers up to date -- **COMPLETE!**
- handle EKF reset for attitude correctly -- **COMPLETE!**
- AutoTune update for large aircraft
- AutoTune fix for over angle P issue
- code coverage analyser -- **COMPLETE!**
- improve developer wiki -- **COMPLETE!**
- find Antenna Tracker maintainer -- **COMPLETE!**
- find Trad Heli maintainer -- **COMPLETE!**