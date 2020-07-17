.. _roadmap:
    
=====================
RoadMap for 2020/2021
=====================

   .. image:: ../images/roadmap-topimage.jpg
       :width: 40%

This roadmap shows the direction of the ArduPilot team in 2020 and beyond.  The purpose of this roadmap
is not to guarantee exactly when features will be added but instead to help the team, `Partners <https://ardupilot.org/about/Partners>`__
and independent developers to spot areas for cooperation.  There will also undoubtedly be developments that are not on this list.

The main point of contact for each area is provided so that those looking to join in or sponsor the development
know who to contact (see :ref:`Contact Us <common-contact-us>` for a list of ways to contact the devs).


Copter
---------------------------------

- ESC feedback handling (Randy)
- 4kHz+ loop rate PIDs (Tridge, Leonard)

- Attitude controller

  - Enable rate loop saturation from external sources (Leonard)
  - SI unit input (Leonard)

- Position Controller

  - Update Z to XY feed-forward architecture (Leonard)
  - Include position error limits based on velocity saturation (Leonard)
  - Include velocity error limits based on accel saturation (Leonard)
  - Handle EKF reset for the position correctly (Leonard)
  - Baseline Velocity input (Leonard)
  
- S-Curve
  - Concept demonstrator done. Implementation needed. (Leonard)
		
- Guided Mode
  - Input shaped using FF (Leonard)
		
- AutoTune
  - Add tuning type to PID object (Bill Geyer)
  
  
Plane
--------------------------

- Automatic Taxing (Tridge)
- Better RealFlight quadplane model
- Increase QuadPlane landing approach options
- Plane architectural improvements (aka “the onion” v2)
- Possibly switch to copter PID controllers
- QuadPlane flat/spoiler control (MichaelDB)
  

Rover & Boat
------------

- S-Curve navigation (`video from 2019 un-conference <https://www.youtube.com/watch?v=LHq5o9zgNWk>`__) (Leonard/Randy)
- Precision Docking (using marker onshore) (Randy)
- High speed sailing inprovements (Peter Hall)
- Roll and pitch control for boats (Peter Hall)
- AIS (i.e., ADSB for boats) (Peter Hall)


Trad Heli
---------

- Automated handling of engine throttle for autonomous operations (Bill Geyer, GSoC)
- Automatic engine failure identification and autorotation entry (Matt Kear)
- Autonomous Autorotation (Matt Kear)
- Handling of manual autorotation (Matt Kear)
- Heli Autotune (Bill Geyer)
- Improve ground handling and ground/air transitions (Bill Geyer/Matt Kear)
- Improve high-speed autonomous maneuvering (Bill Geyer, GSoC)
- Improved shaping functions in the attitude controller (Bill Geyer, GSoC)
- Tie motor spool states to measured rotor speed when a measurement is available (Bill Geyer)


EKF
---

- Bring the missing features from EKF2 into EKF3 to remove EKF2 (Paul)
- EKF3 External Nav support
- Fixing replay (PeterB)
- Set EKF3 as the default
- Robust yaw estimator for Copters


Hardware & OS support
---------------------

- Add FDCAN support (Sid)
- Bi-directional D-shot support (Tridge)
- CAN GPS moving baseline yaw (Tridge)
- CAN IMUs (Phil K for HW)
- CAN SITL Support (Sid)
- CAN ecosystem ramp-up (Sid, Tridge)
- Easier AirSim with the complex environment (Ryan, Rajat)
- Extend maximum mission size	(Tridge)
- Fast mission upload (Tridge)
- Filter params for fixed-wing PIDs (Tridge)
- Fix performance regressions (Tridge)
- High-performance IMUs (ADIS) (Sid)
- High-speed USB support - Nora (Tridge)
- Improve USB performance	(Sid, Michael O.)
- Increase max number of sensors (Tridge)
- Log synthetic airspeed (Tridge)
- Mission VFS download (Tridge)
- Sensor (GPS, Baro, RFND, etc.) re-ordering (Sid, Tridge)
- Ethernet (IP/TCP/UDP) support (Tom)


Hardware
--------

- CAN hardware (PhilipR)
- ADSB inclusion (PhilipR)
- Professional Tightly coupled GNSS (PhilipR)
- Reference multicopter frame (PhilipR)
- Intel Open Drone ID setup (JeffW)


Non-GPS Navigation, Avoidance and Path Planning
-----------------------------------------------

- 3D Bendy Ruler & Object Database (Randy, GSoC)
- Add support for Intel RealSense D435 (ROS & non-ROS) (Patrick, GSoC)
- Bendy Ruler try re-implementing bendy ruler as Lua (Tridge)
- Copter & Rover back away from objects (Randy, Peter Hall)
- Improve Off-Board SLAM and Object Avoidance (:ref:`dev wiki link<ros-cartographer-slam>`) (Randy, Jaime, Patrick Poirier)
- Improve reliability of T265 integration (reset handling, failsafe testing) (Randy, Jaime, Patrick Poirier)
- Seamless GPS <-> Non-GPS transitioning (Randy)
- T265 image output to external VO (Tridge)


Scripting
---------

- All mission features available in Lua (MichaelDB)
- Figure flight mode in Plane as a script (Tridge)
- Scripts creation parameter trees (Tridge)


Documentation
-------------

- Complete MAVLink interface section of developer wiki (Randy)
- Dual GPS for Yaw setup (Henry, Tridge, Randy)
- Frsky Telem Update (Alex, Henry)
- Scripting Documentation (MichaelDB)
- Substitute C5 (Bruno)
- Translation support (Bruno)
- Tuning/Fltr Copter to QuadPlane (Henry)


Miscellaneous 
-------------

- FrSky Sensors to act as a battery monitor, airspeed sensor, etc (ChrisB)
- Black Magic Cinema Camera Pocket 4K control via Bluetooth BLE interface for full camera control (ChrisB)
- Improved detection of a failed airspeed sensor (using synthetic airspeed,wind estimation and through a KF) (ChrisB)
- Support adding information about messages and fields to Onboard logs (PeterB)
- Support OpenDroneID (Tom)


Organizational
--------------

- Assist Ready-To-Fly manufacturers get their products to market
- Find new BugMaster and support roles (Tridge, Randy, James, MichaelDB) -- Complete!
- Governance structure (James)


---------------------------------------------

=======================
Items completed in 2019
=======================

Below is a list of Roadmap items completed in 2019.  There were many more projects completed, as well!


- 3G/LTE telemetry for Rover & Boat (Randy)
- APSync to support connecting to an external Wifi access point for Rover & Boat (Randy/Peter)
- Add tuning type to PID object in AutoTune for Copter
- Closed-loop rotor speed governor for gas and turbine engine helicopters for Tradi Heli
- HAL file operation abstractions
- Improve Off-Board SLAM + Object Avoidance (:ref:`dev wiki link<ros-cartographer-slam>`)
- Improve rotor speed control library architecture for Tradi Heli
- L1 navigation and speed/height controller for helicopter high-speed autonomous missions for Tradi Heli
- Proximity data (i.e., obstacles) recorded in Earth coordinates
- Rate loop updates for Copter
- SLAM integration for position estimation
- Separate FF and PID input for correct scaling for Copter
- Stand-By mode for Copter -- **COMPLETE!**
- Virtual flybar option for acro flight mode for Tradi Heli -- **COMPLETE!**
- Add Lua scripting to ArduPilot -- **COMPLETE!**
- Add support for Intel RealSense T265 (ROS & non-ROS) -- **COMPLETE!**
- Copter & Rover back away from objects -- **COMPLETE!**
- ESC feedback handling for Copter -- **COMPLETE!**
- Find Wiki maintainer -- **COMPLETE!**
- On-board Path Planning around obstacles -- **COMPLETE!**
- Plane architectural improvements (aka "the onion") -- **COMPLETE!**
- System identification mode -- **COMPLETE!**
- Takeoff mode -- **COMPLETE!**


---------------------------------------------

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
- AutoTune update for large aircraft -- **COMPLETE!**
- AutoTune fix for over angle P issue -- **COMPLETE!**
- code coverage analyser -- **COMPLETE!**
- improve developer wiki -- **COMPLETE!**
- find Antenna Tracker maintainer -- **COMPLETE!**
- find Trad Heli maintainer -- **COMPLETE!**
