.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2026
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2026 <https://summerofcode.withgoogle.com/>`__. These are only suggestions so if you have your own ideas then please discuss them on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__

- Fleet Management Webtool
- SITL Model Generation from Flight Data
- Multi-Drone Mesh Networking (MAVLink-aware)
- ArduHumanoid (ArduPilot controlling a simple humanoid)
- AI-Assisted Log Diagnosis & Root-Cause Detection
- Real-Time Companion-Computer Health Monitoring & Failsafe

See lower down on this page for more details on each project

Timeline
========

The timeline for `GSoC 2026 is here <https://developers.google.com/open-source/gsoc/timeline>`__

How to improve your chances of being accepted
=============================================

When making the difficult decision about which students to accept, we look for:

- Clear and detailed application explaining how you think the project could be done
- Relevant prior experience
- Experience contributing to ArduPilot or other open source projects
- Understanding of Git and/or GitHub

Fleet Management WebTool
------------------------

- Skills required: Javascript, Python
- Mentors: Ryan Friedman, Randy Mackay
- Expected Size: 175h
- Level of Difficulty: Medium
- Expected Outcome: Webtool to ease the management of a fleet of ArduPilot vehicles

The goal of this project is create a fleet management web tool that helps companies and individuals manage the data collected by multiple ArduPilot vehicles

- Should extend the capabilities of the existing `LogFinder Webtool <https://firmware.ardupilot.org/Tools/WebTools/LogFinder/>`__
- Accept onboard logs, tlogs, photos and videos uploaded by the GCS or from the vehicle's companion computer (possibly running BlueOS or APSync)
- Allow users to search and download data based on vehicle ID, recording date, location
- Support both table views and map views of the uploaded data

Funding will be provided for hardware and cloud server as required.

SITL Model Generation from Flight Data
--------------------------------------

- Skills required: Python, C++ (ArduPilot/SITL), system identification
- Mentors: Nathaniel Mailhot
- Expected Size: 350h
- Level of Difficulty: Hard
- Expected Outcome: A toolchain that auto-builds or tunes SITL airframe models from real flight logs

The goal of this project is to take ArduPilot logs and estimate the key dynamics/sensor parameters needed for SITL, then output an updated model + params that better match the real vehicle.

Multi-Drone Mesh Networking (MAVLink-aware)
-------------------------------------------

- Skills required: Networking, C/C++, Linux, MAVLink
- Mentors: Nathaniel Mailhot
- Expected Size: 350h
- Level of Difficulty: Hard
- Expected Outcome: A practical mesh networking layer for multi-vehicle comms (telemetry + coordination)

The goal of this project is to enable resilient multi-hop links between multiple ArduPilot vehicles, so telemetry and commands can route through the swarm when direct links drop.

ArduHumanoid (ArduPilot controlling a simple humanoid)
------------------------------------------------------

- Skills required: C++, control, servo systems, simulation (Gazebo/Ignition)
- Mentors: Nathaniel Mailhot
- Expected Size: 175h
- Level of Difficulty: Medium
- Expected Outcome: A minimal humanoid "vehicle type" running on ArduPilot with SITL support

The goal of this project is to prove ArduPilot can command a small humanoid-style jointed frame (think "servo robot"), with a basic control interface and a simple simulated model.

AI-Assisted Log Diagnosis & Root-Cause Detection
------------------------------------------------

- Skills required: Python, ML (classification + retrieval), ArduPilot logs/parameters
- Mentors: Nathaniel Mailhot
- Expected Size: 350h
- Level of Difficulty: Hard
- Expected Outcome: A model/service that flags likely root causes from logs and suggests fixes with confidence

The goal of this project is to automatically diagnose common failures and misconfigurations by learning from labeled log segments, known issue patterns, and parameter states. It should output a probable root cause, suggested fixes, and a confidence score (with links to the relevant evidence in the log).

Real-Time Companion-Computer Health Monitoring & Failsafe
---------------------------------------------------------

- Skills required: C/C++ or Python, MAVLink, Linux companion computers
- Mentors: Jaime Machuca
- Expected Size: 175h
- Level of Difficulty: Medium
- Expected Outcome: A standard MAVLink-based health reporting + failsafe mechanism for companion computers

The goal of this project is to define and implement a consistent "companion health" report (CPU/GPU load, heartbeat, critical services, watchdog) and connect it to configurable failsafes so ArduPilot can respond predictably when the companion degrades or dies.

Projects Completed in past years
--------------------------------

In 2025, students completed the following projects:

- `Non-GPS Position Estimation Using 3D Camera and Pre-Generated Map <https://discuss.ardupilot.org/t/gsoc-25-non-gps-position-estimation-using-3d-camera-and-pre-generated-map-final/138513>`__
- `AI Chat WebTool for use with MP and/or QGC <https://discuss.ardupilot.org/t/gsoc-2025-ai-chat-webtool-final-project-summary/138287>`__
- `AI Chat Integration with all Web Tools <https://discuss.ardupilot.org/t/gsoc-2025-ai-chat-webtool-final-project-summary/138287>`__
- `Gazebo Plug-in Model of a Motor <https://discuss.ardupilot.org/t/gsoc-2025-wrapping-up-gazebo-plug-in-model-of-a-motor/138509>`__
- `SITL AI Reinforcement Learning Concept Script <https://discuss.ardupilot.org/t/gsoc-2025-wrapping-up-sitl-ai-reinforcement-learning-concept-script/138504>`__

In 2024, students completed the following projects:

- `Visual Follow-Me <https://discuss.ardupilot.org/t/gsoc-2024-wrapping-up-visual-follow-me/123232>`__
- `High Altitude Non-GPS Navigation <https://discuss.ardupilot.org/t/gsoc-2024-wrapping-up-high-altitude-non-gps-navigation/122905>`__
- `MAVProxy AI Chat Enhancements <https://discuss.ardupilot.org/t/gsoc-2024-wrapping-up-mavproxy-ai-chat-enhancements/122793>`__
- `All-in-One System Identification Toolkit <https://discuss.ardupilot.org/t/gsoc24-all-in-one-system-identification-toolkit-for-ardupilot-update/121116>`__

In 2023, students completed the following projects:

- `Improvements for ROS2 Support for Ardupilot <https://discuss.ardupilot.org/t/gsoc-23-wrapping-up-improvements-to-the-native-dds-support-in-ardupilot/105643>`__
- `Camera and Gimbal enhancements <https://discuss.ardupilot.org/t/gsoc-2023-wrapping-up-camera-and-gimbal-enhancements/105600>`__
- `GPS-Denied Autonomous Exploration with ROS 2 <https://discuss.ardupilot.org/t/gsoc-2023-gps-denied-autonomous-exploration-with-ros-2/101121>`__
- `MultiCopter Swarm Avoidance <https://discuss.ardupilot.org/t/gsoc-2023-multicopter-swarm-avoidance/102108>`__

In 2022, students worked on these projects:

- `Rover autonomous docking <https://discuss.ardupilot.org/t/gsoc-2022-rover-autodocking-conclusion/90626>`__
- `ROS integration for Non-GPS navigation and off-board path-planning <https://discuss.ardupilot.org/t/gsoc-2022-update-ros-integration-for-non-gps-navigation-and-off-board-path-planning/86948>`__
- `Boat object avoidance with Luxonis AI camera <https://discuss.ardupilot.org/t/gsoc-2022-boat-object-avoidance-with-luxonis-ai-camera/91257>`__

In 2019, students successfully completed these projects:

- AirSim Simulator Support for ArduPilot SITL
- Development of Autonomous Autorotations for Traditional Helicopters
- Further Development of Rover Sailboat Support
- Integration of ArduPilot and VIO tracking camera for GPS-less localization and navigation
- MAVProxy GUI and module development

In 2018, students successfully completed these projects:

- `BalanceBot <https://ardupilot.org/rover/docs/balance_bot-home.html>`__
- RedTail integration with ArduPilot
- Live video improvements for APSync

In 2017, 3 students successfully completed these projects:

- Smart Return-To-Launch which involves storing the vehicle's current location and maintaining the shortest possible safe path back home
- Rework ArduRover architecture to allow more configurations and rover type (`see details here <https://github.com/khancyr/GSOC-2017>`__)
- Add "sensor head" operation of ArduPilot, split between two CPUs

 You can find their proposals and works on the `Google GSoC 2017 archive page <https://summerofcode.withgoogle.com/archive/2017/organizations/5801067908431872>`__
