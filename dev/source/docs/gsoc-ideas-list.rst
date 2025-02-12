.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2025
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2025 <https://summerofcode.withgoogle.com/>`__. These are only suggestions so if you have your own ideas then please discuss them on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__

- Non-GPS Position Estimation Using 3D Camera and Pre-Generated Map
- AI Chat WebTool for use with MP and/or QGC
- AI Chat Integration with all Web Tools

See lower down on this page for more details on each project

Timeline
========

The timeline for `GSoC 2025 is here <https://developers.google.com/open-source/gsoc/timeline>`__

How to improve your chances of being accepted
=============================================

When making the difficult decision about which students to accept, we look for:

- Clear and detailed application explaining how you think the project could be done
- Relevant prior experience
- Experience contributing to ArduPilot or other open source projects
- Understanding of Git and/or GitHub

Non-GPS Position Estimation Using 3D Camera and Pre-Generated Map
-----------------------------------------------------------------

- Skills required: Python, C++
- Mentor: Randy Mackay
- Expected Size: 175h
- Level of Difficulty: Hard
- Expected Outcome: Copter with low-cost 3D camera estimates its local position by comparing the camera point cloud to a pre-generated 3D map

The goal of this project is to allow a Copter to estimate its local position using a low-cost 3D camera (e.g. Intel D465) by comparing the camera's point cloud to a pre-generated 3D map.  The steps involved include:

- Create a tool to capture a 3D map of the flight area.  The resulting map should be loaded onto the vehicle's companion computer (e.g. RPI5)
- Mount a low-cost 3D camera (e.g. Intel D465) onto an ArduPilot copter (e.g. EDU650 or similar) equipped with a companion computer
- Write localisation software (e.g. python code) to compare the output of the 3D camera to the pre-generated 3D map and send the estimated position to the vehicle's EKF (see :ref:`Non-GPS Position Estimation <mavlink-nongps-position-estimation>`)
- Implement a simulator of the system (e.g. gazebo)
- Document the setup and operation for future developers and users

Funding will be provided for hardware including a copter (e.g. Hexsoon EDU650), companion computer and 3D camera (e.g. Intel D465) if necessary

AI Chat WebTool for use with MP and/or QGC
------------------------------------------

- Skills required: JavaScript, OpenAI, Google Gemini
- Mentor: Randy Mackay
- Expected Size: 175h
- Level of Difficulty: Medium
- Expected Outcome: Web tool capable following a pilot's verbal commands and converting them to MAVLink in order to control an ArduPilot multicopter

This project involves re-implementing the `MAVProxy's AI chat module <https://ardupilot.org/mavproxy/docs/modules/chat.html>`__ (`see blog here <https://discuss.ardupilot.org/t/ardupilot-openais-chatgpt-using-mavproxys-chat-module/111336>`__) to run as a `WebTool <https://firmware.ardupilot.org/Tools/WebTools/>`__

Once complete the WebTool should be capable of:

- Connecting to the vehicle via Mission Planner or QGC
- Responding to verbal or written questions and commands from the pilot
- Arming the vehicle
- Issuing takeoff commands and flying the vehicle a specified distance in any direction
- Changing the vehicle's flight mode

Most of the development can be completed using the :ref:`SITL simulator <sitl-simulator-software-in-the-loop>` and any OpenAI or Google Gemini usage costs will be covered

AI Chat Integration with all WebTools
--------------------------------------

- Skills required: JavaScript, OpenAI, Google Gemini
- Mentor: Randy Mackay
- Expected Size: 175h
- Level of Difficulty: Medium
- Expected Outcome: All WebTools include AI chat to help users understand and use the tool

This project involves adding an OpenAI or Google Gemini chat window into some or all of the `ArduPilot Webtools <https://firmware.ardupilot.org/Tools/WebTools/>`__

Once complete some or all of the WebTools should:

- Include a new chat widget allowing users to ask an AI assistant questions about the tool using text or voice
- Allow the AI assistant to operate the tool based on user input (e.g. push buttons, change zoom of graphs, etc)

The top priority WebTool is the "UAV Log viewer" although simpler tools like the "Hardware Report" could be a good starting point

Most of the development can be completed using the :ref:`SITL simulator <sitl-simulator-software-in-the-loop>` and any OpenAI or Google Gemini usage costs will be covered

Projects Completed in past years
--------------------------------

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
