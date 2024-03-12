.. _gsoc-ideas-list:
    
========================================
List of Suggested Projects for GSoC 2024
========================================

This is a list of projects suggested by ArduPilot developers for `GSoC 2024 <https://summerofcode.withgoogle.com/>`__. These are only suggestions so if you have your own ideas then please discuss them on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ or on the `discuss server here <https://discuss.ardupilot.org/c/google-summer-of-code>`__.  We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for GSoC 2024.

- Visual Follow-me using AI
- MAVProxy AI chat enhancements
- WebTools automated log analysis
- Improvements to the Custom Build Server
- High Altitude Non-GPS Position Estimation
- Improve web server for file management and parameter management

See lower down on this page for more details for some of the projects listed above

Timeline
========

The timeline for `GSoC 2024 is here <https://developers.google.com/open-source/gsoc/timeline>`__

How to improve your chances of being accepted
=============================================

When making the difficult decision about which students to accept, we look for:

- Clear and detailed application explaining how you think the project could be done
- Relevant prior experience
- Experience contributing to ArduPilot or other open source projects
- Understanding of Git and/or GitHub

Visual Follow-me using AI
-------------------------

- Skills required: Python, Yolo, C++
- Mentor: Randy Mackay
- Expected Size: 175h or 350h
- Level of Difficulty: Medium
- Expected Outcome: Copter or Rover follows a user while also keeping the camera mount pointed at them

There are two steps to this project

Step #1 is to add Visual Follow-me support for all `ArduPilot camera gimbals <https://ardupilot.org/copter/docs/common-cameras-and-gimbals.html>`__:

- Users will start "image tracking" using the existing `Image Tracking auxiliary function <https://ardupilot.org/copter/docs/common-auxiliary-functions.html>`__ or by sending a `DO_TRIGGER_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_TRIGGER_CONTROL>`__ command from a ground station (no GCSs support sending this yet)
- ArduPilot's `GCS_MAVLink <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink>`__ and `AP_Mount <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount>`__ libraries will be enhanced to forward this command onto the companion computer (e.g. NVidia Nano, RPI5, etc)
- The companion computer will run a new Python program using `Yolov8 <https://docs.ultralytics.com/>`__ (or similar) to recognise objects (e.g. people) and then send `mavlink gimbal commands <https://ardupilot.org/dev/docs/mavlink-gimbal-mount.html>`__ to keep the gimbal pointed at the object
- The Python program could first be written as a MAVProxy module before being migrated to a stand-alone program

Step #2 is to allow the vehicle to follow the object

- Extend the Python program to calculate the object's Location (e.g. lat, lon, altitude) (see `AP_Mount_Backend::get_poi() <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mount/AP_Mount_Backend.cpp#L489>`__ or `AP_Scripting's mount-poi.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/mount-poi.lua>`__ to see how this can be done).  This location should be sent to the vehicle using the `CAMERA_TRACKING_GEO_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_GEO_STATUS>`__ message
- Enhance `Copter's Follow mode <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_follow.cpp>`__ (`over Rover's <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_follow.cpp>`__) to consume and use the `CAMERA_TRACKING_GEO_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_GEO_STATUS>`__ message

An important output of the project is to document the setup for future developers and users.

Some of the development can be completed using the :ref:`SITL simulator <sitl-simulator-software-in-the-loop>` but funding will also be provided for an RC car frame, autopilot and companion computer (e.g. NVidia Nano, RPI5, etc) if required.

MAVProxy AI Chat Enhancements
-----------------------------

- Skills required: Python, AI
- Mentor: Randy Mackay
- Expected Size: 175h
- Level of Difficulty: Medium
- Expected Outcome: The MAVProxy chat module can better control Copters, Planes and Rovers including creating new missions

This project involves numerous small enhancements and bug fixes to `MAVProxy's AI chat module <https://ardupilot.org/mavproxy/docs/modules/chat.html>`__ (`see blog here <https://discuss.ardupilot.org/t/ardupilot-openais-chatgpt-using-mavproxys-chat-module/111336>`__) including:

- Arming and control of Planes
- Arming and control of Rovers and Boats
- Camera gimbal controls (point gimbal, take pictures, record video)
- Create simple missions
- Improve reliability of existing functions

A stretch goal is to replace the use of OpenAI with a locally installed, open source LLM.

Most of the development can be completed using the :ref:`SITL simulator <sitl-simulator-software-in-the-loop>` and any OpenAI usage costs will be covered. Funding for an autopilot and frame will be provided if required.

WebTools automated log analysis
-------------------------------

- Skills required: HTML, JavaScript
- Mentor: Peter Hall
- Expected Size: 175h to 350h
- Level of Difficulty: Medium
- Expected Outcome: A new WebTool with the ability to automatically highlight and diagnose common issues from flight logs.

ArduPilot has several `WebTools <https://firmware.ardupilot.org/Tools/WebTools/>`__ for log review, these run in any browser with all computation on the client side. So far we have tools for setup tasks (FilterReivew, PIDReview, MAGFit) and a tool that focuses on hardware issues (HarwareReport).
The next evolution is a tool to look for in-flight issues. There are two existing automated log review tools. `MissionPlanner <https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html#automatic-analysis-of-logs>`__ includes a basic analysis tool.
`Dronekit log analyzer <https://github.com/dronekit/dronekit-la/tree/master>`__ has a larger number of checks (see: `analyzers <https://github.com/dronekit/dronekit-la/blob/master/docs/reference/analyzers.rst>`__) and would be the initial benchmark for this project.
Once a framework for the tool is up and running checks for as many issues as possible can be added, the tool should focus on making the checks easy to add and update rather than a fancy-looking interface.

Improvements to the Custom Build Server
---------------------------------------

- Skills required: Python, Flask, Javascript, Docker
- Mentor: Shiv Tyagi, Peter Barker
- Expected Size: 175h to 350h
- Level of Difficulty: Medium
- Expected Outcome: Enhanced custom build server with automated build size estimation and operational/security improvements

The custom build server stands as a valuable utility, empowering users to tailor their software builds by selectively enabling or disabling features deemed less crucial for their specific requirements, thereby conserving flash memory on flight controllers. 
Originally developed as part of the Google Summer of Code program in 2020, this application has demonstrated its effectiveness in enhancing flexibility and resource management. However, there remains room for improvement to further elevate its usability and broaden its appeal, ensuring it meets the diverse needs of an expanding user base.

Some of the problems we observe in the custom build server are as follows:

- Build failures occur due to the excessive selection of features that cannot fit into the memory of the intended board.
- Application can become unresponsive due to unexpected exceptions during the build step or any other step in the process.
- The manual addition of branches is required every time a release is made at github.com/ardupilot/ardupilot.
- The testing environment is inadequate. There is no easy way for a developer to test feature additions and deletions.

Some possible improvements to address these issues can be:

- Come up with a mathematical algorithm to estimate the approximate size of each feature on a branch. This can be achieved by leveraging the `test_build_options.py <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/test_build_options.py>`__ script to measure the size of the binary when each feature is sequentially enabled and disabled. An algorithm should be developed to estimate the size of individual features while accounting for their dependencies. Remember, when a feature is enabled, it also activates any dependent features.
- Implement containerisation for running the application. By containerising the application, it can also be divided into multiple services, such as the main application and micro-services responsible for tasks such as reporting the status of server builds. Containerization not only enhances application security but also facilitates scalability and ease of deployment.
- Develop a service responsible for monitoring the GitHub repository (github.com/ardupilot/ardupilot) or firmware.ardupilot.org for new releases. This service can automatically add relevant entries to the main application, enabling it to serve customised builds for newly released branches. This automation streamlines the process of integrating new releases into the build server.
- Enhance the build server to support builds from any repository, not just the upstream repository. While implementing this feature, careful consideration must be given to potential complexities and challenges associated with supporting builds from multiple repositories. 

Some github issues having feature requests for Custom Build Server:

- https://github.com/ArduPilot/ardupilot/issues/21345
- https://github.com/ArduPilot/CustomBuild/issues/2

Remember, these are just suggestions. The contributors can use the application at `custom.ardupilot.org <https://custom.ardupilot.org>`__, read the source code `here <https://www.github.com/ardupilot/CustomBuild>`__ and suggest any other improvement which they would like to see in the app.

High Altitude Non-GPS Position Estimation
-----------------------------------------

- Skills required: Python, C++
- Mentor: Randy Mackay
- Expected Size: 175h or 350h
- Level of Difficulty: Hard
- Expected Outcome: Copter can maintain position at high altitudes without a GPS

ArduPilot copter supports numerous methods of `Non-GPS navigation <https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html>`__ but most are designed for indoor use
and do not work at altitudes above about 40m meaning that in practice they are not useful to protect against loss of GPS.

This project aims to allow Copters to maintain an adequate position estimate at altitudes of at least 100m using downward facing camera (in a gimbal).

- A set of base images will be captured with known Locations (latitude, longitude, altitude, altitude above terrain).  These might be taken by the vehicle itself while GPS is operating normally or they could be satellite images of the area
- A companion computer (e.g. RPI or NVidia Nano) will capture images from a downward facing camera and compare them to the base images to calculate a new latitude, longitude and altitude.  Lag is important as the EKF may struggle if the estimates are over 0.25 seconds old.
- This estimated Location should then be sent to the autopilot using one of these supported mavlink messages

    - `VISION_POSITION_ESTIMATE <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml#L4978>`__ (recommended)
    - `VISION_SPEED_ESTIMATE <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml#L4991>`__
    - `ODOMETRY <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml#L6262>`__
    - `MAV_CMD_EXTERNAL_POSITION_ESTIMATE <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml#L333>`__ (requires the vehicle be dead-reckoning using `wind speed estimates <https://ardupilot.org/copter/docs/airspeed-estimation.html>`__ but is also less sensitive to lag)
    - `GLOBAL_VISION_POSITION_ESTIMATE <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml#L4965>`__
    - `VICON_POSITION_ESTIMATE <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml#L5001>`__
    - `ATT_POS_MOCAP <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml#L5392>`__

- Some EKF tuning will likely be required to allow the EKF to expect very noisy position estimates
- If time permits a `GPS/Non-GPS transition <https://ardupilot.org/copter/docs/common-non-gps-to-gps.html>`__ Lua script (`like this one <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/ahrs-source-gps-optflow.lua>`__) could be developed

An important output of the project is to document the setup for future developers and users.

Most of the development can probably be done using :ref:`SITL <sitl-simulator-software-in-the-loop>`, `Gazebo <https://ardupilot.org/dev/docs/sitl-with-gazebo.html>`__ and/or `Realflight <https://ardupilot.org/dev/docs/sitl-with-realflight.html>`__  but funding will also be provided for a multicopter frame and camera gimbal if required.

Improve Web Server for file and parameter management
----------------------------------------------------

In ArduPilot 4.5.x we have networking support and a web server builtin
and we expect quite a few aicraft to use this in the future.

We would like the web server to be extended to allow for:

 - file management, with upload and download of files
 - parameter management, with a nice UI for displaying and editing parameters

the GSoC student for this project would need good JavaScript and HTML
skills to create a nice user interface. They would also need to know
some lua, although most of the code would be in JavaScript.


Projects Completed in past years
--------------------------------
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
