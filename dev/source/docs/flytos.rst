.. _flytos:

======
FlytOS
======

`FlytOS <https://flytbase.com>`_ is a software framework which provides Drone APIs and SDKs for building high-level drone applications such as aerial delivery, precision agriculture, surveys, photography, industrial inspections and disaster management. It is designed to enable drone-developers build advanced drone applications using its open APIs.

FlytOS is based on Linux and ROS (Robot Operating System), making it an ideal platform for building commercial as well as research orientated drone applications. It supports a wide range of hardware options such as *Raspberry Pi 3, Odroid XU4, Nvidia TX1, Intel Edison, Intel Aero and FlytPOD*. It uses MAVLink to communicate with the autopilot, and exposes high level FlytAPIs in *ROS, CPP, Python, REST and Websocket*. |br|
*Insert companion computer image here*
This makes it easy to build high-level applications using computer-vision, machine-learning, cloud-connectivity and enables developers to create their custom user-interfaces on web/mobile devices of their choice. FlytOS also has modules to manage payloads, security and updates. The modular design of FlytOS allows for integration with external ROS/Linux libraries and custom data plumbing between onboard and offboard apps. FlytOS aims to provide a standard language for the drone application developers to talk to their drones.

.. youtube:: CZFVWDN5Gcc
        :width: 100%

|br|

.. figure:: ../images/FlytOSArch.png
    :target: ../_images/FlytOSArch.png
    :align: center

    FlytOS Architecture Diagram

Developer Tools
===============

FlytOS provides several developer tools, such as FlytSDK and FlytSIM, to further help developers quickly get started.

`FlytSDK <http://docs.flytbase.com/docs/FlytOS/Developers/BuildingCustomApps.html#remote-apps>`_ is the software development kit for web and android developers, to help them get started quickly with FlytOS. A number of `sample applications <https://github.com/flytbase/flytsamples>`_ are available on github, that can be used as templates/reference to build custom applications.

`FlytSIM <http://docs.flytbase.com/docs/FlytOS/Developers/Flytsim.html>`_ is a ROS/Gazebo based simulator to test applications built using FlytAPIs. This allows developers to build and test drone applications, safely and efficiently, minimizing the requirement for flight-tests.

Supported Companion Computers
=============================

* Raspberry Pi3 `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/RaspiGuide.html>`_
* Odroid-XU4 `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/OdroidGuide.html>`_
* Nvidia-TX1 `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/TX1Guide.html>`_
* FlytPOD `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/FlytPODGuide.html>`_
* Intel Edison `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/EdisonGuide.html>`_
* Intel Aero `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/AeroGuide.html>`_
* Intel Joule (*launching soon*)
* Qualcomm Snapdragon Flight (*launching soon*)
* Nvidia-TX2 (*launching soon*)

.. _supported_languages:

Supported Languages
===================
* `PYTHON <http://docs.flytbase.com/docs/FlytOS/Developers/BuildingCustomApps/OnboardPython.html#write-onboard-python>`_
* `C++ <http://docs.flytbase.com/docs/FlytOS/Developers/BuildingCustomApps/OnboardCPP.html#write-onboard-cpp>`_
* `ROS <http://api.flytbase.com/?shell#introduction>`_
* `ROSCPP <http://api.flytbase.com/?cpp--ros#introduction>`_
* `ROSPY <http://api.flytbase.com/?python--ros#introduction>`_
* `JS REST <http://api.flytbase.com/?javascript--REST#introduction>`_
* `JS WEBSOCKET <http://api.flytbase.com/?javascript--Websocket#introduction>`_

Sample Applications
===================

We have made available, a few sample apps to help you get started with drone application development. You can find them on github at `FlytSamples github repository <https://github.com/flytbase/flytsamples>`_. These sample apps are written in all of the above supported languages. Web/android developers could begin with a simple `Joystick App <https://github.com/flytbase/flytsamples/tree/master/Mobile-Apps/Java-Apps/Joystick>`_. A couple of easy-to-understand `CPP/Python/ROS based apps <https://github.com/flytbase/flytsamples/tree/master/CPP-Python-ROS-Apps>`_ are also available.

GPS based Object Following
--------------------------

This android app would enable you to control your drone to follow you wherever you go based on your device's GPS location. Take a look at the `GPS Follow Me code <https://github.com/flytbase/flytsamples/tree/master/Mobile-Apps/Java-Apps/Follow_me>`_, install it in your mobile and see FlytOS in action.

Vision-based Object-Tracking and Following
------------------------------------------

FlytOS comes bundled with Vision-based Object-Tracking and Following module. To learn more about it, checkout `this blog <http://blogs.flytbase.com/computer-vision-for-drones-part-2/>`_.

.. youtube:: bom1VEcxwEA
        :width: 100%

|br|

SONAR based obstacle detection
------------------------------

One could enable his/her drone with a minimalistic obstacle detection by using SONAR, capturing its data, integrating it with FlytOS and eventually maneuvering the drone through an obstacle course. We have provided a `sample implementation <https://github.com/flytbase/flytsamples/tree/master/Sample-Projects/sonar_obstacle_sensor>`_, of using Arduino to trigger SONAR and then transmit the captured data to a companion computer. Using this data, one could write a simple onboard ROS/cpp/python app navigating the drone using FlytAPIs.


Important Links
---------------

`Download FlytOS <https://my.flytbase.com/downloads/>`_ now, and get started with Companion Computer of your choice. |br|
You can refer `FlytOS installation instructions <http://docs.flytbase.com/docs/FlytOS/GettingStarted/FlytOSInstallationGuide.html>`_  for a step-by-step installation guide  |br|
You can post your queries/issues on FlytOS `forum <http://forums.flytbase.com/>`_. |br|
Use the detailed `API reference <http://api.flytbase.com/>`_ to develop drone apps using your preferred language (Python, C++, ROS, RESTful, Websocket, etc.). |br|
We have made available, a few `sample applications <https://github.com/flytbase/flytsamples>`_ are available for the developers to get started quickly.

Keep yourself up-to-date with latest updates by joining FlytOS `Facebook community <https://goo.gl/MWlexy>`_ of users and developers. Visit FlytOS `youtube channel <https://goo.gl/DzfW1V>`_ to see more drone applications in action. |br|
Mail us at support@flytbase.com for dedicated support and visit https://flytbase.com for more information.
 
.. |br| raw:: html

   <br />
