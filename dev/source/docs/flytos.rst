.. _flytos:

======
FlytOS
======

`FlytOS <https://flytbase.com>`__ is a software framework which provides Drone APIs and SDKs for building high-level drone applications such as aerial delivery, precision agriculture, surveys, photography, industrial inspections and disaster management. It is designed to enable drone-developers build advanced drone applications using its open APIs.

FlytOS is based on Linux and ROS (Robot Operating System), making it an ideal platform for building commercial as well as research orientated drone applications. It supports a wide range of hardware options such as *Raspberry Pi 3, Odroid XU4, Nvidia TX1, Intel Edison, Intel Aero and FlytPOD*. It uses MAVLink to communicate with the autopilot, and exposes high level FlytAPIs in *ROS, CPP, Python, REST and Websocket*. |br|

.. figure:: ../images/flytosccsupport.jpg
    :target: ../_images/flytosccsupport.jpg
    :align: center

    Supported Companion Computers

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

`FlytSDK <http://docs.flytbase.com/docs/FlytOS/Developers/BuildingCustomApps.html#remote-apps>`__ is the software development kit for web and android developers. A number of `sample applications <https://github.com/flytbase/flytsamples>`__ are available on github, that can be used as templates/reference to build custom applications.

`FlytSIM <http://docs.flytbase.com/docs/FlytOS/Developers/Flytsim.html>`__ is a ROS/Gazebo based simulator to test applications built using FlytAPIs. This allows developers to build and test drone applications, safely and efficiently, minimizing the requirement for flight-tests.

Supported Companion Computers
=============================

* Raspberry Pi3 `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/RaspiGuide.html>`__
* Odroid-XU4 `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/OdroidGuide.html>`__
* Nvidia-TX1 `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/TX1Guide.html>`__
* FlytPOD `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/FlytPODGuide.html>`__
* Intel Edison `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/EdisonGuide.html>`__
* Intel Aero `[installation instructions] <http://docs.flytbase.com/docs/FlytOS/GettingStarted/AeroGuide.html>`__
* Intel Joule (*coming soon*)
* Qualcomm Snapdragon Flight (*coming soon*)
* Nvidia-TX2 (*coming soon*)

.. _supported_languages:

Supported Languages
===================

FlytOS offers Drone APIs for building applications with onboard as well as remote components. These drone APIs not only provide you, the control over vehicle's navigation, payloads (gimbal, camera, etc.) but also has few inbuilt AI/ML modules such as object detection and tracking, obstacle detection etc.

Onboard APIs
------------

These are APIs available onboard the companion computer and can be used for developing domain specific intelligence and business logic. Typical candidates are tasks requiring high reliability, low latency and relatively low processing power as offered by the onboard computers.

* `C++ <http://api.flytbase.com/?cpp#>`__
* `Python <http://api.flytbase.com/?python#>`__
* `ROSCpp <http://api.flytbase.com/?cpp--ros#introduction>`__
* `ROSPy <http://api.flytbase.com/?python--ros#introduction>`__

Click on corresponding links to know more about building custom apps in `cpp <http://docs.flytbase.com/docs/FlytOS/Developers/BuildingCustomApps/OnboardCPP.html#write-onboard-cpp>`__ and `python <http://docs.flytbase.com/docs/FlytOS/Developers/BuildingCustomApps/OnboardPython.html#write-onboard-python>`__.

Remote APIs
-----------

These are APIs for building web/mobile apps for remote devices and are helpful for creating custom User Interfaces specific to the application as well as for integrating any off-board processing. They are available as `RESTful <http://api.flytbase.com/?javascript--REST#introduction>`__ and `WebSocket <http://api.flytbase.com/?javascript--Websocket#introduction>`__ requests, where typically REST is used for sending commands to the drone and WebSocket for getting continual data stream (telemetry). Click one of the below links, to know more about building custom apps for web and mobile.

* `JS <http://docs.flytbase.com/docs/FlytOS/Developers/BuildingCustomApps/RemoteWeb.html#write-remote-web>`__
* `Java (Android) <http://docs.flytbase.com/docs/FlytOS/Developers/BuildingCustomApps/RemoteMobile.html#write-remote-mobile>`__

Sample Applications
===================

We have made available, a few sample apps to help you get started with drone application development. You can find them on github at `FlytSamples github repository <https://github.com/flytbase/flytsamples>`__. These sample apps are written in all of the above :ref:`supported languages <supported_languages>`. Web/android developers could begin with a simple `Joystick App <https://github.com/flytbase/flytsamples/tree/master/Mobile-Apps/Java-Apps/Joystick>`__. A couple of easy-to-understand `CPP/Python/ROS based apps <https://github.com/flytbase/flytsamples/tree/master/CPP-Python-ROS-Apps>`__ are also available.

Vision-based Object-Tracking and Following
------------------------------------------

FlytOS comes bundled with Vision-based Object-Tracking and Following module. To learn more about it, checkout `this blog <http://blogs.flytbase.com/computer-vision-for-drones-part-2/>`__.

.. youtube:: bom1VEcxwEA
        :width: 100%

|br|

Deep Learning with Nvidia
-------------------------

Using FlytOS on Nvidia-TX1/Nvidia-TX2 opens up possible integration of deep learning applications with drone. To begin with, you could install *caffe*, a popular deep learning framework by follwing our `deep learning tutorial <https://goo.gl/HwNMuY>`__. We also have a sample `object classification and tracking <https://github.com/flytbase/flytos_tx1>`__ example using caffe. `Read more <https://goo.gl/ZReoJ7>`__.

.. youtube:: wSFYOw4VIYY
        :width: 100%

|br|

GPS based Object Following
--------------------------

This android app would enable you to control your drone to follow you wherever you go based on your device's GPS location. Take a look at the `GPS Follow Me code <https://github.com/flytbase/flytsamples/tree/master/Mobile-Apps/Java-Apps/Follow_me>`__, install it in your mobile and see FlytOS in action.


SONAR based obstacle detection
------------------------------

You could enable your drone with a minimalistic obstacle detection by using SONAR, capturing its data, integrating it with FlytOS and eventually maneuvering the drone through an obstacle course. We have provided a `sample implementation <https://github.com/flytbase/flytsamples/tree/master/Sample-Projects/sonar_obstacle_sensor>`__, of using Arduino to trigger SONAR and then transmit the captured data to a companion computer. Using this data, you could write a simple onboard ROS/cpp/python app navigating the drone using FlytAPIs.



Important Links
---------------

* `FlytOS Download <https://my.flytbase.com/downloads>`__
* `FlytOS Documentation <http://docs.flytbase.com/docs/FlytOS/GettingStarted/FlytOSInstallationGuide.html>`__
* `FlytAPI Reference <http://api.flytbase.com>`__
* `Sample Applications <https://github.com/flytbase/flytsamples>`__
* `Discussion Forum <http://forums.flytbase.com>`__
* `ArduPilot Discord Chat <https://ardupilot.org/discord>`__
* `Facebook Community <https://goo.gl/MWlexy>`__
* `Youtube Channel <https://goo.gl/DzfW1V>`__

Mail us at support@flytbase.com for dedicated support and visit https://flytbase.com for more information.
 
.. |br| raw:: html

   <br />
