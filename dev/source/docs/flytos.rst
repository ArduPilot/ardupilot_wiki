.. _flytos:

======
FlytOS
======

`FlytOS <https://flytbase.com>`_ is a software framework which provides Drone APIs and SDKs for building high-level drone applications such as aerial delivery, precision agriculture, surveys, photography, industrial inspections and disaster management. It is designed to enable drone-developers build advanced drone applications using its open APIs.

FlytOS is based on Linux and ROS (Robot Operating System), making it an ideal platform for building commercial as well as research orientated drone applications. It supports a wide range of hardware options such as *Raspberry Pi 3, Odroid XU4, Nvidia TX1, Intel Edison, Intel Aero and FlytPOD*. It uses MAVLink to communicate with the autopilot, and exposes high level FlytAPIs in *ROS, CPP, Python, REST and Websocket*. This makes it easy to build high-level applications using computer-vision, machine-learning, cloud-connectivity and enables developers to create their custom user-interfaces on web/mobile devices of their choice. FlytOS also has modules to manage payloads, security and updates. The modular design of FlytOS allows for integration with external ROS/Linux libraries and custom data plumbing between onboard and offboard apps. FlytOS aims to provide a standard language for the drone application developers to talk to their drones.

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

`FlytSIM <http://docs.flytbase.com/docs/FlytOS/Developers/Flytsim.html>`_ is a ROS/Gazebo based simulator to test applications built using FlytAPIs. This allows developers to build and test drone applications, safely and efficiently, minimising the requirement for flight-tests.


Vision-based Object-Tracking and Following
==========================================

FlytOS comes bundled with Vision-based Object-Tracking and Following module. To learn more about it, chekout `this blog <http://blogs.flytbase.com/computer-vision-for-drones-part-2/>`_.

.. youtube:: bom1VEcxwEA
        :width: 100%

|br|

`Download FlytOS <https://my.flytbase.com/downloads/>`_ now, and get started with Companion Computer of your choice. Follow `FlytOS installation instructions <http://docs.flytbase.com/docs/FlytOS/GettingStarted/FlytOSInstallationGuide.html>`_ and post your queries/issues on FlytOS `forum <http://forums.flytbase.com/>`_. Use the detailed `API reference <http://api.flytbase.com/>`_ to develop drone apps using your preferred language (Python, C++, ROS, RESTful, Websocket, etc.). A rich set of `sample applications <https://github.com/flytbase/flytsamples>`_ are available for the developers to get started quickly.

Keep yourself up-to-date with latest updates by joining FlytOS `Facebook community <https://goo.gl/MWlexy>`_ of users and developers. Visit FlytOS `youtube channel <https://goo.gl/DzfW1V>`_ to see more drone applications in action.

.. |br| raw:: html

   <br />
