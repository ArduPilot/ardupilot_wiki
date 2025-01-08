.. _common-ethernet-adapters:

=================
Ethernet Adapters
=================

Ardupilot has the ability to use Ethernet peripherals and networking (see :ref:`common-network`).  This page includes various switches and adapters known to work

.. image:: ../../../images/Net_Adapter.png
    :target: ../_images/Net_Adapter.png

Most H7 based autopilots do not include native ethernet support but ethernet networking capability can be added using an Ethernet Adapter (see `BotBlox DroneNet <https://botblox.io/dronenet-for-ardupilot/>`__ and `CubeLAN 8-Port Switch <https://irlock.com/products/cubelan-8-port-switch>`__ below) which provide connectivity using PPP protocol over the autopilot's serial port

Hardware
========

- `BotBlox SwitchBlox for Ardupilot <https://botblox.io/switchblox-for-ardupilot/>`__ : ethernet switch to allow connecting multiple devices together
- `BotBlox DroneNet for Ardupilot <https://botblox.io/dronenet-for-ardupilot/>`__ : ethernet switch with CAN, USART, RS485, and GPIO/PWM adapters allowing non-ethernet devices including autopilots to work over ethernet
- `BotBlox SwitchBlox Cable Adapter for Ardupilot <https://botblox.io/switchblox-cable-adapter-for-ardupilot/>`__ : adapter to ease the ethernet port differences across different device manufacturers
- `CubeNode ETH <https://docs.cubepilot.org/user-guides/cubenode/cubenode-eth>`__ : serial to ethernet adapter to allow non-ethernet autopilots to work over ethernet using PPP
- `CubeLAN 8-Port Switch <https://irlock.com/products/cubelan-8-port-switch>`__ : ethernet switch using the CubePilot preferred 5-pin connector

PPP Setup
=========

- PPP capability is not included by default on standard H7 autopilot firmware, so the `Custom Firmware Build Server <https://custom.ardupilot.org/>`__ must be used to build an firmware that includes it

.. image:: ../../../images/build-server-ppp.jpg
    :target: ../_images/build-server-ppp.jpg

.. note:: if using a local build environment (:ref:`building-the-code`), you can include PPP capability by using the ``\-\-enable-PPP`` waf configuration option when building the code for an autopilot locally.

- Connect one of the H7 based autopilot's serial ports to the ethernet switch's or Ethernet-to-PPP-adapter's USART port. For optimum performance a serial port with flow control should be used (e.g. normally SERIAL1 or SERIAL2).  In the following instructions SERIAL2 is used

Autopilot Setup
===============

See ``PPP configuration`` and ``ArduPilot Port Configuration`` sections of :ref:`common-network`. Be sure to set :ref:`NET_ENABLE<NET_ENABLE>` = 1 and reboot first. The Ethernet MAC configuration is internal to the Adapter itself.

Adapter Setup
=============

Currently available adapters use :ref:`DroneCAN <common-uavcan-setup-advanced>` to change or setup the Ethernet interface parameters such as MAC and IP addresses, NET mask, etc. Once setup/changed, the DroneCAN connection can be removed, if desired. When attached to a DroneCAN enabled port on the autopilot, you can use :ref:`Mission Planner<dronecan-uavcan-slcan>` or the :ref:`DroneCAN GUI <dronecan-uavcan-slcan>` tool to explore/change its Ethernet parameters.

In addition, it may implement its own Web Browser interface for status and configuration on the network similar to:

.. image:: ../../../images/PPP_web_server.jpg
    :target: ../_images/PPP_web_server.jpg

Video
=====

.. youtube:: bN6iDP4Zjzg

