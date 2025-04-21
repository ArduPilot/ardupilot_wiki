.. _common-ethernet-adapters:

==============================
Ethernet Adapters and Switches
==============================

Ardupilot has the ability to use Ethernet peripherals and networking (see :ref:`common-network`).  This page includes various switches and adapters known to work

.. image:: ../../../images/Net_Adapter.png
    :target: ../_images/Net_Adapter.png

Most H7 based autopilots do not include native ethernet support but ethernet networking capability can be added using an Ethernet Adapter (see :ref:`BotBlox DroneNet <common-botblox-dronenet>` and :ref:`CubePilot CubeNode ETH <common-cubepilot-cubenodeeth>` below) which provide connectivity using :ref:`PPP protocol <ppp-config>` over the autopilot's serial port.

Hardware
========

- `BlueRobotics Ethernet Switch <https://bluerobotics.com/store/comm-control-power/tether-interface/ethswitch/>`__ : 5-port ethernet switch designed in collaboration with BotBlox
- `BotBlox SwitchBlox for Ardupilot <https://botblox.io/switchblox-for-ardupilot/>`__ : ethernet switch to allow connecting multiple devices together
- :ref:`BotBlox DroneNet for Ardupilot <common-botblox-dronenet>`: ethernet switch with CAN, USART, RS485, and GPIO/PWM adapters allowing non-ethernet devices including autopilots to work over ethernet
- `BotBlox SwitchBlox Cable Adapter for Ardupilot <https://botblox.io/switchblox-cable-adapter-for-ardupilot/>`__ : adapter to ease the ethernet port differences across different device manufacturers
- :ref:`CubePilot CubeNode ETH <common-cubepilot-cubenodeeth>`: serial to ethernet adapter to allow non-ethernet autopilots to work over ethernet using PPP
- `CubePilot CubeLAN 8-Port Switch <https://docs.cubepilot.org/user-guides/switch/cubelan-8-port-switch>`__ : ethernet switch using the CubePilot preferred 5-pin connector

.. toctree::
    :hidden:

    BotBlox DroneNet <common-botblox-dronenet>
    CubePilot CubeNodeETH <common-cubepilot-cubenodeeth>
