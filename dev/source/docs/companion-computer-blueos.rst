.. _companion-computer-blueos:

=============================
BlueOS Installation and Setup
=============================

This section explains how to install and configure BlueOS on an RPI compute module connected to an autopilot and camera gimbal via ethernet.

`BlueOS's official wiki is here <https://blueos.cloud/docs/latest/usage/overview/>`__

Recommended Hardware
--------------------

.. image:: ../images/blueos-hardware.jpg
    :target: ../_images/blueos-hardware.jpg
    :width: 400px

- :ref:`ArduPilot compatible flight controller <common-autopilots>`
- `RPI4 I/O board <https://www.raspberrypi.com/products/compute-module-4-io-board/>`__ or `RPI5 I/O board <https://www.raspberrypi.com/products/compute-module-5-io-board/>`__
- `RPI CM4 <https://www.raspberrypi.com/products/compute-module-4/>`__ or `CM5 <https://www.raspberrypi.com/products/compute-module-5/>`__
- `Ochin Tiny Carrier Board V2 <https://www.seeedstudio.com/Ochin-Tiny-Carrier-Board-V2-for-Raspberry-Pi-CM4-p-5887.html>`__
- :ref:`BotBlox DroneNet ethernet switch <common-botblox-dronenet>` or :ref:`CubeNodeETH <common-cubepilot-cubenodeeth>` and `CubeLan 8-port ethernet switch <https://docs.cubepilot.org/user-guides/switch/cubelan-8-port-switch>`__ (see :ref:`ethernet adapters <common-ethernet-adapters>`)
- (optionally) Ethernet enabled camera gimbal (e.g. :ref:`Siyi A8 <common-siyi-zr10-gimbal>`)
- (optionally) 4G/LTE modem

Setup
-----

Please follow the links below for detailed setup instructions.

.. toctree::
    :maxdepth: 1

    BlueOS Installation <companion-computer-blueos-install>
    BlueOS 4G/LTE Telemetry Setup <companion-computer-blueos-lte-telem>
    BlueOS Live Video Setup <companion-computer-blueos-live-video>
