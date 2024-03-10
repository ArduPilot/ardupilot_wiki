.. _common-escs-and-motors:

===============
ESCs and Motors
===============

.. image:: ../../../images/motors-and-escs-topimage.png

ArduPilot supports a wide variety of ESCs, motors, and Electronic Fuel systems.  The pages below provide setup instructions for the most popular types

Motors
======
.. toctree::
    :maxdepth: 1

[site wiki="copter,rover"]
    Brushed motors and ESCs <common-brushed-motors>
[/site]
[site wiki="copter"]
    Booster motor <booster-motor>
[/site]
[site wiki="plane"]
    ICE (Internal Combustion Engines) <common-ice>
[/site]
[site wiki="rover"]
    Thrusters (for boats) <thrusters>
    Torqeedo Electric Motor (for boats) <common-torqeedo>
    Trolling motors <trolling-motor>
[/site]


ESC for Brushless Motors 
========================

ESCs (Electronic Speed Controls) have many different protocols for communicating with the autopilot for motor speed control, and and can also provide telemetry information on motor rpm, battery voltage. The
capabilities will vary with each individual ESC model. Some ESCs have specialized firmware that allows easy configuration of operating parameters. The following sections
explain the required ArduPilot setup to utilize the protocols, telemetry, and setup programs that various ESCs utilize.
See :ref:`common-esc-guide` for a guide to terminology.

Protocols
---------

.. toctree::
    :maxdepth: 1

    PWM, OneShot and OneShot125 ESCs <common-brushless-escs>
    DShot ESCs <common-dshot-escs>
    KDE CAN ESCs <common-kde-can-escs>
    DroneCAN ESCs <common-uavcan-escs>
    Currawong Velocity ESCs <common-velocity-can-escs>
    Hobbywing DroneCAN ESCs <common-hobbywing-dronecan-esc>
    FETtec OneWire ESCs <common-fettec-onewire>

.. toctree::
   :hidden:

   common-hargrave-dronecan-escs


ESCs using BLHeli32 or BLHeli-S Configuration Firmware
------------------------------------------------------

.. toctree::
    :maxdepth: 1

    BLHeli/BLHeli32 Capable ESCs <common-blheli32-passthru>


Telemetry
---------

.. toctree::
    :maxdepth: 1

    ESC Telemetry <common-esc-telemetry>
    Hobbywing Telemetry Hub (DatalinkV2)(uses LUA driver) <https://www.hobbywingdirect.com/products/data-link-v2>

.. note:: Currently ArduPilot only supports telemetry on BLHeli or DroneCAN/CAN ESCs, not throttle signal wire reported telemetry that some single unit ESCs report.


[site wiki="plane"]

Electronic Fuel Injectors (EFI)
===============================

.. toctree::
    :maxdepth: 1

    Electronic Fuel Injection <common-efi>


[/site]
ESC wiring and Large QuadPlane ESC Issues
=========================================

.. toctree::
    :maxdepth: 1

    ESC Grounding and Wiring Considerations<common-esc-grounding>
    ESC Signalling Issues <common-esc-issues>
    ESC Terminology <common-esc-guide>


