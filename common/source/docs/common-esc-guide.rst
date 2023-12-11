.. _common-esc-guide:

===============================
ESC (Electronic Speed Controls)
===============================

ESCs are available for either brushed (DC) or brushless (AC) motors. :ref:`Brushed motors<common-brushed-motors>` are most commonly used in toy-class planes, multicopters, rovers, boats, and balance bots. Most other larger vehicles use brushless speed controllers.

ESCs are available in a confusing array of communication protocols, programming capability, telemetry, and multi unit configurations. Below is a guide to some of the terms and capabilities.

Control Connection To Autopilot
===============================

- Single signaling wire to autopilot servo/motor output which can use pulse or digital communication protocols
- :ref:`DroneCAN <common-uavcan-escs>` or CAN, which provides a more robust connection with greater length limits at the expense of more wiring.

Communication Protocols
=======================

The ESC may be compatible with one or more of the following protocols:

- :ref:`PWM, OneShot, OneShot125<common-brushless-escs>` are pulse based protocols. Almost all ESCs are compatible with PWM signaling. And this is the default configuration of ArduPilot firmware for serov/motor outputs.
- :ref:`DShot<common-dshot-escs>` is a digital protocol in which the speed is encoded as digital commands to the ESC. Speed of the signaling can be DShot150, DShot300, etc. depending on the ESC's capability. Other ESC commands for ESC led control, etc. can be sent to the ESC if it has the capability.
- :ref:`Bi-Directional DShot<bidir-dshot>` is a variation of Dshot that allows the ESC to return to the autopilot telemetry data. What type of data is ESC dependent. Sometimes referred to as BDShot.
- :ref:`FETtec OneWire<common-fettec-onewire>` ESC is another bi-directional protocol that sends speed commands and returns telemetry.

Telemetry
=========

As stated above some ESCs have the ability to return telemetry from the ESC. In addition to the Bi-Directional DShot and FETTec protocols, some DroneCAN/CAN ESCs have this capability. Some ESCs can provide motor RPM telemetry via an additional wire that is connected to the autopilot on one of its UART ports. See :ref:`common-esc-telemetry` for more info.

ESC Programming
===============

The ESC sometimes has parameters which can be programmed: timing, motor direction, battery cutoff, startup ramp, etc. Depending on the individual ESC, these can be programmed in different ways:

- some via a sequence of throttle commands after entering a programming mode upon startup. These vary widely, ESC to ESC.
- some via an interface program like :ref:`BLHeli<common-blheli32-passthru>`.
- DroneCAN ESCs via DroneCAN parameters using the :ref:`DroneCAN GUI<common-uavcan-gui>` or Mission Planner's SETUP->Optional Hardware->DroneCAN/UAVCAN setup window to access the parameters.
- some use a supplied setup software program