.. _common-canbus-setup-advanced:

=============
CAN Bus Setup
=============

This article shows how to setup CAN bus and what options users have
to accomplish the setup suitable for their specific needs.

.. tip::

   The :ref:`UAVCAN setup page is here <common-uavcan-setup-advanced>`.

Overview
========

A Controller Area Network (CAN bus) is a robust vehicle bus standard designed
to allow microcontrollers and devices to communicate with each other in
applications without a host computer. It is a message-based protocol, designed
originally for multiplex electrical wiring within automobiles to save on copper,
but is also used in many other contexts.

All nodes are connected to each other through a two wire bus. The wires are
120 Ω nominal twisted pair.

Most autopilots that run ArduPilot have either one or two CAN interfaces
for connection of different devices.
The setup of the interfaces can be made in a way that will provide redundancy or
maximum throughput or a mix of both.
This is accomplished with a three layer approach, where apart from the physical
interface there exist a driver layer that represents a specific protocol and a
software layer (ArduPilot) that communicates on CAN bus through these drivers.

Each physical interface can be virtually connected to one of the drivers that
represent protocols to be used.
For example, the most common scenario will be one driver of UAVCAN with both
interfaces connected to it. Such setup will provide redundancy for devices with
two CAN interfaces and full functionality for devices with one CAN interface.

Configuration settings
======================

Enabling CAN interfaces & Drivers
---------------------------------

Each physical port can be turned off or connected to corresponding driver with
parameter **CAN_PX_DRIVER**, where X is the number of port.
The value of this parameter is the id of driver that will be associated with this
port (interface).

If the flight controller has one port then set **CAN_P1_DRIVER** accordingly and
leave **CAN_P2_DRIVER** unset.

.. image:: ../../../images/can-drivers-parameters-single.png

For dual ports set **CAN_P1_DRIVER** and **CAN_P2_DRIVER** or if the flight controller has only CAN2
I.E. Sparky2 then set the ports as follows.

.. image:: ../../../images/can-drivers-parameters-dual.png
    :target: ../_images/can-driver-parameters.png

After change of any **CAN_PX_DRIVER** the autopilot has to be rebooted for the changes to take place.

Configuration of CAN interfaces
-------------------------------

.. image:: ../../../images/can-drivers-parameters-set.png

After enabling the interface and reboot more parameters can be set for each
of the enabled interfaces.

These are:

-  **CAN_PX_BITRATE** -   Sets the desired rate of transfer on this interface Usually the bitrate used by default is 1 Mbit.
-  **CAN_D1_UC_ESC_BM** - Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN
-  **CAN_D1_UC_NODE** -	  UAVCAN node address of the autopilot
-  **CAN_D1_UC_SRV** -    Bitmask with one set for channel to be transmitted as a servo command over UAVCAN
-  **CAN_D1_UC_SRV_RT** - Maximum transmit rate for servo outputs
