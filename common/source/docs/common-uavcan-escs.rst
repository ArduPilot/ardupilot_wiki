.. _common-uavcan-escs:

=============
DroneCAN ESCs
=============

Copter, Plane and Rover support `DroneCAN <https://dronecan.org>`__ Electronic Speed Controllers
(ESCs) that allow two-way communication with the autopilot
enabling potentially easier setup and in-flight monitoring of ESC and
motor health.

..  youtube:: LnUmYgAINBc
    :width: 100%

List of DroneCAN ESCs
=====================


- :ref:`Currawong Velocity ESC <common-velocity-can-escs>`
- :ref:`Hargrave Technologies DroneCAN ESCs <common-hargrave-dronecan-escs>`
- :ref:`Hobbywing CAN ESCs <common-hobbywing-dronecan-esc>`
- `Holybro Kotleta20 <https://holybro.com/products/kotleta20>`__
- :ref:`KDE UVC ESCs <common-kde-can-escs>`
- `Zubax Mitochondrik <https://zubax.com/products/mitochondrik>`__
- `Zubax Myxa <https://zubax.com/products/myxa/>`__
- `Zubax Orel 20 <https://files.zubax.com/products/io.px4.sapog/Zubax_Orel_20_Datasheet.pdf>`__

Connecting to the Flight Controller
===================================

.. image:: ../../../images/Pixhawk_UAVCAN_ESC.jpg
    :target: ../_images/Pixhawk_UAVCAN_ESC.jpg

One ESC (it does not matter which) should be connected to the autopilot's
CAN port using a 4-pin DF13 to 4-pin DroneCAN adapter cable. Each
subsequent ESC should be connected to the previous using a 4-pin
DroneCAN cable.  The final ESC should have a CAN bus terminator plugged
into one of its 4-pin DroneCAN ports.

An FTDI Cable connection to the ESC's debug port is only required for set-up if the ESC does not present its parameters via DroneCAN. In that case, contact the manufacturer for detailed instructions.

Preferably, the ESC can be configured via CAN bus using the :ref:`DroneCAN GUI Tool <common-uavcan-gui>`.

Autopilot Setup
===============

There are several parameters that determine which autopilot servo/motor channels are sent to the DroneCAN ESCs:
For the examples below, the values are shown for DroneCAN driver #1 using CAN Port #1

-  :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1, which assigns driver1 to port1
-  :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` = 1 (DroneCAN protocol)
-  :ref:`CAN_D1_UC_NODE<CAN_D1_UC_NODE>` - which is the node ID of the autopilot sending the commands to the ESCs so that there can be differentiation between multiple sources on the CAN bus. This is normally automatically set during discovery, but can be altered for advanced configurations (multiple sources on the bus).
-  :ref:`CAN_D1_UC_ESC_BM<CAN_D1_UC_ESC_BM>` - bitmask that determines which autopilot servo/motor output signals are sent to the DroneCAN ESCSs
-  :ref:`CAN_D1_UC_ESC_RM<CAN_D1_UC_ESC_RV>` - bitmask that designates which autopilot servo/motor outputs have reversible DroneCAN ESCs, allowing both positive and negative control values to be sent.

Logging and Reporting
---------------------

DroneCAN ESCs provide information back to the autopilot which is recorded in the autopilot's onboard log's CESC messages and can be viewed in any :ref:`ArduPilot compatible log viewer <common-logs>`.  This information includes:

- Error Count
- Voltage
- Current
- Temperature
- RPM
- Power (as a percentage)

The RCOU messages are also written to the onboard logs which hold the requested output level sent to the ESCs expressed as a number from 1000 (meaning stopped) to 2000 (meaning full output).

Additional information
======================

`Zubax Sapog wiki page <https://kb.zubax.com/display/MAINKB/Using+Sapog-based+ESC+with+ArduPilot>`__,
`Sapog reference manual <https://files.zubax.com/products/io.px4.sapog/Sapog_v2_Reference_Manual.pdf>`__,
and `ESC firmware <https://github.com/PX4/sapog>`__.
