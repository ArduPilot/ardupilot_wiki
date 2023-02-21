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

List of CAN ESCs
================

+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+
+ Name                                                                                          + Avail                                                                                      + Ever Worked                                                                                                     +
+===============================================================================================+============================================================================================+=================================================================================================================+
+ `Zubax Orel 20 <https://files.zubax.com/products/io.px4.sapog/Zubax_Orel_20_Datasheet.pdf>`__ + `Yes <https://titaneliteinc.com/titanoc/index.php?route=product/product&product_id=995>`__ + Yes                                                                                                             +
+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+
+ `Zubax Myxa <https://zubax.com/products/myxa/>`__                                             + Yes                                                                                        + Yes                                                                                                             +
+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+
+ `Zubax Mitochondrik <https://zubax.com/products/mitochondrik>`__                              + Yes                                                                                        + Yes                                                                                                             +
+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+
+ `Holybro Kotleta20 <http://www.holybro.com/product/kotleta20/>`__                             + Yes                                                                                        + Yes                                                                                                             +
+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+
+ `AutoQuad ESC32 <http://autoquad.org/esc32/>`__                                               + No                                                                                         + No                                                                                                              +
+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+
+ `VESC <http://vedder.se/2015/01/vesc-open-source-esc/>`__                                     + `Yes <https://www.ollinboardcompany.com/product/vedder-s-speed-controller>`__              + No (`proposal <https://discuss.ardupilot.org/t/next-gen-esc-validation-and-integration-vesc-declined/12534>`__) +
+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+
+ `HiEnd Can Bus ESC <https://www.aerolab.de/esc-regler/hiend-can-bus-esc/>`__                  + Yes                                                                                        + ?                                                                                                               +
+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+
+ :ref:`Velocity ESC <common-velocity-can-escs>`                                                + Yes                                                                                        + Yes                                                                                                             +
+-----------------------------------------------------------------------------------------------+--------------------------------------------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------+

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

Preferably, the ESC can be configured via CAN bus using the `DroneCAN GUI Tool <common-uavcan-gui>`.

Autopilot Setup
===============

There are two parameters that determine which autopilot servo/motor channels are sent to the CAN escs:
For the examples below, the values are shown for CAN driver #1.

-  :ref:`CAN_D1_UC_NODE<CAN_D1_UC_NODE>` - which is the node ID of the autopilot sending the commands to the ESCs so that there can be differentiation between multiple sources on the CAN bus
-  :ref:`CAN_D1_UC_ESC_BM<CAN_D1_UC_ESC_BM>` - bitmask that determines which autopilot servo/motor output signals are sent to the DroneCAN ESCs

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
