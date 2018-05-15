.. _common-uavcan-setup-advanced:

============
UAVCAN Setup
============

This article provides guidance to setup UAVCAN protocol on Ardupilot.

.. tip::

   The UAVCAN protocol should be enabled first. Please refer to the
   :ref:`CAN Bus Setup <common-canbus-setup-advanced>`

Overview
========

UAVCAN is a lightweight protocol designed for reliable communication
in aerospace and robotic applications via CAN bus.
The UAVCAN network is a decentralized peer network, where each peer
(node) has a unique numeric identifier - node ID and that is only one
parameter needs to be set for basic setup.

Detailed description of protocol can be found at http://uavcan.org/

**UAVCAN driver for Aurdupilot do not support auto node numbering in
current version. All nodes should have the ID explicitly set.**

Configuration settings
======================

There are three parameters present at the moment in CAN category of setting:

-  **CAN_DX_UC_NODE** - which is the node ID of the autopilot
-  **CAN_D1_UC_ESC_BM** - bitmask that enables sending of ESC commands
-  **CAN_D1_UC_SRV_BM** - bitmask that enables sending of servo commands

.. image:: ../../../images/uavcan-main-settings.png
    :target: ../_images/uavcan-main-settings.png
    
In a bitmap mask, each position in the binary number represents an ESC or servo ID
that the command will be generated for. In case of copters, usually the ESC bitmask
should be filled and in case of planes - main one is for servo, though any mix is
possible.

To reduce bandwidth, the CAN_D1_UC_ESC_BM and CAN_D1_UC_SRV_BM params should be set
to enable only the motor and servo channels you need CAN signals sent to.

-  Example: For a configuration of CAN servos on channels 1,2,4 and ESC motor on channel 3, set:
-  Example: **CAN_D1_UC_SRV_BM** = 0x0B
-  Example: **CAN_D1_UC_ESC_BM** = 0x04


GNSS receiver configuration settings
====================================

If there is a GNSS connected to UAVCAN network, it has to be enabled in **GPS**
subgroup of parameters.
The **TYPE** parameter should be set to 9 for corresponding GNSS receiver in autopilot.

.. image:: ../../../images/uavcan-gnss-settings.png
    :target: ../_images/uavcan-gnss-settings.png
    

   
   

    
