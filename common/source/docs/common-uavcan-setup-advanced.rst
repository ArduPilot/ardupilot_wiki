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

**UAVCAN driver for Aurdupilot do not support auto node numbering in
current version. All nodes should have the ID explicitly set.**

Configuration settings
======================

There are three parameters present at the moment in CAN category of setting:

-  **CAN_DX_UC_NODE** - which is the node ID of the autopilot
-  **CAN_D1_UC_ESC_BM** - bitmask that enables sending of ESC commands
-  **CAN_D1_UC_SRV_BM** - bitmask that enables sending of servo commands

figure:: ../../../images/uavcan-main-settings.png

In a bitmap mask, each position in the binary number represents an ESC or servo ID
that the command will be generated for. In case of copters, usually the ESC bitmask
should be filled and in case of planes - main one is for servo, though any mix is
possible.

GNSS receiver configuration settings
====================================

If there is a GNSS connected to UAVCAN network, it has to be enabled in **GPS**
subgroup of parameters.
The **TYPE** parameter should be set to 9 for corresponding GNSS receiver in autopilot.

figure:: ../../../images/uavcan-gnss-settings.png
