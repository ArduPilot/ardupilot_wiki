.. _USB-IDs:

====================
USB IDs in ArduPilot
====================

ArduPilot uses a range of USB IDs for various flight controllers and
devices. This document describes the IDs in active use as well as past
IDs that may be on some boards.

Composite Devices
=================

In the information below some IDs are marked as being for composite
USB devices. These are for boards which will present two serial
interfaces for one USB connection. Boards that use composite IDs will
have both OTG1 and OTG2 in their UART_ORDER declaration in the
hwdef.dat for the board. These devices should have a different USB
VID/PID pair from non-composite boards to allow the correct driver to
be loaded on Microsoft Windows.

When a board uses a composite USB ID two serial ports are presented to
the host. The user can configure what protocols these two ports use
using the SERIALn_PROTOCOL parameter. SERIAL0_PROTOCOL is the first
USB interface, and SERIALn_PROTOCOL for the largest value of n on the
board is the 2nd USB interface. Typically the 2nd interface is used to
give a SLCAN interface for CAN diagnostics and configuration.

VID=0x0483 PID=0x5740
=====================

This ID is one from ST Microelectronics, and was used by ArduPilot in
the initial ChibiOS port until January 2020. It is a non-composite
serial ID.

VID=0x1209 PID=0x5740
=====================

This ID is one from http://pid.codes/, and is used by ArduPilot since
December 2019. It is a composite serial ID, with interface 0 labelled
as MAVLink and interface 2 labelled as SLCAN. It is the default USB ID
for boards capable of composite USB devices.

VID=0x1209 PID=0x5741
=====================

This ID is one from http://pid.codes/, and is used by ArduPilot since
January 2020. It is a non-composite serial ID. It is the default USB
ID for boards non-composite USB devices, including bootloaders and all
F4 based boards.

VID=0x16D0 PID=0x0E65
=====================

This ID is one from https://www.mcselec.com, and is reserved for use
by ArduPilot in the future.

VID=0x26AC PID=various
======================

The 0x26AC vendor ID is owned by 3D Robotics, and was used for most
ArduPilot compatible boards up until 2018. Some boards still ship with
a bootloader that uses that VID.

VID=0x2DAE PID=various
======================

The 0x2DAE vendor ID is owned by Hex, and is used for Hex flight
controllers from 2018 onwards.

Specific PIDs are:

 - 0x1101 CubeBlack+
 - 0x1001 CubeBlack bootloader
 - 0x1011 CubeBlack
 - 0x1016 CubeOrange (composite)
 - 0x1005 CubePurple bootloader
 - 0x1015 CubePurple
 - 0x1002 CubeYellow bootloader
 - 0x1012 CubeYellow (composite)

VID=0x3162 PID=various
======================

The 0x3612 vendor ID is owned by Holybro, and is used for Holybro
flight controllers from 2019 onwards.

Specific PIDs are:

 - 0x004B Durandal (composite)

VID=0x27AC PID=various
======================

The 0x27AC vendor ID is used by Laser Navigation for their line of
VRBrain flight controllers.

Specific PIDs are:

 - 0x1151 VRBrain-v51
 - 0x1152 VRBrain-v52
 - 0x1154 VRBrain-v54
 - 0x1910 VRCore-v10
 - 0x1351 VRUBrain-v51
