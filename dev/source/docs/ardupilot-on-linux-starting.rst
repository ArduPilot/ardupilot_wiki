.. _ardupilot-on-linux-starting:

===============
Startup Options
===============

Run a build with -\-help to see the latest command-line options.

-\-serial#
==========

This command line argument, with # replaced by a number, is used to connect the an ArduPilot serial port to a serial device or network protocol.
May also use old-style(-A, -B, ...).

For each ArduPilot serial port, specify either a device file or a short string to configured the network protocol below.
The user account running ardupilot should be added to the dialout group to access serial ports.

Serial Ports: /dev/deviceFile
	- -\-serial0 /dev/ttyAMA0
	- -\-serial1 /dev/ttyS0
TCP networking: tcp:<IP address>:<PortNumber>
	- -\-serial2 tcp:11.0.0.2:5678
	- -\-serial3 tcp:192.168.2.15:1243:wait
	- -\-serial4 udp:11.0.0.2:14550
UDP networking: udp:<IP address>:<PortNumber>
	- -\-serial5 udp:11.0.0.255:14550:bcast
	- -\-serial6 udpin:0.0.0.0:14550

Additional UDP/TCP options:
	- wait: wait for connection(TCP)
	- bcast: use broadcast address(UDP)

-\-log-directory
================
Directory used for logging. May also use -l.

Default is the per-board HAL_BOARD_LOG_DIRECTORY.

-\-terrain-directory
====================
Directory used for terrain. May also use -t.

Default is the per-board HAL_BOARD_TERRAIN_DIRECTORY.

-\-storage-directory
====================
Directory used for storage. May also use -s.

Default is the per-board HAL_BOARD_STORAGE_DIRECTORY.

-\-module-directory
===================
Directory used for modules. May also use -M.

Default is the per-board AP_MODULE_DEFAULT_DIRECTORY.

-\-cpu-affinity
===============
CPUs to bind ArduPilot. May also use -c. May specify a single cpu(1), multiple cpus(1,3), or a range(1-3)
