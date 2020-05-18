============
Flight Modes
============

Flight modes can be directly entered on the command line. This will send
the appropriate command to the APM. Use ``mode n`` to change mode, where
n is the desired mode. To get a list of available modes, enter ``mode``.

mode auto
=========

Enter AUTO mode, starting with the first waypoint.

mode loiter
===========

Enter LOITER mode around the UAV's current position.

mode rtl
========

Enter Return to Launch mode.

mode manual
===========

Enter manual mode. On Arducopter this is equivalent to STABILIZE mode

mode fbwa
=========

Enter Fly By Wire A mode. Applicable on fixed wing APM's only.

land
====

Enter LAND mode. Note this is only applicable on fixed wing APM's.

A specific mode is guided. By entering ``guided alt`` (where alt is the
desired altitude), the user can the select a point on the map window and
the UAV will immediately begin flying towards that point.

guided
======

Fly to the last clicked point on the map, or specified longitude and 
latitude. Requires an altitude:
``guided ALTITUDE`` or ``guided LAT LON ALTITUDE``

