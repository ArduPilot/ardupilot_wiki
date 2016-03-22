.. _learning-ardupilot-vehicle-code:

=================================
Learning ArduPilot - Vehicle Code
=================================

Now that you understand the ArduPilot libraries and the way tasks work
in ArduPilot it is time to start exploring a particular vehicle type.
There are currently 4 vehicles in ArduPilot:

-  Copter - for multicopters and helitopters
-  Plane - for fixed wing aircraft
-  APMrover2 - for ground vehicles and boats
-  AntennaTracker - for antenna trackers

While there are a lot of common elements between different vehicle
types, they are each different. For now we only have a detailed
description of the code structure for the Copter code. You should go and
read the :ref:`Copter Code Overview <apmcopter-code-overview>` wiki page now.

As you explore Copter you should also learn the following:

-  how to :ref:`run the ArduPilot SITL system <simulation-2>` to
   simulate a vehicle and autopilot
-  how to :ref:`use gdb to debug <debugging-with-gdb>` your board or SITL
-  `gdb cheatsheet <https://pixhawk.org/dev/gdb_cheatsheet>`__ 
-  how to :ref:`add parameters <code-overview-adding-a-new-parameter>` to vehicle code or libraries
