.. _guided-mode:

===========
GUIDED Mode
===========

GUIDED mode is used when you want the aircraft to fly to a specific
point on the map, or in specific direction (heading) without setting up a mission. 

The other major use for GUIDED mode is in :ref:`geo-fencing <geofencing>`.
When the geo-fence is breached the aircraft will enter GUIDED mode, and
head to the predefined geo-fence return point, where it will loiter
until the operator takes over.

Location sub-mode
-----------------
Most ground control
stations support a "click to fly to" feature where you can click a point
on the map and the aircraft will fly to that location then loiter. If an 
altitude is requested, the aircraft will try to achieve the target altitude as it flies,
and will loiter down or up to the requested altitude once it arrives, if required. 

.. image:: ../images/guided-altitude.png

This sub-mode can also be requested via MAVLink using the DO_REPOSITION command.

If the vehicle is flying in Guided "Location sub-mode" then GUIDED_CHANGE_ALTITUDE and GUIDED_CHANGE_SPEED 
commands will be ignored.

Heading sub-mode
----------------
The guided mode heading feature isn't supported by Mission Planner or QGroundControl, but can be
requested via MAVLink commands or using Lua. If the aircraft receives a GUIDED_CHANGE_HEADING
command it will fly indefinitely towards the heading in degrees. 

The target altitude and desired airspeed or
groundspeed of the vehicle to fly at can be set using the GUIDED_CHANGE_ALTITUDE and GUIDED_CHANGE_SPEED
MAVLink commands. If the aircraft is flying in Guided "Heading sub-mode" then it will try to
achieve the altitude as it flies, as there is no "end point" to a heading target for a final loiter.

For more information about this, see the MAVLink GUIDED_CHANGE_HEADING documentation 
https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_HEADING


