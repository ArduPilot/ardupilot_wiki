========
Geofence
========

The geofence represents a hard limit to where the UAV will not fly
beyond. It is useful for preventing runaway UAVs (barring other
software/hardware issues). If it breaches the fence the failsafe will be
activated, returning it to the home point.

It is represented as an arbitrary polygon of waypoints around the flying
area in the fixed-wing APM. In Arducopter, it is a radius and altitude
parameter(s) representing a cylinder around the flying area.

A fence menu is available on the GUI console.

fence list
==========

View the currently loaded geofence on the APM.

fence load
==========

Load a new set of geofence coordinates from file.

.. code:: bash

    fence load filename.txt

fence save
==========

Save the set of current geofence coordinates to file.

.. code:: bash

    fence save filename.txt

fence draw
==========

Draw the geofence on the map (if loaded).

fence enable
============

Enable the geofence failsafe.

fence disable
=============

Disable the geofence failsafe.

fence clear
===========

Deletes all geofence coordinates.

fence move
==========

Move a specific fence coordinate to a new location. When entered, a new
location can be selected on the map window. This requires the map module
to be loaded.

.. code:: bash

    fence move 6

fence remove
============

Remove a specific geofence coordinate.

.. code:: bash

    fence remove 6

