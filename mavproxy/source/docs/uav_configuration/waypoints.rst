.. _mavproxy-waypoints:

=========
Waypoints
=========

MAVProxy includes a few commands for managing waypoints. These can be
entered via the command line or via the menu bar of the GUI console.

wp list
=======

View the currently loaded waypoints on the APM.

wp load
=======

Load a new set of waypoints from file.

.. code:: bash

    wp load filename.txt

wp save
=======

Save the set of current waypoints to file.

.. code:: bash

    wp save filename.txt

wp clear
========

Delete all waypoints from the APM. It is not recommended to do this
during an AUTO mission.

wp update
=========

Similar to ``wp load``, except only a single specified waypoint is
loaded.

.. code:: bash

    wp update filename.txt 6

wp move
=======

Move a specific waypoint to a new location. When entered, a new location
can be selected on the map window. If a DTED map is available, the AGL
height of the waypoint will be maintained. This requires the map module
to be loaded.

.. code:: bash

    wp move 6

wp loop
=======

Closes the loop on a mission, allowing the APM to repeat a mission.

wp remove
=========

Remove a specific waypoint from the mission.

.. code:: bash

    wp remove 6

wp set
======

The the specified waypoint as the current waypoint. The mission will be
run from this waypoint onwards. If the APM is running in AUTO mode, it
will go to this waypoint immediately.

.. code:: bash

    wp set 6

wp undo
=======

Reverts the last waypoint change or edit. Only the last change made via
the wp command can be undone.

.. code:: bash

    wp undo

Using a GUI
===========

The map module includes methods for viewing and editing waypoints on a
map.

