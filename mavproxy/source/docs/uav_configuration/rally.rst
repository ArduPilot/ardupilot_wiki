============
Rally Points
============

Rally points are a list of home points for the APM. On entering a
failsafe mode (and the parameters are set to return home) the APM will
fly the UAV to the nearest rally point. Note that a maximum of 5 rally
points are supported per APM.

This is useful during long range flights, where it would be safer for
the UAV to return to a closer home point rather than flying all the way
back to the original home point.

A rally menu is available on the GUI console.

rally list
==========

View the currently loaded rally points on the APM.

rally load
==========

Load a new set of rally points from file.

.. code:: bash

    rally load filename.txt

rally save
==========

Save the set of current rally points to file.

.. code:: bash

    rally save filename.txt

rally clear
===========

Delete the rally points from the APM.

rally move
==========

Move a specific rally point to a new location. When entered, a new
location can be selected on the map window. This requires the map module
to be loaded.

.. code:: bash

    rally move 6

rally add
=========

Add a new rally point. When entered, a location can be selected on the
map window. This requires the map module to be loaded.

rally remove
============

Remove a specific rally point.

.. code:: bash

    rally remove 6

rally land
==========

Command the APM to land from a rally point. See `this DIYDrones
article <http://diydrones.com/profiles/blogs/landing-from-rally-points>`_
for full details.

.. code:: bash

    rally land

