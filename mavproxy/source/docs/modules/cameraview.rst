=============
Camera Viewer
=============

.. code:: bash

    module load cameraview

Adds a layer to the map showing the area in view of a belly-mounted
camera, accounting for the current roll and pitch of the UAV. It also
provides the data for geotagging captured images.

This module requires the cuav library (available via pypi) for
integration with the camera hardware:

.. code:: bash

    pip install cuav

Also see the cuav documentation `here <http://canberrauav.github.io/cuav/build/html/index.html>`_

