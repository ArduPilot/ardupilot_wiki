=================
Terrain Reporting
=================

.. code:: bash

    module load terrain
    
The terrain module passes downloads and manages DTEM (Digital Terrain
Elevation Model) datasets and adds terrain height functionality to the
map module. It also allows the autopilot to download terrain data of the
surrounding area so that it has an approximate idea of its height above
ground.

By default, it uses the freely available `SRTM
dataset <https://en.wikipedia.org/wiki/Shuttle_Radar_Topography_Mission>`_.
For the purposes of this module, a copy of this dataset is hosted
`here <https://terrain.ardupilot.org/SRTM3/>`_ Level 3
resolution (90m) is used.

Whilst this module does require an Internet connection to download the
nearby SRTM tiles, it will cache them on the local hard drive for
offline use.

Once loaded with ``module load terrain``, the module will automatically
download, cache and send small SRTM tiles to the autopilot's internal memory
(the SD card on the autopilot, for example).

The status of the module can be shown by entering ``terrain status``.

To get the terrain height at a particular location, use 
``terrain check <lat> <lon>``` or right click on the map (module must be loaded to display)

To use higher resolution SRTM data, a Level 1 dataset is `available <https://terrain.ardupilot.org/SRTM1/>`_.
Use ``terrain set source SRTM1`` to use. To go back to the Level 3 dataset, use ``terrain set source SRTM3``

In addition, you can prevent the downloading of tiles from the Internet for testing purposes, using the following command to disable it:

.. code:: bash

    terrain set offline 1



