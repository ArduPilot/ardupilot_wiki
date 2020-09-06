=================
Terrain Reporting
=================

.. code:: bash

    module load terrain
    
The terrain module passes downloads and manages DTEM (Digital Terrain
Elevation Model) datasets and adds terrain height functionality to the
map module. It also allows the APM to download terrain data of the
surrounding area so that it has an approximate idea of its height above
ground.

By default, it uses the freely available `SRTM
dataset <https://en.wikipedia.org/wiki/Shuttle_Radar_Topography_Mission>`_.
For the purposes of this module, a copy of this dataset is hosted
`here <https://firmware.ardupilot.org/SRTM/>`_ Level 1
resolution (90m) is used.

Whilst this module does require an Internet connection to download the
nearby SRTM tiles, it will cache them on the local hard drive for
offline use.

Once loaded with ``module load terrain``, the module will automatically
download, cache and send small SRTM tiles to the APM's internal memory
(the SD card on the Pixhawk, for example).

The status of the module can be shown by entering ``terrain status``.

To get the terrain height at a particular location, use 
``terrain check <lat> <lon>```


