=========
GPS Input
=========

.. code:: bash

    module load GPSInput
    
The GPSInput module allows for GPS_INPUT message support. It will listen on a local udp port for messages and pass them on to the connected flight controller.

GPSInput.port
=============

Changes the UDP port that is being listened on:

.. code:: bash

    GPSInput.port <portnumber>

