==========================
uBlox GPS Managment Module
==========================

.. code:: bash

    module load ublox
    
The ublox module allows the user to upload GNSS databases (almanacs) from the uBlox website to a uBlox GPS module.
This is useful for getting a quicker position lock under poor signal conditions. See
`here <https://www.u-blox.com/en/assistnow>`_ for details about this database.



To use the uBlox API you will need to create a file containing your API token in ``~/.mavproxy/ublox/api_token``.

ublox reset
===========

This command reboots the GPS module. Also known as a "cold start"

ublox status
============

Returns the current status of the GPS

ublox mga
=========

Upload the mga database from to the GPS module. It is assumed that ``offline.ubx`` and ``dbd.ubx`` are
in the ``~/.mavproxy/ublox`` folder.

