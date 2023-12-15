==============
NMEA GPS Input
==============

.. code:: bash

    module load NMEAGPS

Parses and displays the position of a NMEA GPS connected via COM port. The
position is *not* sent to ArduPilot.

This module requires the ``pynmea2`` Python package to be installed.

Use ``nmeagps connect`` to connect to the NMEA GPS module and ``nmeagps disconnect`` to
disconnect from the module.

To see the current position reported by the NMEA GPS module, use ``nmeagps status``.

The module has the following settings, which via be set via ``nmeagps set``.

==================   ===============================================  ===============================
Setting              Description                                      Default
==================   ===============================================  ===============================
port                 COM port                                         ''
baudrate             Baud rate                                        9600
==================   ===============================================  ===============================