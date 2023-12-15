===============
NTRIP Injection
===============

.. code:: bash

    module load ntrip

Connects to an NTRIP (GPS RTK correction) server and passes the correction messages
to ArduPilot. This would allow a (compatible) GPS module to recieve RTCM corrections.

Use ``ntrip start`` to start sending correction data and ``ntrip stop`` to
stop sending correction data.

To see the current status, use ``ntrip status``.

The module has the following settings, which via be set via ``ntrip set``.

==================   ===============================================  ===============================
Setting              Description                                      Default
==================   ===============================================  ===============================
caster               URL of server (ie: ntrip.data.gnss.ga.gov.au)    ''
username             Username to access server                        IBS
password             Password to access server                        IBS
mountpoint           NTRIP mountpoint                                 ''
logfile              Filename to save NTRIP data to                   ''
sendalllinks         Send on all links                                False
sendmul              Send data multiple times                         1
==================   ===============================================  ===============================

An example of configuring the ntrip module is below:

.. code:: bash

    module load ntrip
    ntrip set caster caster.centipede.fr
    ntrip set port 2101
    ntrip set mountpoint TCY22
    ntrip set username centipede
    ntrip set password centipede
    ntrip start
