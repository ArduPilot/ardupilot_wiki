=============
Offline Tools
=============

There are a couple of offline tools within the MAVLink package that can
be used for a postflight analysis.

.. _mavproxy-mavgraph:

mavgraph.py
===========

This can graph any of the flight data.

.. code:: bash

    mavgraph.py flight.tlog VFR_HUD.alt

Other than specifying the tlog file, the commands are exactly the same
as the :ref:`graph <mavproxy-modulegraph>` module in MAVProxy.

mavtogpx.py
===========

Exports the GPS points in a logfile to the GPX format, which can be read
by Google Earth.

.. code:: bash

    mavtogpx.py flight.tlog

mavparms.py
===========

Finds the APM parameters in a logfile and displays them on the console.

.. code:: bash

    mavparms.py flight.tlog

mavflighttime.py
================

Prints out the total time spent in the air. Note that a threshold of
3m/s defines the difference between air time and ground time. This can
be changed by the ``--groundspeed=`` argument.

.. code:: bash

    mavflighttime.py flight.tlog

mavflightmodes.py
=================

Prints out the timestamp and name of each flight mode change in a
logfile.

.. code:: bash

    mavflightmodes.py flight.tlog

