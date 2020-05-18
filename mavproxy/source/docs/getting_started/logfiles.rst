========
Logfiles
========

When connected to a vehicle, MAVProxy will automatically save any recieved telemetry to a logfile.

This logfile can be used for post-flight analysis or flight replay.

By default, the logfile will be saved as ``mav.tlog`` and ``mav.tlog.raw`` in the same folder that MAVProxy was executed from. For example:

.. code:: bash

    ~/Documents/code/MAVProxy$ mavproxy.py
    
Will log the telemetry to ``mav.tlog`` and ``mav.tlog.raw`` in the ``~/Documents/code/MAVProxy`` folder.

In the example:

.. code:: bash

    C:\users\stephen\Documents\logs> mavproxy.py
    
Will log the telemetry to ``mav.tlog`` and ``mav.tlog.raw`` in the ``C:\users\stephen\Documents\logs`` folder.

Note that both the ``mav.tlog`` and the ``mav.tlog.raw`` files contain the same data (albiet in different formats). In most cases however the raw file is not used by the analysis or replay tools.

MAVProxy can use the ``--aircraft=name`` (see :ref:`here <mavproxy-starting>`) argument to place the logfile in a specific name and date folder. This is useful for keeping logfiles sorted. See starting for further details.

For example: 

.. code:: bash

    C:\users\stephen\Documents> mavproxy.py --aircraft=copter
    
Will log the telemetry to ``flight.tlog`` and ``flight.tlog.raw`` in the ``C:\users\stephen\Documents\Logs\Copter\<date>\flight<n>\`` folder.

The ``--mission=name`` (see :ref:`here <mavproxy-starting>`) startup argument can be used with the ``--aircraft=name`` argument to use a specific name for the flight, rather than <date>. For example:

 .. code:: bash

    C:\users\stephen\Documents> mavproxy.py --aircraft=copter --mission=flyby
    
Will log the telemetry to ``flight.tlog`` and ``flight.tlog.raw`` in the ``C:\users\stephen\Documents\Logs\Copter\flyby\flight<n>\`` folder.

The ``--state-basedir=dir`` argument can be used to specify the base directory of where the telemetry files will be stored.

 .. code:: bash

    C:\users\stephen\Documents> mavproxy.py --aircraft=copter --mission=flyby --state-basedir=C:\logs
    
Will store the log files in ``C:\logs\copter\flyby\flight<n>\`` folder.
    
The MAVExplorer program can be used to view graphs and track of the flight data stored in these files.

   
    
