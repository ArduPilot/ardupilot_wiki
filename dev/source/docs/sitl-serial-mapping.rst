.. _sitl-serial-mapping:



=============================
Archived: SITL Serial Mapping
=============================

.. warning:: This topic is archived.

This document attempts to supply information about the virtual serial ports present in ArduPilot's SITL.

+-------------+------------+--------------+-------------------------+
|             |Sim_vehicle |              |                         |
| ParamPrefix | Cmd Line   | Def Role     | Default Connection      |
+=============+============+==============+=========================+
| \SERIAL0_   | \- -uartA= | Console      | tcp:localhost:5760:wait |
+-------------+------------+--------------+-------------------------+
| \SERIAL1_   | \- -uartC= | MAVLink      | tcp:localhost:5762      |
+-------------+------------+--------------+-------------------------+
| \SERIAL2_   | \- -uartD= | MAVLink      | tcp:localhost:5763      |
+-------------+------------+--------------+-------------------------+
| \SERIAL3_   | \- -uartB= | GPS          | Simulated GPS           |
+-------------+------------+--------------+-------------------------+
| \SERIAL4_   | \- -uartE= | GPS          | Simulated GPS           |
+-------------+------------+--------------+-------------------------+
| \SERIAL5_   | \- -uartF= |              |                         |
+-------------+------------+--------------+-------------------------+
| \SERIAL6_   | \- -uartG= |              |                         |
+-------------+------------+--------------+-------------------------+
| \SERIAL7_   | \- -uartH= |              |                         |
+-------------+------------+--------------+-------------------------+
