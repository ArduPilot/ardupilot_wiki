.. _sitl-serial-mapping:



=============================
Archived: SITL Serial Mapping
=============================

.. warning:: This topic is archived. The information has been moved to :ref:`learning-ardupilot-uarts-and-the-console`.

This document attempts to supply information about the virtual serial ports present in ArduPilot's SITL.

+-------------+----------------------+----------+-------------------------+
| ParamPrefix | Sim_vehicle Cmd Line | Def Role | Default Connection      |
+=============+======================+==========+=========================+
| \SERIAL0_   | \- -serial0=         | Console  | tcp:localhost:5760:wait |
+-------------+----------------------+----------+-------------------------+
| \SERIAL1_   | \- -serial1=         | MAVLink  | tcp:localhost:5762      |
+-------------+----------------------+----------+-------------------------+
| \SERIAL2_   | \- -serial2=         | MAVLink  | tcp:localhost:5763      |
+-------------+----------------------+----------+-------------------------+
| \SERIAL3_   | \- -serial3=         | GPS      | Simulated GPS           |
+-------------+----------------------+----------+-------------------------+
| \SERIAL4_   | \- -serial4=         | GPS      | Simulated GPS           |
+-------------+----------------------+----------+-------------------------+
| \SERIAL5_   | \- -serial5=         |          |                         |
+-------------+----------------------+----------+-------------------------+
| \SERIAL6_   | \- -serial6=         |          |                         |
+-------------+----------------------+----------+-------------------------+
| \SERIAL7_   | \- -serial7=         |          |                         |
+-------------+----------------------+----------+-------------------------+
