.. _common-pixracer-serial-names:

=======================
PixRacer Serial Mapping
=======================

Many different names are given to the serial hardware on PixRacer
where ArduPilot is concerned.  This document attempts to supply a
mapping between them.

+-----------+-------------+------------+--------------+-----------------+----------------+
| DataSheet | ParamPrefix | HW Label   | Def Role     | SERIAL_CONTROL* | Notes          |
+===========+=============+============+==============+=================+================+
|           | \SERIAL0_   | USB        | Console      |                 | Micro-USB Plug |
+-----------+-------------+------------+--------------+-----------------+----------------+
| UART4     | \SERIAL3_   | GPS        | GPS          | ?               |                |
+-----------+-------------+------------+--------------+-----------------+----------------+
| USART2    | \SERIAL1_   | TELEM1     | MAVLink      | ?               |                |
+-----------+-------------+------------+--------------+-----------------+----------------+
| USART3    | \SERIAL2_   | TELEM2     | MAVLink      | ?               |                |
+-----------+-------------+------------+--------------+-----------------+----------------+
| UART7     | \SERIAL6    | DEBUG      | Debug        | 10              |                |
+-----------+-------------+------------+--------------+-----------------+----------------+
| USART8    | \SERIAL4    | FRS        | GPS+         |                 | inverted       |
+-----------+-------------+------------+--------------+-----------------+----------------+
| USART1    | \SERIAL5    | ESP        | ESP8266      |                 | bl serial      |
+-----------+-------------+------------+--------------+-----------------+----------------+

.. note:

   ** SERIAL_CONTROL is used to talk from a GS directly to a serially-attached device.
   ++ While the default here is GPS, the port is inverted so this won't work easily.  Usually SERIAL4_PROTOCOL should be set to an FrSky protocol.

   On MAVProxy one can use ``module load nsh`` followied by ``nsh port 0`` to talk to a Hayes modem connected to TELEM1
