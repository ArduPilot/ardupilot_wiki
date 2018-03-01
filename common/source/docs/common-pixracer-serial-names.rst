.. _common-pixracer-serial-names:

=======================
PixRacer Serial Mapping
=======================

Many different names are given to the serial hardware on PixRacer
where ArduPilot is concerned.  This document attempts to supply a
mapping between them.

+-----------+-------------+------------+-----------------+---------------------+
| DataSheet | Param Prefix| HW Label   | Default Protocol| Notes               |
+===========+=============+============+=================+=====================+
|           |  SERIAL0    | USB        | Console         | Micro-USB Plug      |
+-----------+-------------+------------+-----------------+---------------------+
| USART2    |  SERIAL1    | TELEM1     | MAVLink         | bootloader serial## |
+-----------+-------------+------------+-----------------+---------------------+
| USART3    |  SERIAL2    | TELEM2     | MAVLink         |                     |
+-----------+-------------+------------+-----------------+---------------------+
| UART4     |  SERIAL3    | GPS        | GPS             |                     |
+-----------+-------------+------------+-----------------+---------------------+
| USART8    |  SERIAL4    | FRSky      | GPS2**          | normally inverted   |
+-----------+-------------+------------+-----------------+---------------------+
| USART1    |  SERIAL5    | ESP-01     | MAVLink         |                     |
+-----------+-------------+------------+-----------------+---------------------+
| UART7     |  SERIAL6    | DEBUG      | disabled        |                     |
+-----------+-------------+------------+-----------------+---------------------+

Notes:

   ** While the default here is GPS, the port is inverted so this won't work with a GPS, it was intended for use with FrSky receivers for telemetry.  Usually :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` should be set to an FrSky protocol and a diode tied from TX (anode) to RX and attached to the SPort of an FrSky receiver.Alternatively, :ref:`SERIAL4_OPTIONS<SERIAL4_OPTIONS>` can be set to 2, to disable the inverters for normal use.

   ## can be used for firmware upload instead of USB port, if needed.

