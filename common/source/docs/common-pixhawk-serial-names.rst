.. _common-pixhawk-serial-mapping:

============================
Pixhawk FMUv2 / Cube FMU v3 Serial Mapping
============================

Many different names are given to the serial hardware on Pixhawk and
Cube where ArduPilot is concerned.  This document attempts to supply a
mapping between them.

+-----------+----------------+---------+------------+--------------+-------------+----------------+
| DataSheet | NuttX Userland | PX4_HAL | HW Label   | Cube Full CB | Def Role    | Notes          |
+===========+================+=========+============+==============+=============+================+
|           | /dev/ttyACM0   | UARTA   |            | Console      | Console     | Micro-USB Plug |
+-----------+----------------+---------+------------+--------------+-------------+----------------+
| USART?    | /dev/ttyS3     | UARTB   | GPS        | GPS1         | GPS         |                |
+-----------+----------------+---------+------------+--------------+-------------+----------------+
| USART2    | /dev/ttyS1     | UARTC   | TELEM1     | TELEM1       | MAVLink     | bl serial      |
+-----------+----------------+---------+------------+--------------+-------------+----------------+
| USART?    | /dev/ttyS2     | UARTD   | TELEM2     | TELEM2       | MAVLink     |                |
+-----------+----------------+---------+------------+--------------+-------------+----------------+
| USART?    | /dev/ttyS6     | UARTE   | SERIAL4    | GPS2         | GPS         |                |
+-----------+----------------+---------+------------+--------------+-------------+----------------+
| USART?    | /dev/ttyS3     | UARTF   | SERIAL5    | CONS         | NSH Console |                |
+-----------+----------------+---------+------------+--------------+-------------+----------------+
