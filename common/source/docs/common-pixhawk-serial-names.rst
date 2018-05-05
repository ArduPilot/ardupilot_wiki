.. _common-pixhawk-serial-mapping:

==========================================
Pixhawk FMUv2 / Cube FMU v3 Serial Mapping
==========================================

Many different names are given to the serial hardware on Pixhawk and
Cube where ArduPilot is concerned.  This document attempts to supply a
mapping between them.

+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+
| DataSheet | NuttX Userland | PX4_HAL | ParamPrefix | HW Label   | Cube Full CB | Def Role     | SERIAL_CONTROL* | Notes          |
+===========+================+=========+=============+============+==============+==============+=================+================+
|           | /dev/ttyACM0   | UARTA   | \SERIAL0_   |            | Console      | Console      |                 | Micro-USB Plug |
+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+
| UART4     | /dev/ttyS3     | UARTB   | \SERIAL3_   | GPS        | GPS1         | GPS          | 2               |                |
+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+
| USART2    | /dev/ttyS1     | UARTC   | \SERIAL1_   | TELEM1     | TELEM1       | MAVLink      | 0               | bl serial      |
+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+
| USART3    | /dev/ttyS2     | UARTD   | \SERIAL2_   | TELEM2     | TELEM2       | MAVLink      | 1               |                |
+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+
| UART8     | /dev/ttyS6     | UARTE   | \SERIAL4_   | SERIAL4    | GPS2         | GPS          | 3               |                |
+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+
| UART7     | /dev/ttyS5     |         |             | SERIAL5    | CONS         | NSH Console  | 10              |                |
+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+
| USART1    | /dev/ttyS0     |         |             |            |              | PX4IO Debug  |                 |                |
+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+
| USART6    | /dev/ttyS4     |         |             |            |              | PX4IO Serial |                 |                |
+-----------+----------------+---------+-------------+------------+--------------+--------------+-----------------+----------------+

.. note:

   SERIAL_CONTROL is used to talk from a GS directly to a serially-attached device.

   On MAVProxy one can use ``module load nsh`` followied by ``nsh port 0`` to talk to a Hayes modem connected to TELEM1
