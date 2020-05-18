======================
Writing to Serial Port
======================

.. code:: bash

    module load serial
    
Allows transmission of data packets through the serial ports on the APM
via MAVProxy (ie. remote control of the serial ports). Use
``serial lock`` and ``serial unlock`` to give (or release) exclusive
control of the serial ports.

To change to serial port settings use ``serial set [setting] [value]``,
where setting can be port, baudrate or timeout. To send data, enter
``serial send [data]``.

Any recieved serial data will be output to the console.

