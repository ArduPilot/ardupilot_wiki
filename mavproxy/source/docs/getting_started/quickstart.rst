.. _mavproxy-quickstart:

==========
Quickstart
==========

In its simplest form, MAVProxy only needs the address of the USB port
or network address to connect to.

Over USB
========

If there is only 1 autopilot connected, the ``--master`` is not required.
MAVProxy will autodetect the correct port.

Linux:
------

.. code:: bash

    mavproxy.py --master=/dev/ttyUSB0

Windows:
--------

From a command line window:

.. code:: bash

    mavproxy --master=COM14

You can create a simple double-clickable batch file to allow a quick start and contend with multiple autopilot ports being present. Create a .bat file on the desktop containing:

.. code::

    @echo off
    set /p port=Which COM Port?:
    if not exist Mavproxy_Data mkdir "Mavproxy_Data"
    cd Mavproxy_Data
    mavproxy.exe --master=COM%port% --console --map --load-module graph

.. note:: this also creates a data directory for the parameter and log files that are saved by MAVProxy to avoid cluttering the desktop. You can also modify the last line to load any additional modules that you wish, like joystick, simply by adding more "--load-module xxx" to the command.

macOS:
------

.. code:: bash

    mavproxy.py --master=/dev/ttyusbserialxxx

----------------------------------------------------------------------------

Normally MAVProxy will auto-detect the correct baudrate. If required,
the baud rate can instead be manually specified.

Linux:

.. code:: bash

    mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600

Windows:

.. code:: bash

    mavproxy --master=COM14 --baudrate=57600

macOS:

.. code:: bash

    mavproxy.py --master=/dev/tty.usbserialxxxxx --baudrate=57600


Over Network
============

Specify the IP address and port containing a mavlink stream. The address
to connect to must be your own IP address or loopback address. Alternatively,
if the local IP address is not known, use 0.0.0.0 for MAVProxy find and use
the local IP address. The type of stream (tcp or udp) should also be specified.

.. code:: bash

    mavproxy.py --master=tcp:192.168.1.1:14550
    mavproxy.py --master=udp:127.0.0.1:14550
    mavproxy.py --master=tcp:0.0.0.0:14550
    
If connecting to a remote IP address, the udpout or tcpout arguments should be used:

.. code:: bash

    mavproxy.py --master=udpout:10.10.1.1:14550
    mavproxy.py --master=tcpout:10.10.1.1:14550
    

.. note::

   MavProxy can output a mavlink stream to remote network addresses using
   UDP Broadcast.
