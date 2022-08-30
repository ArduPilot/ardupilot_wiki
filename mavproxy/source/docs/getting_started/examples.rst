========
Examples
========

The following section provides a couple of common usage scenarios for MAVProxy.

Quadcopter over a standard (57600 kbps) radio link on a serial port. MAVProxy will auto-detect the correct serial port. The quadcopter is called "Quaddy". The GUI console, map and text-to-speech are also enabled:

.. code:: bash

    mavproxy.py --baud=57600 --aircraft="Quaddy" --speech --console --map --quadcopter
    
Run MAVProxy in daemon mode, taking in a radio link (COM17 at 9600 kbps) and outputting it to the local network. This allows a single instance of MAVProxy to distribute the telemetry stream to multiple clients:

.. code:: bash

    mavproxy.py --master=COM17,9600 --out=udpin:0.0.0.0:14550 --daemon
    
And for each client (where xxx.xxx.xxx.xxx is the daemon's IP address):

.. code:: bash
    
    mavproxy.py –master=udpout:xxx.xxx.xxx.xxx:14550
    
Use multiple radio links (one serial, one network) to the remote vehicle. This is useful for situations where a backup radio link is required:

.. code:: bash

    mavproxy.py --master=\dev\ttyUSB0,57600 --master=udpin:192.168.16.15:2626
    
    
