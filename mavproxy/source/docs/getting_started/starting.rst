.. _mavproxy-starting:

===============
Startup Options
===============

There are no mandatory command line options for MAVProxy. An exception
to this is the ``--master`` option if there is more than 1 APM connected
to the system.

Note that MAVProxy can only connect to 1 vehicle at a time. If control
of multiple vehicles is desired, multiple MAVProxy sessions should be
used - one session for each vehicle.

-\\-master
==========

Specifies which port (serial, USB or network address/port) the UAV is
communicating on.

Multiple ``--master`` can be used. MAVProxy will sort the packets into a
single stream automatically. This is useful if independent redundant
links are being used.

If an IP address is specified, it must be the local computer's IP
address, or loopback address. The IP connection type (TCP or UDP) must be prefixed to the
IP address.

If a serial port is specified, an optional comma-separated baud rate may
also be specified. If present, this overrides the rate given by
``--baudrate``.

.. code:: bash

    mavproxy.py --master=/dev/ttyUSB0
    mavproxy.py --master="com14"
    mavproxy.py --master=tcp:192.168.1.1:14550
    mavproxy.py --master=udp:192.168.1.1:14550 --master=/dev/ttyUSB0
    mavproxy.py --master=/dev/ttyUSB0,57600
    mavproxy.py --master=udp:127.0.0.1:14550
    mavproxy.py --master=tcp:0.0.0.0:14550

If connecting to a remote IP address, the udpout or tcpout arguments should be used:

.. code:: bash

    mavproxy.py --master=udpout:10.10.1.1:14550
    mavproxy.py --master=tcpout:10.10.1.1:14550

-\\-quadcopter
==============

Use quadcopter controls.

-\\-baudrate
============

Specify baudrate of ``--master`` and ``--out`` ports. Only applicable
for serial links.

-\\-out
=======

Forward the MAVLink packets to a remote device (serial, USB or network
address/port). Useful if using multiple ground station computers or
relaying the stream through an intermediate node.

MAVlink usually uses port 14550 for IP-based packet forwarding, though
other ports can be used if needed.

If a serial port is given (for example a radio device connecting an
airbourne MAVProxy instance with the GCS), a link-specific baud rate can
be given after the device, delimited by a comma. This overrides the
global default baudrate specified by ``--baudrate``.

If forwarding to a network address via tcp or udp, this must be prefixed
before the IP address.

A UDP broadcast can be made via the udpbcast:<ip>:<port> that starts as 
UDP broadcast then locks onto the first client that connects. <ip> specifies 
the valid client range. For example, udpbcast:192.168.2.255:14550 will connect to 
the first client in the 192.168.2.0 to 192.168.2.254 range on port 14550.

.. code:: bash

    mavproxy.py --master=/dev/ttyUSB0 --out=udp:192.168.1.1:14550
    mavproxy.py --master=/dev/ttyACM0,115200 --out=/dev/ttyUSB0,57600
    mavproxy.py --master=/dev/ttyACM0,115200 --out=COM17,57600
    mavproxy.py --master=/dev/ttyACM0,57600 --out=udpbcast:192.168.2.255:14550 


-\\-sitl
========

Input from APM Software in the loop (SITL) simulator. Usually ``--sitl=127.0.0.1:5501``

-\\-streamrate
==============

MAVLink stream rate.

-\\-source-system
=================

MAVLink source system for this GCS.

-\\-source-component
====================

MAVLink source component for this GCS.

-\\-target-system
=================

MAVLink target master system.

-\\-target-component
====================

MAVLink target master component.

-\\-logfile
===========

Name of MAVLink master logfile. Default is mav.tlog.

-\\-append-log
==============

Append to latest log file, rather than creating a new one. Useful if
re-connecting mid-flight.

-\\-setup
=========

Startup in the APM's command line interface (CLI) mode.

-\\-nodtr
=========

Disable DTR drop on close.

-\\-show-errors
===============

Show MAVLink error packets.

-\\-speech
==========

Use text-to-speech. Requires the speechd software on Linux.

-\\-aircraft
============

Name of the aircraft being flown. If used, logfiles will be stored in
**/Logs/AircraftName/Date/flightNumber/flight.tlog**. Useful for keeping
flight logs organised.

-\\-cmd
=======

Initial commands to run in MAVProxy. Delimited by ``;``

.. code:: bash

    mavproxy.py --master=/dev/ttyUSB0 --cmd="param load init.parm; module load map;"

-\\-console
===========

Load the GUI console module on startup.

-\\-map
=======

Load the moving map module on startup.

-\\-load-module
===============

Load the specified module on startup. Can be used multiple times, or
with a comma separated list.

-\\-mavversion
==============

Specify MAVLink version. Can be 1.0 or 2.0. Otherwise MAVProxy will autodetect the MAVLink version

-\\-auto-protocol
=================

Auto detect MAVLink protocol version.

-\\-continue
============

Continue logs.

-\\-nowait
==========

Don't wait for HEARTBEAT packets on startup.

-\\-dialect
===========

MAVLink dialect. Uses the APM dialect by default.

-\\-rtscts
==========

Use RTS/CTS hardware flow control.

-\\-mission
===========

Give the current mission a name. If used, the flight log will be stored
as :file:`/Logs/aircraftname/missionname` rather than the default
:file:`/Logs/aircraftname/currentdatetime`.

-\\-daemon
==========

Run in daemon mode (as a background process). No interactive shell will
be started.

-\\-state-basedir
=================

The base directory will logs are stored, if it is not the current
directory.

-\\-version
===========

Return version information about MAVProxy.

-\\-moddebug
============

Controls the level of debugging output displayed on the console. Default is 0
(no debug output). A value of 3 is useful for debugging crashes or errors in
MAVProxy and its modules.

-\\-default-modules
===================

A comma separated list of the modules to load on startup by default. The default 
value of this parameter is ``log,signing,wp,rally,fence,param,relay,tuneopt,arm,mode,calibration,rc,auxopt,misc,cmdlong,battery,terrain,output,adsb``

-\\-non-interactive
===================

Do not start interactive shell
