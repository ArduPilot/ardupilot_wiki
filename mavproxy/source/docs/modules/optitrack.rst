=================
Optitrack Support
=================

.. code:: bash

    module load optitrack

Connects to an `Optitrack motion capture server <https://optitrack.com/software/motive/>`__
and passes the vehicle position and orientation messages to ArduPilot via the
``ATT_POS_MOCAP`` MAVLink message.

It is assumed that the server is using the default ports of 1510 and 1511.

Use ``optitrack start`` to start sending position data.

==================   ===============================================  ===============================
Setting              Description                                      Default
==================   ===============================================  ===============================
server               IP address of server                             127.0.0.1
client               IP address of client                             127.0.0.1
msg_intvl_ms         How often to send position updates (ms)          75
obj_id               Mocap object ID                                  1
==================   ===============================================  ===============================