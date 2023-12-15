==============
REST Publisher
==============

.. code:: bash

    module load restserver

Publishes MAVLink messages to a REST server.

This module requires the ``flask`` and ``Werkzeug`` Python packages to be installed.

The IP and address of the server can be set by (for example) ``restserver address 127.0.0.1:4777``.
The default address is ``localhost:5000``.

The server can be started and stopped via the ``restserver start`` and ``restserver stop``
respectively.

When the server is running, messages will be available from the
endpoint ``http://localhost:5000/rest/mavlink`` (using the default IP and port) in
JSON format.