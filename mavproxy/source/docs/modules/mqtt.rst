==============
MQTT Publisher
==============

.. code:: bash

    module load mqtt

Publishes MAVLink messages to a MQTT server.

This module requires the ``paho-mqtt`` Python package to be installed.

The module has the following settings, which via be set via ``mqtt set``.

==================   ===============================================  ===============================
Setting              Description                                      Default
==================   ===============================================  ===============================
ip                   IP address of the MQTT server                    127.0.0.1
port                 Port of the MQTT server                          1883
name                 Publisher name                                   MAVProxy
prefix               Any prefix to add to the published messages      ''
==================   ===============================================  ===============================
