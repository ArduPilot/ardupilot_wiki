===============
Link Management
===============

The link module allows the user to add, remove and monitor the status of
communications links to the APM. This can be done at any time.

It is useful for UAV systems that have multiple communications links.
MAVProxy will automatically read the data from both links, no user
intervention is necessary for link failover.

A link menu is available on the GUI console.

For ease of management, links can have labels.

link add
========

Add an additional communications link. This can be either a network
address or serial port. For a serial port, the default baud rate is
57600. Use ``set baudrate xxx`` to change baud rate for any subsequent
link add's.

To add a label to a link, use ``:{"label":"LinkName"}`` after the link
details. Note the link label must not have any spaces in it.

.. code:: bash

    link add 127.0.0.1:14550
    link add tcp:127.0.0.1:14550
    link add tcp:127.0.0.1:14550:{"label":"3GMobile"}
    set baudrate 115200
    link add COM17
    
See the startup :ref:`section <mavproxy-quickstart>`  for full details on link types

link remove
===========

Remove a communications link. The ID or label of the link is used to identify the
link to remove.

.. code:: bash

    link remove 2
    link remove 3GMobile

link list
=========

List the active communications links, their ID's and labels (if applicable).

.. code:: bash

    link list


