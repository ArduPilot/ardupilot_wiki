========
PPP Link
========

.. code:: bash

    module load ppp
    
The pp module provides a generic data transfer channel within the Mavlink stream,
which can be used by the pppd program.

Starting and Stopping
=====================

.. code:: bash

    ppp start
    
Starts the ppp link and a ppp daemon (pppd).

.. code:: bash

    ppp stop
    
Stops the ppp link and daemon.

.. code:: bash

    ppp status
    
Prints the current status of the ppp link to the console.
