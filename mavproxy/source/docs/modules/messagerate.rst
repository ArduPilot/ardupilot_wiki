================
Set Message Rate
================

.. code:: bash

    module messagerate
    
The messagerate module allows the user to specify the rate of individual
MAVlink messages sent from the flight controller. This will override any
requested stream rates

Usage: ``messagerate <status | reset | set(msg)(rate) | get(msg)>``

- ``status``: list current message rates
- ``reset``: reset all message rates to defaults.
- ``set msg rate``: set the message ``msg`` to rate ``rate``
- ``get msg``: Get the message ``msg`` rate

For example, to set the HEARTBEAT MAVLink message rate to 10Hz:

.. code:: bash

    messagerate set HEARTBEAT 10