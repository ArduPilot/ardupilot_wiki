==================
Obstacle Generator
==================

.. code:: bash

    module load genobstacles
    module load asterix

Allows the user to send simulated obstacles to ArduPilot, in order to test object avoidance
techniques. This module was originally developed for the 2018 UAV Outback Challenge.

When loaded, the user can right-click on the MAVProxy map to drop obstacles. These include birds, planes
and clouds. These are sent to 127.0.0.1:45454 in ASTERIX format.

The ``asterix`` module will recieve these obstacles from 127.0.0.1:45454 and convert to a ``ADSB_VEHICLE``
MAVLink message and then on-send to ArduPilot.

If ADSB avoidance is enabled, ArduPilot will then attempt to avoid the object.
