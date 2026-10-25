.. _simlation:

==========
Simulation
==========

:ref:`SITL<dev:sitl-simulator-software-in-the-loop>` has built-in physics models for the flapping fin Blimp and the four motor Blimp. In order to invoke it for simulation:

.. code:: bash

    sim_vehicle.py -v Blimp --console --map

To simulate the four motor Blimp instead, add ``-f blimp-motor``:

.. code:: bash

    sim_vehicle.py -v Blimp -f blimp-motor --console --map
