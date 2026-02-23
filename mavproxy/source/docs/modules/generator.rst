==============
Generator
==============

.. code:: bash

    module load generator

Shows generator information on the MAVProxy graphical console.

If an on-board electrical generator is sending ``GENERATOR_STATUS`` MAVLink messages,
these will be displayed on the console. This includes fields such as RPM, current, voltage,
runtime and diagnostic flags.
