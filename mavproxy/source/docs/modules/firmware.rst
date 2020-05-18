===================
Firmware Management
===================

.. code:: bash

    module load firmware
    
The firmware module provides downloading and listing of available APM firmware. It 
does not currently handle firmware flashing

List
====

Download the list of available firmware:

.. code:: bash

    fw manifest download
    
Prints out the available latest firmwares:

.. code:: bash

    fw list FILTERS
    
Use ``FILTERS`` to filter the columns. An example would be ``platform=PX4-v2`` to 
only list the firmware for the PX4-V2 platform. Another example would be 
``releasetype=OFFICIAL frame=quad platform=PX4-v2`` to only list the most recent official firmware
for quadcopters for the PX4-v2 platform.

Download
========

Download the specified firmware. It uses the same ``FILTERS`` as fw list.

.. code:: bash

    fw download FILTERS
    
   
