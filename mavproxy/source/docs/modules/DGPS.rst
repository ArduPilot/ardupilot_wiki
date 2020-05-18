================
Differential GPS
================

.. code:: bash

    module load DGPS
    
Provides a method to supply DGPS data up to the UAV, in order to
provide greater positional accuracy. The ground station
segment of the DGPS system is required to be networked with the ground
station running MAVProxy.

It is assumed that the DGPS data comes from the network address 127.0.0.1:13320.

The DGPS data can be in the SBP/RTCP/UBC protocols

