==============
Packet Signing
==============

.. code:: bash

    module load signing
    
The signing module sets up and controls the signing of packet with a key. In Mavlink 2.0, 
this is a feature that allows a vehicle to only accept commands from packets signed with the 
correct key. This is useful as it lowers the risk of a 3rd party gaining control over the vehicle.

Setup and Removal
=================

.. code:: bash

    signing setup KEY
    
Set the key to ``KEY`` on both MAVProxy and the vehicle. **This is sent in plaintext to the 
vehicle, so ensure this setup is done over a secure link.**

All packets sent to the vehicle after this command will be accompanied with a SHA-512 hash of the key 
to confirm the packet's authenticity.

.. code:: bash

    signing remove KEY
    
Remove the signing key ``KEY`` from MAVProxy and the vehicle.

.. code:: bash

    signing disable KEY
    
Disable signing of packets with key ``KEY`` from MAVProxy. Does not affect the vehicle - 
it will still be expecting signed packets.

.. code:: bash

    signing key KEY
    
Enable signing of packets with key ``KEY`` from MAVProxy. Does not affect the vehicle.


