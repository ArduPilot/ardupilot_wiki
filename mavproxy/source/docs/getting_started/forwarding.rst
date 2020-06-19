.. _mavproxy-forwarding:

====================
Telemetry Forwarding
====================

A key feature of MAVProxy is its ability to forward the messages from
your UAV over the network via UDP to multiple other ground station
software on other devices. For example: you can run a ground station on
a laptop next to your antenna and forward via wifi to a
smartphone/tablet which lets you easily relocate to launch into wind
before heading back to your fixed antenna. I have also used it to send
telemetry data to a friend acting as spotter several kilometers away
(via 4G vpn) during a longer flight so that he could monitor the entire
flight and determine where to look to find the aircraft in flight.

To forward the MAV data over the network including to a local program on
your PC we simply need to add some extra parameters when starting
MAVProxy via the command line.

To connect with a local ground station software (such as Mission Planner)
start MAVProxy as above with the command:

.. code:: bash
 
    mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --out 127.0.0.1:14550

Then open the ground station software and select a UDP
connection on port 14550. It should then connect to your UAV.

Finally you can add the IP address of any computer to forward the
telemetry stream onwards to other ground stations.

#. On the local network/wifi you will need to ensure there is no
   firewall on the client PC stopping the incoming stream to your ground
   station software.
#. Add --out IP_ADDRESS:14550 to the end of the mavproxy command.
   You can add as many separate --out parameters as you want depending
   on how many extra ground stations you are running.
#. Set each ground station to listen for UDP packets on port 14550

