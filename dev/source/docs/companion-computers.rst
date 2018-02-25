.. _companion-computers:

===================
Companion Computers
===================

Companion Computers can be used to interface and communicate with
ArduPilot on a flight controller using the MAVLink protocol. By doing
this your companion computer gets all the MAVLink data produced by the
autopilot (including GPS data) and can use it to make intelligent
decisions during flight. For example, "take a photo when the vehicle is
at these GPS co-ordinates", gather and preprocess information from advanced
sensors or actuate on lights, auxiliary servos or any other interfaces.

Companion Computers let flying tasks to the flight controller (pilot tasks),
meanwhile adding processing power for other tasks (on board crew tasks).

Related topics on this wiki include:

.. toctree::
    :maxdepth: 1

    APSync <apsync-intro>
    Arduino family <https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566/1>
    BeaglePilot Project <beaglepilot>
    DroneKit Tutorial <droneapi-tutorial>
    FlytOS <flytos>
    Intel Edison <intel-edison>
    Maverick <https://goodrobots.github.io/maverick/#/>
    NVidia TX1 <companion-computer-nvidia-tx1>
    ODroid <odroid-via-mavlink>
    Raspberry Pi <raspberry-pi-via-mavlink>
    Turnkey Companion Computer Solutions <turnkey-companion-computer-solutions>
