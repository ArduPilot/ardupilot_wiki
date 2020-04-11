.. _companion-computers:

===================
Companion Computers
===================

Companion Computers can be used to interface and communicate with
ArduPilot on a flight controller using the MAVLink protocol.  By doing
this your companion computer gets all the MAVLink data produced by the
autopilot (including GPS data) and can use it to make intelligent
decisions during flight. For example, "take a photo when the vehicle is
at these GPS co-ordinates", gather and pre-process information from advanced
sensors or actuate lights, auxiliary servos or any other interfaces.

There are two major parts to Companion Computers - hardware and software.

The Companion Computer hardware refers to the specific computer hardware being used.
This is typically a small ARM-based Single Board Computer. Specific tutorials for 
popular Companion Computer hardware are listed below.

.. toctree::
    :maxdepth: 1

    Arduino family <https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566/1>
    LYCHEE (Cube Carrier Board for Raspberry Pi Compute Module) <https://dronee.aero/pages/lychee>
    NVidia TX1 <companion-computer-nvidia-tx1>
    NVidia TX2 <companion-computer-nvidia-tx2>
    ODroid <odroid-via-mavlink>
    Raspberry Pi <raspberry-pi-via-mavlink>

The Companion Computer software refers to the programs and tools that run on the Companion
Computer. They will take in MAVLink telemetry from the Flight Controller and can route and 
process the telemetry data. Specific tutorials for popular Companion Computer software 
tools/suites are listed below.

.. toctree::
    :maxdepth: 1

    APSync <apsync-intro>
    DroneKit <droneapi-tutorial>
    FlytOS <flytos>
    Maverick <https://goodrobots.github.io/maverick/#/>
    ROS <ros>
    Rpanion-server <https://www.docs.rpanion.com/software/rpanion-server>

A number of vendors also offer turnkey systems. See the below link for details.

.. toctree::
    :maxdepth: 1

    Turnkey Companion Computer Solutions <turnkey-companion-computer-solutions>

