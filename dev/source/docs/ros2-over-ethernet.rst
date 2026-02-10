.. _ros2-over-ethernet:

===================
ROS 2 over Ethernet
===================

The following tutorial explains how to connect a flight controller to a companion computer using ethernet for DDS communication.

Ensure you have ROS 2 :ref:`installed <ros2>` and have run :ref:`SITL <ros2-sitl>` successfully before attempting this page.

Additionally, make sure you understand the :ref:`basics of networking in ArduPilot. <common-network>`

.. note:: ROS2 (via DDS) is not part of the standard ArduPilot build. Use the `Custom Build Server <https://custom.ardupilot.org/add_build>`__ and ensure "MicroXRCE DDS support for ROS 2" is checked.

Motivation
==========

Starting in ArduPilot 4.5, ethernet is now supported as a new communication interface.
Compared to connecting an autopilot over serial, ethernet has the following advantages:

* Ethernet is more immune to noise because it uses twisted pair wiring
* Ethernet can run multiple protocols over the same protocol (unless you use PPP)
* Ethernet is easier to debug with tools such as ``tcpdump`` or ``Wireshark``
* Ethernet uses standard cabling, so it's more difficult to mix up TX and RX

Physical Equipment
==================

The following equipment is required to complete this tutorial:

* A computer that can run the MicroROS Agent and the ROS 2 CLI
* An autopilot with ethernet support

Necessary parameters for static configuration
=============================================

This list of parameters is given as an example.
If you had the following static IP addresses:

* An autopilot has the IP address ``192.168.144.14``
* A Computer running the MicroROS agent has the IP address ``192.168.144.6``
* The MicroROS agent is running on port ``2019``

Then, you would configure all of the below parameters.


.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - Parameter Name
     - Value
   * - :ref:`DDS_ENABLE<DDS_ENABLE>`
     - 1
   * - :ref:`DDS_IP0<DDS_IP0>`
     - 192
   * - :ref:`DDS_IP1<DDS_IP1>`
     - 168
   * - :ref:`DDS_IP2<DDS_IP2>`
     - 144
   * - :ref:`DDS_IP3<DDS_IP3>`
     - 6
   * - :ref:`DDS_UDP_PORT<DDS_UDP_PORT>`
     - 2019
   * - :ref:`NET_DHCP<NET_DHCP>`
     - 0
   * - :ref:`NET_ENABLE<NET_ENABLE>`
     - 1
   * - :ref:`NET_IPADDR0<NET_IPADDR0>`
     - 192
   * - :ref:`NET_IPADDR1<NET_IPADDR1>`
     - 168
   * - :ref:`NET_IPADDR2<NET_IPADDR2>`
     - 144
   * - :ref:`NET_IPADDR3<NET_IPADDR3>`
     - 14


Modify the addresses to suit your needs; the rest can remain the same.

Steps
=====

#. Flash the autopilot with software compiled with ``--enable-DDS``
#. Connect the autopilot via ethernet to the computer
#. Open a MavProxy session
#. Configure the parameters described above, starting with the ``ENABLE`` parameters first. 
#. Reboot the flight controller
#. Start the MicroROS Agent with the same port as the parameter for ``DDS_UDP_PORT``

  .. tabs::

    .. group-tab:: ArduPilot 4.5

        .. code-block:: bash

          ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019 -r dds_xrce_profile.xml

    .. group-tab:: ArduPilot 4.6 and later

        .. code-block:: bash

          ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

#. Use the ROS 2 CLI to interact with the autopilot
