.. _common-tracking-systems:

===================================================
Alpha Unmanned Systems Vessel Control Station (VCS)
===================================================

[copywiki destination="copter,plane"]

Alpha Unmanned Systems supports the **Vessel-Based Control Station (VCS)** for tracking any device compatible with the MAVLink V1 or V2 protocol.
    .. image:: ../../../images/aus_vcs/aus-vcs.png
        :target: ../_images/aus_vcs/aus-vcs.png
        :width: 400px
        :align: center
      
The gyrostabilized mobile tracking antenna includes **four Ethernet ports**: one port for connecting to the radio link, and three ports for connecting the GNSS, joystick, and PC. The system tracks automatically the target when a GPS position from the UAV is received through the radio links and the local position received by the installed GNSS. Radio links are redundant and selected automatically by the system or by the user.

System Pack
-----------
- `Vessel-based Tracking Station <https://alphaunmannedsystems.com/product/vessel-based-control-station/>`__.
- **AC power cable** with **24V external supply connector**.
- 3x **IP67 Ethernet cables**.
- **Mission Planner Plugin**.

System Setup
-------------
The **VTA** (Vessel-based Tracking Antenna) is the main system of a Mobile or Vessel-based Control Station, but to be fully operational, it must be coupled with the following systems:

* UAV Operator laptop (or fixed PC)
* Payload operator laptop (or fixed PC)
* Joystick Ethernet 
* AC/DC power supply
* Optional backup battery (output voltage must be below 24V: 18-23V)

  .. image:: ../../../images/aus_vcs/aus_connection_vcs.png
      :target: ../_images/aus_vcs/aus_connection_vcs.png
      :width: 500px
      :align: center

Connection Diagram
-------------------
The system is composed of 4 ETH connectors available for the connection of external systems. They are: 

* **ETH 0 – VIDEO:** This is the only ETH port for payload connection. It is not connected to the ETH Switch, but directly to the data link. Normally, this port would be connected to the payload control laptop or PC. This port does not have POE (Power Over Ethernet).
* **ETH 2 & 4 – CS/JY:** Port for the connection of the control station laptop or UAV control Joystick. This port does have POE.
* **ETH 5 – IMU & GNSSCOMPASS:** This port is reserved for the connection of the IMU & GNSSCOMPASS module. This port does have POE.

  .. image:: ../../../images/aus_vcs/aus_vcs_connectors.png
      :target: ../_images/aus_vcs/aus_vcs_connectors.png
      :width: 600px
      :align: center

Data Link Interface
---------------------
The system has available two redundant datalinks which once configured can connect automatically to the airborne datalinks, using one at a time.

One datalink of 900MHz is placed internally into the VTA and transfers only telemetry information to the tracking system, while the other datalink of a wide range of available frequencies (2.4-5.8 GHz) is externally placed into the Pan Tilt Unit (PTU) and transfers telemetry information and video through the external port.

The following diagram shows the pinout of the PTU used to connect and power data links. This is to be used only for custom data link integrations.

  .. image:: ../../../images/aus_vcs/aus_vcs_payload1.png
      :target: ../_images/aus_vcs/aus_vcs_payload1.png
      :width: 400px
      :align: center

  .. image:: ../../../images/aus_vcs/aus_vcs_payload2.png
      :target: ../_images/aus_vcs/aus_vcs_payload2.png
      :width: 400px
      :align: center

.. note::
   Telemetry is transferred through serial interface to the internal tracking controller, which selects automatically the active link and transfers the information to the PC through the HUB port.

Data output
------------

The VTA main processing unit has its own proprietary ICD to feed other systems (C2 or UAV ground control software) with its status information. 
This information can be streamed in parallel with other streams like NMEA 0183/NMEA 2000 output or the GNSS and compass IMU information. 

.. note::
   More information about the specific communications protocol (ICD) can be requested from `Alpha Unmanned Systems <https://alphaunmannedsystems.com/contact/>`__.

VCS Installation and integration on vessels
-------------------------------------------

Vessels are equipped with a variety of RF equipment, typically including radars and satcom devices. As they may interfere with the communications between the VTA and the UAV, there are some general rules for finding a good spot for installation on the vessel: 

* VTA antennas or secondary radios must be as far as possible from the radar and, if possible, in a different horizontal plane to avoid being under the RF interference of the radar. 
* The GNSS compass and IMU must have a clear view of the sky to correctly receive the signal from GNSS satellites. 
* If some area between the VTA and the UAV’s flight zone would be shadowed by the vessel’s mast or any other vessel structure, this can be solved by installing the 900MHz antenna or secondary radio systems and antennas at a different location. A schematic overview of potential locations can be found below.

  .. image:: ../../../images/aus_vcs/vessel_installation_schematic.png
      :target: ../_images/aus_vcs/vessel_installation_schematic.png
      :width: 600px
      :align: center

* BLUE circles represent potential spots for the installation of the main VTA. 
* RED circles represent the potential installation points of the 900 MHz antennas, secondary radio or secondary VTA.
* BLUE lines are ETH cables to be routed from one system to the other. 

Integration steps
-----------------

#. **Antenna location definition:** based on line of sight and RF compatibility with other systems. 
#. **Operators location definition:** the permanent location for the operator must be defined taking into consideration that EP will require visual access to the UAV during take-off and landing. Operators room must be able to accommodate 2 operators with 2 laptops and it is desirable to have extra screens. 
#. **Antenna Interface definition:** interface between antenna and vessel is defined. Normally it will be a mast with a plate on top matching the holes pattern of the base of the VCS antennas. 
#. **Distances:** lengths between all the components (antenna, center of heli-deck and operators room) must be defined. These lengths will be used to define reference offsets and to manufacture the cables with the appropriate measure. 

Vessel Based Tracking Antenna Requirements
------------------------------------------

The tracking antenna has the following requirements that must be taken into consideration: 

* **Power supply:** the antenna utilizes 220 V AC power supply; it can include a 24V DC backup power source but normally on a vessel the AC power source would be integrated with a UPS system. From the control room, the power supply of the VTA must be able be switched on and off by means of a switch as the ON/OFF button of the system will not be accessible. 
* **Cables routing:** 2 ETH cables and one power supply cable must be routed from the antenna to the operator’s room. One more ETH cable must be routed from the antenna to the flight deck for the control joystick of the external pilot.

System Configuration
--------------------

By default the device has internally the following IP configurations to connect between the different systems:

.. table:: 
   :align: center

   ===========  =============
   Parameter    Value
   ===========  =============
   vcsIp        192.168.0.15
   gcsIp        192.168.0.11
   gcsPort      1976
   gnssIp       192.168.0.45
   mask         255.255.255.0
   gateway      192.168.0.1
   ===========  =============

Ground PC IP must be configured to be the same as gcsIp to connect, then the user can configure these IPs connecting by UDP to the following parameters with any UDP terminal such as **ScriptCommunicator or Putty**:

.. table::
   :align: center

   ==========  ============
   Parameter   Value
   ==========  ============
   localport   1999
   remoteIP    192.168.0.15
   remotePort  1999
   ==========  ============

Once connected to the VCS, the following commands allow to change any of the mentioned parameters:

.. list-table:: System Parameters
   :widths: 20 80
   :header-rows: 1

   * - Command
     - Description
   * - help
     - This command shows all the available commands. When followed by another command it shows a brief description of how to use the command.
   * - getConfig
     - This command is used to show the current network configuration. 
   * - default
     - This command returns all the configurable parameters to their default values.
   * - reboot
     - This command reboots the VTA. Some parameters, like all the network parameters, require the VTA to reboot for the changes to take effect.

.. list-table:: Network Parameters
   :widths: 20 80
   :header-rows: 1

   * - Command
     - Description
   * - vcsIp
     - This command configures the VCS IP.
   * - vcsPort
     - This command configures the Port used to send the Alarms to Visionair.
   * - gcsIp
     - This command configures the IP of the PC running Visionair.
   * - gcsPort
     - This command configures the Port used to send the Autopilot Telemetry to Visionair.
   * - gnssIp
     - This command configures the IP of the GNSS module.
   * - gnssPort
     - This command configures the Port used by the GNSS to send the ...
   * - mask
     - This command configures the MASK used by the network.
   * - gateway
     - This command configures the GATEWAY of the network.

In case some of those parameters are incorrect and the PC cannot connect to the system after reboot, the user can connect through a USB cable to a side USB port to send the ``default`` command mentioned previously.
  .. image:: ../../../images/aus_vcs/aus_vcs_usb_connector.png
      :target: ../_images/aus_vcs/aus_vcs_usb_connector.png
      :width: 500px
      :align: center
.. note::
   For advanced GNSS configuration, please contact `Alpha Unmanned Systems <https://alphaunmannedsystems.com/contact/>`__ for detailed instructions.

To connect the links mentioned to the UAV's autopilot, at least one of these links must be active.

Each link will be connected via a serial RS232 interface to either ``TELEM1`` or ``TELEM2`` on the aircraft autopilot. Since these ports operate with UART, a TTL-RS232 converter is required. 

Both air and ground links must be configured based on the manufacturer's instructions. When both air and ground links are configured, the VCS uses the main link as the active link by default and, in case this link is lost, automatically changes to the backup link (900MHz radio).

The corresponding connections are shown in the diagram below:
  .. image:: ../../../images/aus_vcs/aus-vcs-air-link.png
      :target: ../_images/aus_vcs/aus-vcs-air-link.png
      :width: 200px
      :align: center

In **Mission Planner**, connect the autopilot and configure the **baud rates** for **TELEM1** and **TELEM2** by adjusting the following parameters:

- :ref:`**SERIAL0_BAUD** <SERIAL0_BAUD>` = 115 (115200 bps) if using serial port 0.
- :ref:`**SERIAL0_PROTOCOL** <SERIAL0_PROTOCOL>` = 2 (MAVLink2) if using MAVLink2 protocol. If MAVLink1 is needed, set parameter to 1.
- :ref:`**SERIAL1_BAUD** <SERIAL1_BAUD>` = 115 (115200 bps) if using serial port 0.
- :ref:`**SERIAL1_PROTOCOL** <SERIAL1_PROTOCOL>` = 2 (MAVLink2) if using MAVLink2 protocol. If MAVLink1 is needed, set parameter to 1.

The **VCS** will receive information from both links and transmit it over **Ethernet** via port **1976**.  
Connect the **PC**, acting as the ground station, to the VCS and open Mission Planner. Then, connect to **UDP port 1976**.

  .. image:: ../../../images/aus_vcs/aus-vcs-mp-conn.png
      :target: ../_images/aus_vcs/aus-vcs-mp-conn.png
      :width: 600px
      :align: center

If all the radios are linked and configured, Mission Planner should receive **autopilot configuration data**.  
Additionally, the system provides a **Mission Planner plugin** to monitor the real-time status of the VCS, links, GNSS, and user commands such as **link switching**, **storage mode**, **reset**, and **static position tracking**. 
  .. image:: ../../../images/aus_vcs/vcs_plugin.png
      :target: ../_images/aus_vcs/vcs_plugin.png
      :width: 600px
      :align: center

.. note::
    Both VCS configuration terminal and plugin must not be active at the same time due to connection interferences to internal ports.

For more information, please contact `Alpha Unmanned Systems <https://alphaunmannedsystems.com/contact/>`__.