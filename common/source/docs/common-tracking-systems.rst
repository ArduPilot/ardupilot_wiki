.. _common-tracking-systems:

=============================================
Alpha Unmanned Systems Vessel Control Station (VCS)
=============================================

Alpha Unmanned Systems develops and supports the **Vessel-based Control Station (VCS)** for tracking any device compatible with the **MAVLink V1 or V2 protocol**.

    .. image:: ../../../images/aus-vcs.png
        :target: ../_images/aus-vcs.png
        :width: 400px
        :align: center
      
The gyrostabilized mobile tracking antenna system includes **four Ethernet ports**: one port for connecting to the **radio link**, and three ports for connecting the **GNSS**, **joystick**, and **PC**.

System Pack
-----------
- `Vessel-based Tracking Station <https://alphaunmannedsystems.com/product/vessel-based-control-station/>`__.
- **AC power cable** with **24V external supply connector**.
- 3x **IP67 Ethernet cables**.
- **Mission Planner Plugin**.

Autopilot and Mission Planner Configuration
--------------------

The **Vessel Control Station** includes **900 MHz** and **2.4 GHz** selectable links for direct communication with the tracked UAV. To connect to the UAV’s autopilot, at least one of these links must be active. 

Each link will be connected via a **serial RS232 interface** to either **TELEM1** or **TELEM2** on the autopilot. Since these ports operate with **UART**, a **TTL-RS232 converter** is required. The corresponding connections are shown in the diagram below:

  .. image:: ../../../images/aus-vcs-air-link.png
      :target: ../_images/aus-vcs-air-link.png
      :width: 200px
      :align: center

In **Mission Planner**, connect the autopilot and configure the **baud rates** for **TELEM1** and **TELEM2** by adjusting the following parameters:

- :ref:`**SERIAL0_BAUD** <SERIAL0_BAUD>` = 115 (115200 bps) if using serial port 0.
- :ref:`**SERIAL0_PROTOCOL** <SERIAL0_PROTOCOL>` = 2 (MAVLink2) if using MAVLink2 protocol. If MAVLink1 is needed, set parameter to 1.
- :ref:`**SERIAL1_BAUD** <SERIAL1_BAUD>` = 115 (115200 bps) if using serial port 0.
- :ref:`**SERIAL1_PROTOCOL** <SERIAL1_PROTOCOL>` = 2 (MAVLink2) if using MAVLink2 protocol. If MAVLink1 is needed, set parameter to 1.

The **VCS** will receive information from both links and transmit it over **Ethernet** via port **1976**.  
Connect the **PC**, acting as the ground station, to the VCS and open Mission Planner. Then, connect to **UDP port 1976**.

  .. image:: ../../../images/aus-vcs-mp-conn.png
      :target: ../_images/aus-vcs-mp-conn.png
      :width: 600px
      :align: center

If all the radios are linked and configured, Mission Planner should receive **autopilot configuration data**.  
Additionally, the system provides a **Mission Planner plugin** to monitor the real-time status of the VCS, links, GNSS, and user commands such as **link switching**, **storage mode**, **reset**, and **static position tracking**. 

For more information, please contact `Alpha Unmanned Systems <https://alphaunmannedsystems.com/contact/>`__.