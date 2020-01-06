
==========
UAVCAN GUI
==========

UAVCAN_GUI is a tool that allows viewing, configuration and software updates of nodes
connected to the BUS.

Download `UAVCAN_GUI <https://uavcan.org/GUI_Tool/Overview/>`_ and install.

Before the autopilot can be connected SLCAN mode must be available. <Link>

**Set baud rate**

.. image:: ../../../images/can_uavcan_gui_baud.png

UAVCAN_GUI tool will start with an interface as the image below.
Click on tick next to node address. Leave the node address unless you have another on the Bus with the same address.

.. image:: ../../../images/can_uavcan_gui.png

If the node has bootloader only installed then firmware will need to be
uploaded to the node. Maintenance will be displayed. Click on the button lower right and then double Click
on org.ardupilot.ap_periph as highlighted.

.. image:: ../../../images/can_uavcan_gui_upd.png

The following pop up window will appear. Click on update firmware and select the correct file for the node connected.
These can be found `here. <https://firmware.ardupilot.org/AP_Periph/>`_

.. image:: ../../../images/can_uavcan_gui_pop.png

Once the firmware has finished uploading to node the main window will change to operational as per
image above. Press fetch all button, double click on flash bootloader and enter 1 in the highlighted box. Press
send and close. The address of the node can be changed in this window to avoid conflict with another node on the CANBUS.

.. image:: ../../../images/can_uavcan_gui_supd.png

A debug message will show complete.

.. image:: ../../../images/can_uavcan_gui_supdc.png

Messages coming through the CANBUS can be viewed using the bus monitor tool in "Tools"

.. image:: ../../../images/can_uavcan_gui_mon.png

To return to normal operation, close UAVCAN_GUI tool power down/reboot.
