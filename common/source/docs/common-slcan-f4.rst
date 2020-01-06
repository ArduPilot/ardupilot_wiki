SLCAN F4 based Autopilots
-------------------------
Enabling SLCAN allows the autopilot to connect to CANBUS through USB on PC.
It allows viewing, configuration and software updates of devices connected to the BUS.
There are two different approaches for using SLCAN on UAVCAN_GUI or Mission Planner.

**Mission Planner**

Connect the flight controller/autopilot to Mission Planner.

| Set **CAN_SLCAN_CPORT 1** and reboot.

.. image:: ../../../images/can-slcan-cport.png

Connect normally to Mission Planner and navigate to setup, optional hardware, UAVCAN.
Click on SLCan Mode, firmware updates and parameter changes of the node
are available.  To return the autopilot disconnect and reboot/power down.

.. image:: ../../../images/can-slcan-mpc.png

|
| **UAVCAN_GUI**
|
| Connect the flight autopilot to Mission Planner.
| Set the parameters as above **CAN_SLCAN_CPORT 1** and reboot.
| Set **CAN_SLCAN_SERNUM 0** This will direct the output to USB. Do not reboot, close Mission Planner

.. image:: ../../../images/can-slcan-srnum.png

| Autopilot can be connected to UAVCAN_gui. <page-link>
| or MP-Slcan <page link>
