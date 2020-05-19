.. _common-slcan-f7h7:

======================================
SLCAN Access on F7/H7 Based Autopilots
======================================

Enabling SLCAN allows the autopilot to connect to CANBUS through USB on PC.
It allows viewing, configuration and software updates of devices connected to the CANBUS.
There are two different applications for using SLCAN to modify UAVCAN device parameters: UAVCAN GUI or Mission Planner. But first, the SLCAN interface must be enabled.

SLCAN Interface
===============

The ArduPilot firmware provides two software USB interfaces, accessed simultaneously from the single physical USB connector. In  Mission Planner, you will see two COM ports assigned to the autopilot (if not, see Troubleshooting section below). One is for the SLCAN interface, the other for the normal MAVLink based Ground Control Station (GCS) connection. 

In firmware 4.0 and earlier, they will be indistinguishable in the Mission Planner COM port selection box until you try to connect to the GCS with the SLCAN port that has been enabled for SLCAN protocol. (In later firmware versions, together with using the latest `Mission Planner driver set <https://firmware.ardupilot.org/Tools/MissionPlanner/driver.msi>`__ , each port will be clearly labeled.)

The port that will not connect (if already configured for SLCAN protocol, however, default is usually for MAVLink protocol so it can connect normally) is the SLCAN interface. This COM port is associated with a SERIALx port in the autopilot parameters. This SERIALx port will be the highest numbered port in the parameter list. 

If both COM ports connect to Mission Planner, then the SLCAN protocol has not been set on that port and must be changed to use SLCAN.

Configuring SLCAN SERIALx PORT
==============================

Connect to Mission Planner. In the Full Parameters list set the highest numbered ``SERIALx_PROTOCOL`` to 22 and reboot. Now only the non SLCAN USB COM port will connect to Mission Planner and the SLCAN port will be ready for use.


Making Changes to UAVCAN Devices
================================

Once SLCAN has been enabled, you can use :ref:`Mission Planner <common-mp-slcan>`  to make changes immediately to device parameters or firmware update.

:ref:`UAVCAN GUI tool <common-uavcan-gui>` can also be used


Troubleshooting
===============

Occasionally, Windows will only present one COM port in Mission Planner. In that case, you may still be able to connect to Mission Planner and change the parameters, if the COM port Windows has chosen to display is the autopilot port with MAVLink protocol selected.  However, you will most likely need to load the composite USB driver in order to obtain both COM ports, as shown in :ref:`these instructions<loading-composite-USB>` , once the SLCAN port protocol has been enabled.

If you reboot the autopilot, the composite Windows USB driver may be unloaded and the process repeated upon re-attachment.

If the SLCAN port has been configured, and if only one USB COM port appears, it may not be the normal MAVLink protocol port (usually SERIAL0) and you will not be able to connect Mission Planner. You can either re-enable the Composite Driver each time you attach, as above, or simply revert the SLCAN port to MAVLink protocol once attached with the Composite driver and subsequently use the single COM port Windows presents when connecting to Mission Planner.

