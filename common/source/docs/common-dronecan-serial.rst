.. _common-dronecan-serial:

===========================
DroneCAN Serial PassThrough
===========================

Normally, any serial port on an AP-Peripheral device, like a :ref:`common-uavcan-adapter-node` will be pre-configured for a specific peripheral (like a GPS or Rangefinder)to be attached to one or more of its UARTs.

However, firmware versions V4.5 and later have added the ability to bibidirectionally pass through data to a DrooneCAN peripheral device that has serial ports defined.

Up to three ports can be mapped into the vehicle from the peripheral and their baud rate and protocol programmed with ArduPilot parameters:

- First, the CAN port driver must be set to "DroneCAN" protocol.
- Second, the driver's ``CAN_Dx_UC_SER_EN`` must be set to "1"  to enable this feature. A reboot may be required to allow the following parameters to be shown and set. Note that up to 3 serial ports may be enabled per driver. For the following only the first port (1) parameters will be listed:

- :ref:`CAN_D1_UC_S1_NOD<CAN_D1_UC_S1_NOD>`: CAN remote node number for serial port. See "Setting the Node Number" section below.
- :ref:`CAN_D1_UC_S1_IDX<CAN_D1_UC_S1_IDX>`: The peripheral' Serial Port index that the following parameters apply. This selects the SERIALx of the peripheral and its corresponding UART. Consult the peripherals specifications or its "hwdef.dat" file `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef>`__ to determine its index.
- :ref:`CAN_D1_UC_S1_BD<CAN_D1_UC_S1_BD>`: The baud rate for the port
- :ref:`CAN_D1_UC_S1_PRO<CAN_D1_UC_S1_PRO>`: The ports protocol for the device/connection to its UART.

Setting the Node Number
=======================

DroneCAN peripherals usually get a CAN bus node number automatically assigned by ArduPilot. For most peripheral setups, knowledge of this node number is not needed by the user (the exception being two DroneCAN GPSs in a Moving Baseline configuration). However, for Serial passthrough, this number is needed to be determined and placed in the ``CAN_Dx_UC_S1_NOD`` parameter to enable communication to occur properly.

The user can attach the DroneCAN device and look up its node number using :ref:`Mission Planner <dronecan-uavcan-slcan>` or :ref:`the DroneCAN GUI<dronecan-uavcan-slcan>`. It is recommended to also change the peripheral's ``CAN_NODE`` parameter on the peripheral from "0" to the one assigned by ArduPilot to prevent its changing on subsequent system reconfigurations and invalidating the value set in ``CAN_Dx_UC_S1_NOD`` parameter.
