.. _common-mro-uavcan-adapter-node:

===============================
mRobotics DroneCAN Adapter Node
===============================


The mRo CAN Node is a collaborative project with ArduPilot. It began with a desire to make CAN peripherals more standard in the drone community. Compared to other boards that have open-source hardware but closed source code, mRo CAN node is completely open. The CAN Node F303 is the version 2 of mRobotics' CAN Node, featuring more flash space.

With this board, you can easily utilize the ArduPilot driver library to convert ArduPilot supported GPS, I2C, and GPIO based (ie Safety Switch) peripherals to DroneCAN bus peripherals.

.. image:: ../../../images/mRo-can-node.jpg


mRo CAN Node uses open-source code, which utilizes the existing and proven ArduPilot DroneCAN driver library (see :ref:`DroneCAN Adapter Nodes <common-uavcan-adapter-node>` for firmware versions and capabilities). It features a  bootloader for "over CAN" firmware updates and an RTOS running Chibios.

The adapter's CAN Node ID can be set manually or automatically (Not yet supported in stable ArduPilot). The hardware design is open source, so that anyone can contribute to the CAN peripheral world with a proven CAN / MCU interface.

As delivered, it is setup to be plug-and-play as a DroneCAN GPS/Magnetometer/LED/Airspeed by simply plugging in the 6pin GH connector from any mRo or other GPS, and connecting I2C Compasses, RGB LEDs, and/or Airspeed sensors supported by ArduPilot, as desired. 

This adapter uses the "f303-GPS" or "f303-Universal" (default) firmware in https://firmware.ardupilot.org/AP_Periph. See the :ref:`SLCAN section of DroneCAN advanced setup<common-uavcan-setup-advanced>` for downloading instructions, if changes are desired or updates become available.

Setup
=====

Verify the following parameters are set on your autopilot via your ground station.

- :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1 (assuming its on the first CAN bus of the autopilot and will be using the first driver)
- :ref:`CAN_D1_DRIVER<CAN_D1_PROTOCOL>` = 1 (DroneCAN)(assuming its the first driver)
- :ref:`GPS1_TYPE<GPS1_TYPE>` = 9 (DroneCAN) or if it will be the secondary GPS, set instead
- :ref:`GPS2_TYPE<GPS2_TYPE>` = 9 (DroneCAN)
- Reboot

Make sure the DroneCAN compass driver is not disabled. (:ref:`COMPASS_DISBLMSK<COMPASS_DISBLMSK>` bit 9 should not be set). 

If your controller has multiple compasses attached already, you may want to set the attached compass as a higher priority than others, if it has better precision or performance.

To enable an attached I2C RGB LED set:

- :ref:`NTF_LED_TYPES<NTF_LED_TYPES>` bit 5 should be set 

If you want to enable an attached I2C airspeed sensor, set:

- :ref:`ARSPD_TYPE<ARSPD_TYPE>` = 8 (DroneCAN)

Then you must access your Adapter's CAN parameters to select Airspeed and LED type being attached to the adapter. Using Mission Planner (or :ref:`common-uavcan-gui`), go to the menu SETUP/Optional Hardware/DroneCAN/SLCan Mode CAN1/Parameters. Once the CAN Adapters parameters appear, change the following (this changes them inside the DroneCAN adapter, NOT the autopilot, allowing the adapter to use the correct driver and convert the information to DroneCAN airspeed format):

- :ref:`ARSPD_TYPE<ARSPD_TYPE>` = to match your sensor type: 3,for an I2C-MS5525; 1 for an MS4525DO, etc.
- :ref:`ARSPD_USE<ARSPD_USE>` =  1
- :ref:`ARSPD_BUS<ARSPD_BUS>` =  0

For the LED, set its :ref:`NTF_LED_TYPES<NTF_LED_TYPES>` appropriately for the attached LED.

Then reboot.

Where to buy
============

`mRobotics <https://store.mrobotics.io/product-p/mro10042a.htm>`_

Schematics
==========

`Schematic <https://github.com/ArduPilot/Schematics/blob/master/mRobotics/mRo_CANnode_V1_R1.pdf>`_

.. note:: F103 is replaced with F303 microprocessor
