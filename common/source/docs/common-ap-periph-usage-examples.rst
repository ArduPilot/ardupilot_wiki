.. _common-ap-periph-usage-examples:
[copywiki destination="copter,plane,rover,dev"]
======================
DroneCAN Adapter Setup
======================

These are several ways that a DroneCAN adapter node running AP_Periph can be used and this page is a small selection of the possibilities. These sections should walk you through setting up the autopilot and the node so that they work together.

.. note:: DroneCAN Adapter nodes usually come with firmware that supports a subset of ArduPilot's large peripheral capability. Be sure to check that the firmware supplied supports the peripherals you are attemptime to setup. If not, check the `Firmware <https://firmware.ardupilot.org/AP_Periph/>`__  page for possible alternative firmware for that adapter that may have the capability.

GPS Setup
=========
Attach the GPS to a DroneCAN Adapter's serial port

- set :ref:`GPS1_TYPE<GPS1_TYPE>` = 9 on the autopilot
- set the DroneCAN adapter's ``SERIALx_PROTOCOL`` on which the GPS is connected to 5 (GPS)

Compass Setup
=============
Attach the Compass to the DroneCAN Adapter's external I2C port. It will appear as a normal Compass in ArduPilot and normal :ref:`compass setup procedures<common-compass-calibration-in-mission-planner>` would apply.

.. note:: The MatekL431 CAN Adapter comes setup for a SPI connected Compass (RM3100). If an I2C Compass is attached, different firmware must be loaded for it to function.

I2C Airspeed Sensor
===================
Attach to the I2C bus pinned out from the DroneCAN Adapter.

- set :ref:`ARSPD_TYPE<ARSPD_TYPE>` =8 (DroneCAN)
- set the DroneCAN's ``ARSPD_TYPE`` parameter to match the type of airspeed sensor connected.

Rangefinder Setup
=================
To use rangefinders, follow the instructions at  :ref:`DroneCAN Setup Advanced<common-uavcan-setup-advanced>` to set up the ArduPilot parameters. Using MissionPlanner or DroneCAN Gui, set the parameters on the adaptor node following the instructions for the relevant rangefinder.

.. note:: The orientation of the rangefinder (RNGFND1_ORIENT) must be set to 0 on the adaptor node.

.. note:: The RNGFNDx_ADDR ArduPilot parameter must be set above 0 and be equal to the number set on the DroneCAN adapter node.

PWM or DShot Output Node
========================
Adapters that have timer outputs available can be used as PWM and DShot output nodes.

The `MatekL431 DroneCAN PWM device <https://www.mateksys.com/?portfolio=can-l4-pwm>`__ is an example of this.

This can be useful for providing additional servo outputs beyond what the autopilot provides, but is most useful for allowing the ESCs and Sevos to be remotely located, far from the autopilot using the CAN bus to communicate. A maximum of 32 outputs in ArduPilot is possible.

In addition, the autopilot outputs can be configured for GPIO use, and still allow up to 32 servo/motor outputs to be realized with DroneCAN peripherals.

.. note:: In order to use DShot output, the node must have a DShot-capable firmware installed.

DShot ESC Adapter
-----------------
In this example, a quadplane with outputs 5-8 used for 4 DShot ESCs with telemetry will be shown. 3 wires from each ESC should be connected to the DroneCAN adapter node: Ground, DShot signal, and serial telemetry. The ground from each ESC should be connected to a ground pad on the DroneCAN node. The signal wires from the 4 ESCs should be connected to the first 4 PWM pads on the node, marked as outputs “1”, “2”, “3”, and “4”. The serial telemetry pin from all 4 ESCs should be all connected to the same UART RX pin on the adapter node.

On the main autopilot you need to set:

- :ref:`CAN_Px_DRIVER <CAN_P1_DRIVER>` = 1
- :ref:`CAN_Dx_PROTOCOL <CAN_D1_PROTOCOL>` = 1
- :ref:`CAN_Dx_UC_ESC_BM <CAN_D1_UC_ESC_BM>` = 240 or the bitmask of motor outputs
- :ref:`CAN_Dx_UC_ESC_OF <CAN_D1_UC_ESC_OF>` = 4 or the offset number to the first ESC output. This makes the transmission of CAN packets much more efficient

On the DroneCAN PWM node you need to set:

- :ref:`OUTx_FUNCTION <dev:OUT1_FUNCTION>` = 33 + ESC number
- :ref:`OUT_BLH_MASK <dev:OUT_BLH_MASK>` = 15 or a bitmask of which ESCs are active

PWM Output
----------
On the main autopilot you need to set:

- :ref:`CAN_Dx_UC_SRV_BM <CAN_D1_UC_SRV_BM>` to a bitmask of servos you want to send over CAN.
- :ref:`CAN_Dx_UC_SRV_RT <CAN_D1_UC_SRV_RT>` to the output rate. This is typically 50 Hz for most servos.

On the CAN node you need to set:

- :ref:`OUT_BLH_MASK <dev:OUT_BLH_MASK>` = 0 to disable DShot
- :ref:`ESC_PWM_TYPE <dev:ESC_PWM_TYPE>` = 0 for normal PWM
- :ref:`OUTx_FUNCTION <dev:OUT1_FUNCTION>` to a value of 50 plus the servo number for each output you want to be enabled as PWM output
- :ref:`OUTx_FUNCTION <dev:OUT1_FUNCTION>` = 0 for any outputs you do not have connected. Do not leave them at the default of 33 + ESC number

For example, if you had an elevator servo on SERVO2 on the main autopilot and you want this to appear on the first output of the CAN node (on the node's pin marked “1”) then you would set OUT1_FUNCTION = 52 (that is 50 + the servo number on the autopilot). If you wanted a rudder that is on SERVO4 to appear on output 4 then you would set OUT4_FUNCTION=54.

.. note:: It is also recommended to set OUTn_MIN to 1000, OUTn_MAX to 2000, and OUTn_TRIM to 1500. That will allow you to use the SERVOn_MIN, SERVOn_MAX, and SERVOn_TRIM values on the main autopilot to control the outputs range and center in the usual way. It is possible to use other values on the node, but it gets more complicated to understand the mapping of the PWM values, so using 1000, 1500, 2000 is recommended. Doing this also means the PWM value in your autopilot logs matches what is output by the node.

Combining DShot and PWM outputs
-------------------------------
There are a few rules to follow when combining PWM and DShot outputs on the same node. If you are familiar with doing this on autopilot, it is the same rules. The pins are grouped by timer and all of the pins attached to a timer must be the same type. For example, on the MatekL431 node the first 4 outputs are all on the same timer. This means that those output pins must all be DShot outputs and 1 PWM on the fifth output or the first 4 as PWM and 1 DShot.
