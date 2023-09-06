.. _common-tattu-dronecan-battery:

===========================
Tattu Plus DroneCAN Battery
===========================

.. image:: ../../../images/tattu-dronecan-battery.jpg
   :target: ../_images/tattu-dronecan-battery.jpg
   :width: 450px

Tattu Plus batteries with AS150U connectors support DroneCAN which allows the autopilot to retrieve the battery's total voltage, individual cell voltages, current, temperature and percentage of remaining capacity.

The contents of this page were verified using a Tattu Plus 1.0 battery with SKU:TAA16KP12S15C.

Some images courtesy of genstattu.com and 3dxr.co.uk

Connection and Configuration
============================

A female AS150U connector with leads should be used to connect the battery's signal wires to the autopilot's CAN port as shown below.

.. image:: ../../../images/tattu-dronecan-battery-autopilot.png
   :target: ../_images/tattu-dronecan-battery-autopilot.png
   :width: 450px

Connect to the autopilot with a ground station and set the following parameters and then reboot the autopilot

- Set :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` to 1 (First Driver)
- Set :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` to 1 (DroneCAN)
- Set :ref:`BATT_MONITOR<BATT_MONITOR>` to 8 (DroneCAN)

Testing
=======

Once connected the battery voltage, current, percent remaining, etc will be visible in the ground station and recorded in the onboard logs.  The image below shows where the data appears on Mission Planner's status screen.

.. image:: ../../../images/tattu-dronecan-battery-mp.png
   :target: ../_images/tattu-dronecan-battery-mp.png
   :width: 450px
