.. _common_dronecan_servos:

===============
DroneCAN Servos
===============

ArduPilot supports :ref:`DroneCAN<common-uavcan-setup-advanced>` servos. DroneCAN connected servos offer high noise immunity and longer lead runs that normal :ref:`PWM servos<common-servo>`.

-  `Currawong <https://www.currawongeng.com/can-servo/>`__
-  `Himark <http://himarkuaveng.com/cate-21898-23052.html>`__
-  `HiTec <https://www.hiteccs.com/actuators/products/search/servo/1/spec/1-18/accessory/0/sort/0>`__
-  `Zeus <http://zeus-actuators.com/>`__

Autopilot Setup
===============

There are several parameters that determine which autopilot servo/motor channels are sent to the DroneCAN servos:
For the examples below, the values are shown for DroneCAN driver #1 using CAN Port #1

-  :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1, which assigns driver1 to port1
-  :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` = 1 (DroneCAN protocol)
-  :ref:`CAN_D1_UC_NODE<CAN_D1_UC_NODE>` - which is the node ID of the autopilot sending the commands to the servos so that there can be differentiation between multiple sources on the CAN bus. This is normally automatically set during discovery, but can be altered for advanced configurations (multiple sources on the bus).
-  :ref:`CAN_D1_UC_ESC_BM<CAN_D1_UC_ESC_BM>` - bitmask that determines which autopilot servo/motor output signals are sent to the DroneCAN servos

.. note:: Himark servos use a different command set. In order to use them, also set :ref:`CAN_D1_UC_OPTION<CAN_D1_UC_OPTION>` bit 5 (+32).

