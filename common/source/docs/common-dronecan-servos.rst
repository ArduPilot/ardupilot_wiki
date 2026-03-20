.. _common_dronecan_servos:

===============
DroneCAN Servos
===============

ArduPilot supports :ref:`DroneCAN<common-uavcan-setup-advanced>` servos. DroneCAN connected servos offer high noise immunity and longer lead runs that normal :ref:`PWM servos<common-servo>`.

-  `Currawong <https://www.currawongeng.com/can-servo/>`__
-  `Himark <http://himarkuaveng.com/cate-21898-23052.html>`__
-  `Hitec <https://www.hiteccs.com/actuators/products/search/servo/1/spec/1-18/accessory/0/sort/0>`__
-  `Zeus <http://zeus-actuators.com/>`__

Autopilot Setup
===============

There are several parameters that determine which autopilot servo/motor channels are sent to the DroneCAN servos:
For the examples below, the values are shown for DroneCAN driver #1 using CAN Port #1

-  :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1, which assigns driver1 to port1
-  :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` = 1 (DroneCAN protocol)
-  :ref:`CAN_D1_UC_NODE<CAN_D1_UC_NODE>`: node ID of the autopilot sending the commands to the servos so that there can be differentiation between multiple sources on the CAN bus. This is normally automatically set during discovery, but can be altered for advanced configurations (multiple sources on the bus).
-  :ref:`CAN_D1_UC_SRV_BM<CAN_D1_UC_SRV_BM>`: bitmask of servo output channels that should be sent via DroneCAN
-  (Optionally) :ref:`CAN_D1_UC_SRV_RT<CAN_D1_UC_SRV_RT>`: rate that servo outputs are sent from the autopilot to the actuator

.. note:: Himark servos use a different command set. In order to use them, also set :ref:`CAN_D1_UC_OPTION<CAN_D1_UC_OPTION>` bit 5 (+32).

AP_Periph Setup
===============

To set up servo control on an AP_Periph node, set the following parameters on the node:

-  :ref:`OUTx_FUNCTION<dev:OUT1_FUNCTION>` = 51 for actuator id 1, 52 for actuator id 2, etc. This assigns the output channel to respond to ``uavcan.equipment.actuator.Command``.
-  :ref:`OUTx_MIN<dev:OUT1_MIN>` = 1100, the minimum PWM output in microseconds. Set according to servo specifications.
-  :ref:`OUTx_MAX<dev:OUT1_MAX>` = 1900, the maximum PWM output in microseconds. Set according to servo specifications.
-  :ref:`OUTx_TRIM<dev:OUT1_TRIM>` = 1500, the PWM output in microseconds at neutral autopilot command.
-  :ref:`OUTx_REVERSED<dev:OUT1_REVERSED>` = 0 (default), set to 1 to reverse the servo direction.
-  :ref:`OUT_RATE<dev:OUT_RATE>` = 50 (default), output rate in Hz for all servo outputs on the node.

Actuator Telemetry Setup
------------------------

Some AP_Periph nodes, such as the `Vimdrones L431 CAN Servo Hub <https://dev.vimdrones.com/products/vimdrones_can_servo_hub/>`__, support actuator telemetry via the ``uavcan.equipment.actuator.Status`` message, which allows monitoring of servo current draw and load as a percentage of rated current. Set the following parameters on the node:

-  :ref:`ACT_NUM_CHANS<dev:ACT_NUM_CHANS>` = 4, number of servo channels to monitor for telemetry.
-  :ref:`ACT_CURR_PIN1<dev:ACT_CURR_PIN1>` = 9, starting ADC pin for current monitoring on the Vimdrones L431 CAN Servo Hub used in this example. Subsequent channels use the next pins (10, 11, ...) on that node; other AP_Periph hardware may use different ADC pin numbering, so consult the specific AP_Periph board documentation to determine the correct ADC pin IDs.
-  :ref:`ACT_AMP_OFFSET<dev:ACT_AMP_OFFSET>` = 0, current sensor offset in mA. Used to zero-calibrate the sensor under no load.
-  :ref:`ACT_AMP_PERVLT<dev:ACT_AMP_PERVLT>` = 100, current sensor scale in mA per volt.
-  :ref:`ACT_CURR_MAX<dev:ACT_CURR_MAX>` = 2.5, maximum current in Amps, used to calculate ``power_rating_pct`` in the telemetry message.
-  :ref:`SRV_TLM_MSG_RATE<dev:SRV_TLM_MSG_RATE>` = 10, rate in Hz at which telemetry messages are sent.
