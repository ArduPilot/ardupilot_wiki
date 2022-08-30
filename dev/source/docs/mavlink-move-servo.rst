.. _mavlink-move-servo:

============
Move a Servo
============

This page explains how MAVLink can be used by a ground station or companion computer to move a servo.  The :ref:`user wiki pages for servos is here <copter:common-servo-landingpage>`.

Set the Servo position with MAV_CMD_DO_SET_SERVO
------------------------------------------------

The servo's position can be set by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command, param1 and param2 fields set as specified for the `MAV_CMD_DO_SET_SERVO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO>`__ command.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>uint8_t</td>
   <td>System ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>uint8_t</td>
   <td>Component ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>command</strong></td>
   <td>uint16_t</td>
   <td>MAV_CMD_DO_SET_SERVO=183</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Servo instance number</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>PWM (normally between 1000 and 2000)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   </tbody>
   </table>

.. note::

   "ServoRelayEvent: Channel x is already in use" will be displayed if you try to move a servo that is configured for use as a control surface or for some other feature.  Check the :ref:`SERVOx_FUNCTION <copter:SERVO1_FUNCTION>` parameter value and ensure it is 0 ("Disabled"), 1 ("RCPassThru"), 22 ("SprayerPump"), 23 ("SprayerSpinner"), or between 51 ("RCIN1") and 66 ("RCIN16")

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter:

- module load message
- module load graph
- graph SERVO_OUTPUT_RAW.servo8_raw

+------------------------------------------------------+-----------------------------+
| Example MAVProxy/SITL Command                        | Description                 |
+======================================================+=============================+
| ``message COMMAND_LONG 0 0 183 0 8 1200 0 0 0 0 0``  | Move servo output 8 to 1200 |
+------------------------------------------------------+-----------------------------+

Cycle the Servo position with MAV_CMD_DO_REPEAT_SERVO
-----------------------------------------------------

The servo's position can be set to cycle (i.e. toggle) between a PWM value and :ref:`SERVOx_TRIM <copter:SERVO1_TRIM>` by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with fields set as specified for the `MAV_CMD_DO_REPEAT_SERVO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPEAT_SERVO>`__ command.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>uint8_t</td>
   <td>System ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>uint8_t</td>
   <td>Component ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>command</strong></td>
   <td>uint16_t</td>
   <td>MAV_CMD_DO_REPEAT_SERVO=184</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>Servo instance number</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>PWM (normally between 1000 and 2000)</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>Cycle count</td>
   </tr>
   <tr>
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>Cycle time (in milliseconds)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   </tbody>
   </table>

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter:

- module load message
- module load graph
- graph SERVO_OUTPUT_RAW.servo8_raw

+--------------------------------------------------------+-----------------------------------------------------------+
| Example MAVProxy/SITL Command                          | Description                                               |
+========================================================+===========================================================+
| ``message COMMAND_LONG 0 0 184 0 8 1200 3 1000 0 0 0`` | Cycle servo output 8 between 1200 and 1500 3 times at 1hz |
+--------------------------------------------------------+-----------------------------------------------------------+
