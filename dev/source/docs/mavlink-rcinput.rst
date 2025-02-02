.. _mavlink-rcinput:

==========================
RC Input (aka Pilot Input)
==========================

Traditionally the pilot's manual input via an RC transmitter is sent to the autopilot via a separate wireless link but it can alternatively be sent via MAVLink.  This page describes the messages that can be used and gives advice on some potential issues

For reference here are some user focused RC related wiki pages:

- :ref:`Joystick/Gamepad setup <copter:common-joystick>`
- :ref:`RC Input Channel Mapping (aka RCMAP) <copter:common-rcmap>`
- :ref:`Auxiliary Functions <copter:common-auxiliary-functions>`

The autopilot will ignore the RC input messages if the sender's system id does not match the autopilot's :ref:`SYSID_MYGCS <copter:SYSID_MYGCS>` and :ref:`SYSID_ENFORCE <copter:SYSID_ENFORCE>` = 1

RC input via MAVlink typically results in a laggy response caused by telemetry system lag and/or bandwidth limitations (these messages normally share bandwidth with other MAVLink messages passed between GCS and the vehicle)

The Pilot's RC inputs can be monitored by the GCS or companion computer via the `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`__ message

RC_CHANNELS_OVERRIDE
--------------------

The `RC_CHANNELS_OVERRIDE <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`__ message allows setting the PWM equivalent value for each channel.
This is normally a value between 1000 and 2000 but ArduPilot's RC input library uses the RCx_MIN, RCx_MAX and RCx_TRIM parameters to scale the input.  Note though that these parameters are normally set by the user during the RC Calibration stage.

If both a regular transmitter (e.g. Futaba, Spektrum, etc) which sends actualy PWM values and a MAVLink enabled RC input system (sending messages via one of the message listed on this page) then care should be taken to ensure the RCx_MIN/MAX ranges are consistent between the two systems.
If the MAVLink RC input stops, ArduPilot falls back to regular RC input within a few seconds.  This timeout is configurable using the :ref:`RC_OVERRIDE_TIME <copter:RC_OVERRIDE_TIME>` parameter.

Be careful not to change the flight mode inadvertently.  By default either channel 5 or channel 8 (depending upon the vehicle type) is used to set the flight mode but this can be disabled by setting :ref:`FLTMODE_CH <copter:FLTMODE_CH>` or :ref:`MODE_CH <rover:MODE_CH>` to 0

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
   <td><strong>chan1_raw</strong></td>
   <td>uint16_t</td>
   <td>RC channel 1 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX (e.g 65535) to ignore this field</td>
   </tr>
   <tr>
   <td><strong>chan2_raw</strong></td>
   <td>uint16_t</td>
   <td>RC channel 2 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td><strong>chan3_raw</strong></td>
   <td>uint16_t</td>
   <td>RC channel 3 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td><strong>chan4_raw</strong></td>
   <td>uint16_t</td>
   <td>RC channel 4 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td><strong>chan5_raw</strong></td>
   <td>uint16_t</td>
   <td>RC channel 5 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td><strong>chan6_raw</strong></td>
   <td>uint16_t</td>
   <td>RC channel 6 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td><strong>chan7_raw</strong></td>
   <td>uint16_t</td>
   <td>RC channel 7 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td><strong>chan8_raw</strong></td>
   <td>uint16_t</td>
   <td>RC channel 8 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan9_raw</td>
   <td>uint16_t</td>
   <td>RC channel 9 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan10_raw</td>
   <td>uint16_t</td>
   <td>RC channel 10 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan11_raw</td>
   <td>uint16_t</td>
   <td>RC channel 11 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan12_raw</td>
   <td>uint16_t</td>
   <td>RC channel 12 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan13_raw</td>
   <td>uint16_t</td>
   <td>RC channel 13 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan14_raw</td>
   <td>uint16_t</td>
   <td>RC channel 14 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan15_raw</td>
   <td>uint16_t</td>
   <td>RC channel 15 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan16_raw</td>
   <td>uint16_t</td>
   <td>RC channel 16 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan17_raw</td>
   <td>uint16_t</td>
   <td>RC channel 17 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   <tr>
   <td>chan18_raw</td>
   <td>uint16_t</td>
   <td>RC channel 18 value. Normally 1000 ~ 2000, 0 to release channel back to the RC radio, UINT16_MAX to ignore this field</td>
   </tr>
   </tbody>
   </table>

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test the message.  Before running these commands enter:

- module load message
- graph RC_CHANNELS.chan1_raw RC_CHANNELS.chan2_raw RC_CHANNELS.chan3_raw RC_CHANNELS.chan4_raw RC_CHANNELS.chan7_raw 

During simulator testing it may be useful to enable/disable the RC failsafe by setting :ref:`FS_THR_ENABLE <copter:FS_THR_ENABLE>` = 0 and/or simulate an RC failure by setting :ref:`SIM_RC_FAIL <copter:SIM_RC_FAIL>` = 1

+-------------------------------------------------------------------------------------+----------------------------------------------------------+
| Example MAVProxy/SITL Command                                                       | Description                                              |
+=====================================================================================+==========================================================+
| ``message RC_CHANNELS_OVERRIDE 0 0 1500 1500 1500 1500 0 0 0 0``                    | Set channels 1 ~ 4 to 1500                               |
+-------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message RC_CHANNELS_OVERRIDE 0 0 1800 1500 1500 1500 0 0 0 0``                    | Set ch1 (roll) to 1800 (e.g. roll right)                 |
+-------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message RC_CHANNELS_OVERRIDE 0 0 1500 1200 1500 1500 0 0 0 0``                    | Set ch2 (pitch) to 1200 (e.g. pitch forward)             |
+-------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message RC_CHANNELS_OVERRIDE 0 0 1500 1500 1800 1500 0 0 0 0``                    | Set ch3 (throttle) to 1800 (e.g. climb)                  |
+-------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message RC_CHANNELS_OVERRIDE 0 0 1500 1500 1500 1800 0 0 0 0``                    | Set ch4 (yaw) to 1800 (e.g. rotate clockwise)            |
+-------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message RC_CHANNELS_OVERRIDE 0 0 0 0 0 1800 0 0 0 0``                             | Set ch4 (yaw) to 1800, all other channels from normal RC |
+-------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message RC_CHANNELS_OVERRIDE 0 0 0 0 0 0 0 0 1800 0``                             | Set ch7 to 1800, all other channels from normal RC       |
+-------------------------------------------------------------------------------------+----------------------------------------------------------+
| ``message RC_CHANNELS_OVERRIDE 0 0 65535 65535 65535 65535 65535 65535 1800 65535`` | Set ch7 to 1800, all other channels unchanged            |
+-------------------------------------------------------------------------------------+----------------------------------------------------------+

MANUAL_CONTROL
--------------

The `MANUAL_CONTROL <https://mavlink.io/en/messages/common.html#MANUAL_CONTROL>`__ message allows sending roll, pitch, throttle and yaw values as normalised values between -1000 and +1000 and avoids any potential issues with channel mapping or PWM input ranges 

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>target</strong></td>
   <td>uint8_t</td>
   <td>System ID of flight controller (e.g. 1)</td>
   </tr>
   <tr>
   <td><strong>x</strong></td>
   <td>int16_t</td>
   <td>X-axis / Pitch, normally -1000 (backwards) ~ +1000 (forwards), INT16_MAX (32767) if this axis is invalid</td>
   </tr>
   <tr>
   <td><strong>y</strong></td>
   <td>int16_t</td>
   <td>Y-axis / Roll, normally -1000 (left) ~ +1000 (right), INT16_MAX if this axis is invalid</td>
   </tr>
   <tr>
   <td><strong>z</strong></td>
   <td>int16_t</td>
   <td>Z-axis / Thrust, normally 0 (down) ~ +1000 (up), INT16_MAX if this axis is invalid</td>
   </tr>
   <tr>
   <td><strong>r</strong></td>
   <td>int16_t</td>
   <td>R-axis / Yaw, normally -1000 (counter-clockwise) ~ +1000 (clockwise), INT16_MAX if this axis is invalid</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>buttons</strong></td>
   <td>uint16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>buttons2</td>
   <td>uint16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>enabled_extensions</td>
   <td>uint8_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>s</td>
   <td>int16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>t</td>
   <td>int16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>aux1</td>
   <td>int16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>aux2</td>
   <td>int16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>aux3</td>
   <td>int16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>aux4</td>
   <td>int16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>aux5</td>
   <td>int16_t</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>aux6</td>
   <td>int16_t</td>
   <td>not used</td>
   </tr>
   </tbody>
   </table>

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test the message.  Before running these commands enter:

- module load message

+----------------------------------------------------------+-------------------------------------+
| Example MAVProxy/SITL Command                            | Description                         |
+==========================================================+=====================================+
| ``message MANUAL_CONTROL 1 300 0 500 0 0``               | Pitch forward 30%, Throttle 50%     |
+----------------------------------------------------------+-------------------------------------+
| ``message MANUAL_CONTROL 1 0 300 500 0 0``               | Roll right 30%, Throttle 50%        |
+----------------------------------------------------------+-------------------------------------+
| ``message MANUAL_CONTROL 1 0 0 1000 0 0``                | Throttle 100%                       |
+----------------------------------------------------------+-------------------------------------+
| ``message MANUAL_CONTROL 1 0 0 500 100 0``               | Yaw right 10%, Throttle 50%         |
+----------------------------------------------------------+-------------------------------------+
| ``message MANUAL_CONTROL 1 32767 32767 32767 100 32767`` | Yaw right 10%, all others unchanged |
+----------------------------------------------------------+-------------------------------------+

Auxiliary Functions
-------------------

Auxiliary functions can be executed by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ or `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`__ message with the "command" field set to `MAV_CMD_DO_AUX_FUNCTION <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_AUX_FUNCTION>`__ (e.g. 218)

- "param1" should be set to the auxiliary function.  The full list of available functions can be found here on the user focused :ref:`Auxiliary Functions page <copter:common-auxiliary-functions>`
- "param2" should be set to 0:Switch Low (e.g deactivate function), 1:Switch Middle or 2:Switch Highs (e.g. activate function)

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test the message.  Before running these commands enter:

- module load message

+---------------------------------------------------+----------------------+
| Example MAVProxy/SITL Command                     | Description          |
+===================================================+======================+
| ``message COMMAND_LONG 0 0 218 0 46 2 0 0 0 0 0`` | RC Overrides Enable  |
+---------------------------------------------------+----------------------+
| ``message COMMAND_LONG 0 0 218 0 46 0 0 0 0 0 0`` | RC Overrides Disable |
+---------------------------------------------------+----------------------+
| ``message COMMAND_LONG 0 0 218 0 65 2 0 0 0 0 0`` | GPS Disable          |
+---------------------------------------------------+----------------------+
| ``message COMMAND_LONG 0 0 218 0 65 0 0 0 0 0 0`` | GPS Enable           |
+---------------------------------------------------+----------------------+

Effect on Failsafes
-------------------

It may be necessary to extend the RC failsafe timeout due to lag and loss of messages on the telemetry link.  This can be done by increasing the :ref:`RC_FS_TIMEOUT <copter:RC_FS_TIMEOUT>` parameter

Handling both Regular RC and MAVLink RC input
---------------------------------------------

If MAVLink based RC input is sent from the GCS this will generally override the regular RC input.  If the MAVLink RC input stops, ArduPilot falls back to regular RC input within a few seconds.  This timeout is configurable using the :ref:`RC_OVERRIDE_TIME <copter:RC_OVERRIDE_TIME>` parameter.

The "RC Override Enable" :ref:`auxiliary switch <copter:common-auxiliary-functions>` can be used to allow a pilot with a regular RC to forcibly disable any MAVLink RC input

The :ref:`RC_OPTIONS <copter:RC_OPTIONS>` parameter includes an "Ignore MAVLink Overrides" option that can be used to more permanently disable MAVLink RC input
