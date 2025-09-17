.. _buttons:

====================
Button Inputs in Sub
====================

Unlike other vehicles, Sub has its own specialized button functionality. Other vehicles include hardware buttons as described in the :ref:`common-buttons` hardware section.

Instead, Sub uses "joystick" buttons associated with the GCS or GCS attached Joystick. The GCS sends frequent button state updates via the `MAVLink Manual Control protocol <https://mavlink.io/en/services/manual_control.html>`__, which includes both motion-axis commands and button states.

Up to 32 buttons can be configured with a host of functions. In addition, by assigning a "shift" button function, each button can execute a "shifted" function, so up to 63 functions can be implemented.

.. note:: See :ref:`sub-hardware` for the default assignments of button functions.

Each button has two parameters:

* ``BTNx_FUNCTION``
* ``BTNx_SFUNCTION``

The possible values for the function or shifted function are:

+---------+--------------------------+-------------------------------------------------------------------------------+
|**Value**|**Meaning**               |**Description**                                                                |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 0       | Disabled                 | Do nothing (button unused by the autopilot).                                  |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 1       | shift                    | Modifier to swap other buttons to the "shifted" functions.                    |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 2       | arm_toggle               | Swap the autopilot's armed/disarmed state.                                    |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 3       | arm                      | Arm the autopilot.                                                            |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 4       | disarm                   | Disarm the autopilot.                                                         |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 5       | mode_manual              | Change to MANUAL :ref:`flight mode <modes>`.                                  |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 6       | mode_stabilize           | Change to STABILIZE :ref:`flight mode <modes>`.                               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 7       | mode_depth_hold          | Change to ALT_HOLD :ref:`flight mode <modes>`.                                |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 8       | mode_poshold             | Change to POSHOLD :ref:`flight mode <modes>`.                                 |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 9       | mode_auto                | Change to AUTO :ref:`flight mode <modes>`.                                    |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 10      | mode_circle              | Change to CIRCLE :ref:`flight mode <modes>`.                                  |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 11      | mode_guided              | Change to GUIDED :ref:`flight mode <modes>`.                                  |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 12      | mode_acro                | Change to ACRO :ref:`flight mode <modes>`.                                    |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 13      | mode_surftrak            | Change to SURFTRAK :ref:`flight mode <modes>`.                                |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 21      | mount_center             | Move the camera mount to its center orientation.                              |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 22      | mount_tilt_up            | Tilt (pitch) up the camera mount.                                             |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 23      | mount_tilt_down          | Tilt (pitch) down the camera mount.                                           |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 24      | camera_trigger           | Trigger the camera shutter (**NOT IMPLEMENTED**).                             |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 25      | camera_source_toggle     | Toggle between available camera sources (with connected video switch).        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 26      | mount_pan_right          | Pan (yaw) the camera mount to the right.                                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 27      | mount_pan_left           | Pan (yaw) the camera mount to the left.                                       |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 31      | lights1_cycle            | Step lights1 (once) brighter/dimmer as part of a cycle between the limits.    |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 32      | lights1_brighter         | Step lights1 brighter by the configured step size, up to the output limit.    |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 33      | lights1_dimmer           | Step lights1 dimmer by the configured step size, up to the output limit.      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 34      | lights2_cycle            | Step lights2 (once) brighter/dimmer as part of a cycle between the limits.    |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 35      | lights2_brighter         | Step lights2 brighter by the configured step size, up to the output limit.    |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 36      | lights2_dimmer           | Step lights2 dimmer by the configured step size, up to the output limit.      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 41      | gain_toggle              | Toggle pilot gain between 50% and 100%.                                       |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 42      | gain_inc                 | Increment pilot gain by the configured joystick gain step size.               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 43      | gain_dec                 | Decrement pilot gain by the configured joystick gain step size.               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 44      | trim_roll_inc            | Increase the roll trim level.                                                 |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 45      | trim_roll_dec            | Decrease the roll trim level.                                                 |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 46      | trim_pitch_inc           | Increase the pitch trim level.                                                |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 47      | trim_pitch_dec           | Decrease the pitch trim level.                                                |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 48      | input_hold_set           | Toggle holding current joystick axis inputs (cruise control on/off).          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 49      | roll_pitch_toggle        | Toggle between roll/pitch and forward/lateral control on joystick input.      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 51      | relay_1_on               | Set the :ref:`RELAY1_PIN <RELAY1_PIN>` output to ON.                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 52      | relay_1_off              | Set the :ref:`RELAY1_PIN <RELAY1_PIN>` output to OFF.                         |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 53      | relay_1_toggle           | Toggle the :ref:`RELAY1_PIN <RELAY1_PIN>` output between ON and OFF.          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 54      | relay_2_on               | Set the :ref:`RELAY2_PIN <RELAY2_PIN>` output to ON.                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 55      | relay_2_off              | Set the :ref:`RELAY2_PIN <RELAY2_PIN>` output to OFF.                         |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 56      | relay_2_toggle           | Toggle the :ref:`RELAY2_PIN <RELAY2_PIN>` output between ON and OFF.          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 57      | relay_3_on               | Set the :ref:`RELAY3_PIN <RELAY3_PIN>` output to ON.                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 58      | relay_3_off              | Set the :ref:`RELAY3_PIN <RELAY3_PIN>` output to OFF.                         |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 59      | relay_3_toggle           | Toggle the :ref:`RELAY3_PIN <RELAY3_PIN>` output between ON and OFF.          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 61      | actuator_1_inc           | Increase actuator 1 (RC9/AUX1 on old firmware).                               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 62      | actuator_1_dec           | Decrease actuator 1 (RC9/AUX1 on old firmware).                               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 63      | actuator_1_min           | Set actuator 1 (RC9/AUX1 on old firmware) to its minimum value.               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 64      | actuator_1_max           | Set actuator 1 (RC9/AUX1 on old firmware) to its maximum value.               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 65      | actuator_1_center        | Set actuator 1 (RC9/AUX1 on old firmware) to its center value.                |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 66      | actuator_2_inc           | Increase actuator 2 (RC10/AUX2 on old firmware).                              |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 67      | actuator_2_dec           | Decrease actuator 2 (RC10/AUX2 on old firmware).                              |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 68      | actuator_2_min           | Set actuator 2 (RC10/AUX2 on old firmware) to its minimum value.              |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 69      | actuator_2_max           | Set actuator 2 (RC10/AUX2 on old firmware) to its maximum value.              |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 70      | actuator_2_center        | Set actuator 2 (RC10/AUX2 on old firmware) to its center value.               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 71      | actuator_3_inc           | Increase actuator 3 (RC11/AUX3 on old firmware).                              |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 72      | actuator_3_dec           | Decrease actuator 3 (RC11/AUX3 on old firmware).                              |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 73      | actuator_3_min           | Set actuator 3 (RC11/AUX3 on old firmware) to its minimum value.              |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 74      | actuator_3_max           | Set actuator 3 (RC11/AUX3on old firmware) to its maximum value.               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 75      | actuator_3_center        | Set actuator 3 (RC11/AUX3 on old firmware) to its center value.               |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 76      | actuator_1_min_momentary | Set actuator 1 to its minimum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 77      | actuator_1_max_momentary | Set actuator 1 to its maximum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 78      | actuator_1_min_toggle    | Toggle actuator 1 between its minimum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 79      | actuator_1_max_toggle    | Toggle actuator 1 between its maximum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 80      | actuator_2_min_momentary | Set actuator 2 to its minimum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 81      | actuator_2_max_momentary | Set actuator 2 to its maximum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 82      | actuator_2_min_toggle    | Toggle actuator 2 between its minimum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 83      | actuator_2_max_toggle    | Toggle actuator 2 between its maximum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 84      | actuator_3_min_momentary | Set actuator 3 to its minimum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 85      | actuator_3_max_momentary | Set actuator 3 to its maximum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 86      | actuator_3_min_toggle    | Toggle actuator 3 between its minimum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 87      | actuator_3_max_toggle    | Toggle actuator 3 between its maximum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 91      | custom_1                 | Activate custom functionality 1 (if implemented).                             |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 92      | custom_2                 | Activate custom functionality 2 (if implemented).                             |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 93      | custom_3                 | Activate custom functionality 3 (if implemented).                             |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 94      | custom_4                 | Activate custom functionality 4 (if implemented).                             |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 95      | custom_5                 | Activate custom functionality 5 (if implemented).                             |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 96      | custom_6                 | Activate custom functionality 6 (if implemented).                             |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 101     | relay_4_on               | Set the :ref:`RELAY4_PIN <RELAY4_PIN>` output to ON.                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 102     | relay_4_off              | Set the :ref:`RELAY4_PIN <RELAY4_PIN>` output to OFF.                         |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 103     | relay_4_toggle           | Toggle the :ref:`RELAY4_PIN <RELAY4_PIN>` output between ON and OFF.          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 104     | relay_1_momentary        | Set the :ref:`RELAY1_PIN <RELAY1_PIN>` output to ON, while the button is held.|
+---------+--------------------------+-------------------------------------------------------------------------------+
| 105     | relay_2_momentary        | Set the :ref:`RELAY2_PIN <RELAY2_PIN>` output to ON, while the button is held.|
+---------+--------------------------+-------------------------------------------------------------------------------+
| 106     | relay_3_momentary        | Set the :ref:`RELAY3_PIN <RELAY3_PIN>` output to ON, while the button is held.|
+---------+--------------------------+-------------------------------------------------------------------------------+
| 107     | relay_4_momentary        | Set the :ref:`RELAY4_PIN <RELAY4_PIN>` output to ON, while the button is held.|
+---------+--------------------------+-------------------------------------------------------------------------------+
| 108     | script_1                 | Register :ref:`Lua script <common-lua-scripts>` input 1 as pressed/held.      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 109     | script_2                 | Register :ref:`Lua script <common-lua-scripts>` input 2 as pressed/held.      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 110     | script_3                 | Register :ref:`Lua script <common-lua-scripts>` input 3 as pressed/held.      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 111     | script_4                 | Register :ref:`Lua script <common-lua-scripts>` input 4 as pressed/held.      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 112     | actuator_4_min           | Set actuator 4 to its minimum value.                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 113     | actuator_4_max           | Set actuator 4 to its maximum value.                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 114     | actuator_4_center        | Set actuator 4 to its center value.                                           |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 115     | actuator_4_inc           | Increase actuator 4.                                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 116     | actuator_4_dec           | Decrease actuator 4.                                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 117     | actuator_4_min_momentary | Set actuator 4 to its minimum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 118     | actuator_4_max_momentary | Set actuator 4 to its maximum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 119     | actuator_4_min_toggle    | Toggle actuator 4 between its minimum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 120     | actuator_4_max_toggle    | Toggle actuator 4 between its maximum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 121     | actuator_5_min           | Set actuator 5 to its minimum value.                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 122     | actuator_5_max           | Set actuator 5 to its maximum value.                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 123     | actuator_5_center        | Set actuator 5 to its center value.                                           |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 124     | actuator_5_inc           | Increase actuator 5.                                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 125     | actuator_5_dec           | Decrease actuator 5.                                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 126     | actuator_5_min_momentary | Set actuator 5 to its minimum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 127     | actuator_5_max_momentary | Set actuator 5 to its maximum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 128     | actuator_5_min_toggle    | Toggle actuator 5 between its minimum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 129     | actuator_5_max_toggle    | Toggle actuator 5 between its maximum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 130     | actuator_6_min           | Set actuator 6 to its minimum value.                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 131     | actuator_6_max           | Set actuator 6 to its maximum value.                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 132     | actuator_6_center        | Set actuator 6 to its center value.                                           |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 133     | actuator_6_inc           | Increase actuator 6.                                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 134     | actuator_6_dec           | Decrease actuator 6.                                                          |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 135     | actuator_6_min_momentary | Set actuator 6 to its minimum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 136     | actuator_6_max_momentary | Set actuator 6 to its maximum value while held, then return to center.        |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 137     | actuator_6_min_toggle    | Toggle actuator 6 between its minimum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
| 138     | actuator_6_max_toggle    | Toggle actuator 6 between its maximum and center values.                      |
+---------+--------------------------+-------------------------------------------------------------------------------+
