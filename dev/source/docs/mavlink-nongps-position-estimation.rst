.. _mavlink-nongps-position-estimation:

===========================
Non-GPS Position Estimation
===========================

This page explains how MAVLink can be used to send in external position and velocity estimates to ArduPilot's EKF allowing it to maintain a position estimate and thus control itself without a GPS

This is also called "External Navigation" although to be more precise it involves estimation rather than navigation or control.

.. note::

    The user wiki pages for :ref:`Non-GPS navigation is here <copter:common-non-gps-navigation-landing-page>` and :ref:`GPS/Non-GPS transitions is here <copter:common-non-gps-to-gps>`

Any of the following messages should be sent to the autopilot at 4hz or higher:

- `ODOMETRY <https://mavlink.io/en/messages/common.html#ODOMETRY>`__ (the preferred method)
- `VISION_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE>`__ and optionally `VISION_SPEED_ESTIMATE <https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE>`__
- `VISION_POSITION_DELTA <https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA>`__
- `VICON_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#VICON_POSITION_ESTIMATE>`__
- `ATT_POS_MOCAP <https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP>`__
- `GLOBAL_VISION_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE>`__ (not recommended)
- `GPS_INPUT <https://mavlink.io/en/messages/common.html#GPS_INPUT>`__ (not recommended)

ArduPilot's parameters should be setup as if a :ref:`ModalAI VOXL <copter:common-modalai-voxl>` is used which includes:

- Set :ref:`VISO_TYPE <VISO_TYPE>` = 3 (VOXL)
- Set :ref:`VISO_POS_X <VISO_POS_X>`, :ref:`VISO_POS_Y <VISO_POS_Y>`, :ref:`VISO_POS_Z <VISO_POS_Z>` to the camera's position on the vehicle
- Set :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 6 (ExternalNav)
- Set :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 6 (ExternalNav) or 0 (None)
- Set :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` = 6 (ExternalNav) or 1 (Baro)
- Set :ref:`EK3_SRC1_VELZ <EK3_SRC1_VELZ>` = 6 (ExternalNav) or 0 (None)
- Set :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 6 (ExternalNav) or 1 (Compass)

In addition it may be useful to setup :ref:`GPS/Non-GPS transitions <copter:common-non-gps-to-gps>` to allow switching between GPS and External Navigation

If no GPS is attached to the autopilot then the :ref:`EKF origin must be set <mavlink-get-set-home-and-origin>` before the EKF can start estimating its position.  If the vehicle will always be flown at the same location the `ahrs-set-origin.lua script <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/ahrs-set-origin.lua>`__ may be used

ODOMETRY message
----------------

The preferred method is to send an `ODOMETRY <https://mavlink.io/en/messages/common.html#ODOMETRY>`__ with the fields populated as described below:

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Field Name</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>time_usec</strong></td>
   <td>uint64_t</td>
   <td>Timestamp since system boot.  This does not need to be syncronised with the autopilot's time</td>
   </tr>
   <tr>
   <td><strong>frame_id</strong></td>
   <td>uint8_t</td>
   <td>MAV_FRAME_BODY_FRD (12) or MAV_FRAME_LOCAL_FRD (20)</td>
   </tr>
   <tr>
   <td><strong>child_frame_id</strong></td>
   <td>uint8_t</td>
   <td>MAV_FRAME_BODY_FRD (12) or MAV_FRAME_LOCAL_FRD (20)</td>
   </tr>
   <tr>
   <td><strong>x</strong></td>
   <td>float</td>
   <td>X position in meters</td>
   </tr>
   <tr>
   <td><strong>y</strong></td>
   <td>float</td>
   <td>Y position in meters</td>
   </tr>
   <tr>
   <td><strong>z</strong></td>
   <td>float</td>
   <td>Z position in meters (positive is down)</td>
   </tr>
   <tr>
   <td><strong>q</strong></td>
   <td>float[4]</td>
   <td>Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)</td>
   </tr>
   <tr>
   <td><strong>vx</strong></td>
   <td>float</td>
   <td>X axis linear speed in m/s</td>
   </tr>
   <tr>
   <td><strong>vy</strong></td>
   <td>float</td>
   <td>Y axis linear speed in m/s</td>
   </tr>
   <tr>
   <td><strong>vz</strong></td>
   <td>float</td>
   <td>Z axis linear speed in m/s (positive is down)</td>
   </tr>
   <tr>
   <td><strong>rollspeed</strong></td>
   <td>float</td>
   <td>Roll angular speed in rad/s (backwards is positive)</td>
   </tr>
   <tr>
   <td><strong>pitchspeed</strong></td>
   <td>float</td>
   <td>Pitch angular speed in rad/s (forward is positive)</td>
   </tr>
   <tr>
   <td><strong>yawspeed</strong></td>
   <td>float</td>
   <td>Yaw angular speed in rad/s (clockwise is positive)</td>
   </tr>
   <tr>
   <tr>
   <td><strong>pos_covariance</strong></td>
   <td>float[21]</td>
   <td>elements 0, 6 and 11 are x, y and z-axis position error. 15, 18, 20 are roll, pitch and yaw angle error. Ignored if NaN</td>
   </tr>
   <tr>
   <tr style="color: #c0c0c0">
   <td><strong>velocity_covariance</strong></td>
   <td>float[21]</td>
   <td>not used</td>
   </tr>
   <tr>
   <td><strong>reset_counter</strong></td>
   <td>uint8_t</td>
   <td>External estimator reset counter.  This should be incremented when the estimate resets position, velocity, attitude or angular speed</td>
   </tr>
   <tr>
   <tr style="color: #c0c0c0">
   <td><strong>estimator_type</strong></td>
   <td>uint8_t</td>
   <td>not used</td>
   </tr>
   <tr>
   <td><strong>quality</strong></td>
   <td>uint8_t</td>
   <td>quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality</td>
   </tr>
   </tbody>
   </table>

Six "pos_covariance" array elements are used to calculate the position error and angle error

- 0: x-axis position error (m)
- 6: y-axis position error (m)
- 11: z-axis position error (m)
- 15: roll angle error (rad)
- 18: pitch angle error (rad)
- 20: yaw angle error (rad)

For both position and angle error, the 3 elements are combined to create a single error which is used during EKF sensor fusion

.. image:: ../images/nongps-position-estimation-poserr.png
    :target: ../_images/nongps-position-estimation-poserr.png

In addition the position error is constrained to be between :ref:`VISO_POS_M_NSE <VISO_POS_M_NSE>` and 100 and the angle error between :ref:`VISO_YAW_M_NSE <VISO_YAW_M_NSE>` and 1.5

Messages with "quality" below the :ref:`VISO_QUAL_MIN <VISO_QUAL_MIN>` parameter value are ignored

OPTICAL_FLOW message
--------------------

An offboard optical flow sensor can send its horizontal flow sensor data to the autopilot using the `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`__ message

Users should set :ref:`FLOW_TYPE<FLOW_TYPE>` = 5 (MAVLink) and then perform the :ref:`Optical Flow Setup procedure <copter:common-optical-flow-sensor-setup>` before use

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Field Name</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>time_usec</strong></td>
   <td>uint64_t</td>
   <td>Timestamp since system boot.  This does not need to be syncronised with the autopilot's time</td>
   </tr>
   <tr>
   <td>sensor_id</td>
   <td>uint8_t</td>
   <td>not used</td>
   </tr>
   <tr>
   <td><strong>flow_x</strong></td>
   <td>int16_t</td>
   <td>Flow rate around X-axis</td>
   </tr>
   <tr>
   <td><strong>flow_x</strong></td>
   <td>int16_t</td>
   <td>Flow rate around Y-axis</td>
   </tr>
   <tr>
   <td>flow_comp_m_x</td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr>
   <td>flow_comp_m_y</td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr>
   <td><strong>quality</strong></td>
   <td>uint8_t</td>
   <td>Optical flow quality 0:bad, 255:best</td>
   </tr>
   <tr>
   <td>ground_distance</td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr>
   <td>flow_rate_x</td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr>
   <td>flow_rate_y</td>
   <td>float</td>
   <td>not used</td>
   </tr>
   </tbody>
   </table>

Please see this `issue 29131 <https://github.com/ArduPilot/ardupilot/issues/29131>`__ for improvements we should make to the handling of this message
