.. _ros2-interfaces:

=================
ROS 2 Interfaces
=================

The purpose of this page is to document all the ROS 2 interfaces available in ArduPilot.
Content is organized by functional area the data is used.

Sensors
=======

ArduPilot exposes "sensor" type data over DDS, that usually corresponds to a physical component, or some filterered version of it.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Topic Name</th>
   <th>Topic Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td>ap/navsat/navsat0</td>
   <td>sensor_msgs/msg/NavSatFix</td> 
   <td>This is the reported GPS sensor position from the GPS subsystem.
   TODO It will include a frame ID as instance number</td>
   </tr>
   <tr>
   <td>ap/battery</td>
   <td>sensor_msgs/msg/BatteryState</td> 
   <td>This sends the battery state for each enabled battery.
   The battery instance is available in the frame ID.
   Each enabled battery will be have data published.
   </tr>
   <tr>
   <td>ap/imu/experimental/data</td>
   <td>sensor_msgs/msg/IMU</td>
   <td>This is the high rate IMU data from the IMU that is currently used in the EKF.</td>
   </tr>
   <tr>
   <td>ap/airspeed</td>
   <td>geometry_msgs/msg/Vector3Stamped</td>
   <td>This is the 3D airspeed estimate of the vehicle in body frame.</td>
   </tr>
   </tbody>
   </table>

Pose, Rates, and Coordinates
============================

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Topic Name</th>
   <th>Topic Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td>ap/gps_global_origin/filtered</td>
   <td>geographic_msgs/msg/GeoPointStamped</td> 
   <td>This is the filtered AHRS's inertial navigation origin. This is NOT the same ask the HOME location.
   </tr>
   <tr>
   <td>ap/twist/filtered</td>
   <td>geometry_msgs/msg/TwistStamped</td> 
   <td>This is the filtered AHRS's velocity in the local ENU frame relative to home.</td>
   </tr>
   <tr>
   <td>ap/pose/filtered</td>
   <td>geometry_msgs/msg/PoseStamped</td> 
   <td>This is the filtered AHRS's pose in the local ENU frame relative to home.</td>
   </tr>
   <tr>
   <td>ap/geopose/filtered</td>
   <td>geographic_msgs/msg/GeoPoseStamped</td> 
   <td>This is the filtered AHRS's pose (position+orientation) in global coordinates</td>
   </tr>
   <tr>
   <td>ap/tf_static</td>
   <td>tf2_msgs/msg/TFMessage</td> 
   <td>AP broadcasts its known static transforms on this topic.
   The transforms include the GPS sensor offsets relative to the vehicle origin.</td>
   </tr>
   </tbody>
   </table>

Time
====

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Topic Name</th>
   <th>Topic Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td>ap/time</td>
   <td>builtin_interface/msg/Time</td> 
   <td>This sends time from AP's real time clock.</td>
   </tr>
   <tr>
   <td>ap/clock</td>
   <td>rosgraph_msgs/msg/Clock</td> 
   <td>This sends time from AP's real time clock in a format suitable for aligning ROS time of a companion computer.</td>
   </tr>
   </tbody>
   </table>

Control
=======

The control interfaces are how a companion computer can command the autopilot to move
either it's control surfaces, motors, or tell the autopilot to control to a setpoint position, velocity, acceleration.
Control includes the high level navigation objectives.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
    <th>Topic Name</th>
    <th>Topic Type</th>
    <th>Description</th>
   </tr>
   <tr>
   <td>ap/joy</td>
   <td>sensor_msgs/msg/Joy</td> 
   <td>Receive joystick commands that override the RC input.</td>
   </tr>
   <tr>
   <td>ap/cmd_vel</td>
   <td>geometry_msgs/msg/TwistStamped</td> 
   <td>Receive REP-147 velocity commands.
    Some vehicles support body frame while others support earth frame. 
   </td>
   </tr>
   <tr>
   <td>ap/cmd_gps_pose</td>
   <td>ardupilot_msgs/msg/GlobalPosition</td> 
   <td>Receive REP-147 "High level goal".
    This message is called "GlobalPosition" in REP-147.
    Consult the source code to determine which fields are supported on which vehicles.
    </td>
   </tr>
   </tbody>
   </table>

For more information on the high level goal interface,
see the :ref:`Waypoint Goal Interface<ros2-waypoint-goal-interface>` 

Commands
========

ArduPilot exposes service servers for the following purposes:

* Pre-arm check
* Arming
* Changing modes
* Takeoff (copter only)

Odometry
========

Ardupilot may not have a good estimate of where it is relative to where it started moving.
A companion computer can interface with sensors that provide odometry, which is the computation of the dynamic transform
from the ``odom`` frame to the ``base_link`` frame. This data is fed into ArduPilot's ``AP_VisualOdom`` library.
This data may come from visual sensors, however other technologies such as radar and lidar can 
also provide odometry data. Regardless of the method of odometry, 
ArduPilot has a single ROS interface to receive it.

This data typically comes from a `TF2 Transfrom Tree <https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html>`_.
For more information on the coordinate systems used, review `ROS REP-105 <https://www.ros.org/reps/rep-0105.html>`_.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Topic Name</th>
   <th>Topic Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td>ap/tf</td>
   <td>tf2_msgs/msg/TFMessage</td> 
   <td>Receive the odometry dynamic transform on the normal tf2 dynamic transform topic.</td>
   </tr>
   <tr>
   </tbody>
   </table>

.. warning:: 
   Only the dynamic transformations on ``/ap/tf`` that have parent_frame ``odom`` and child_frame ``base_link`` are fed into ``AP_VisualOdom``. 
   Other frame configurations will be gracefully ignored, so feel free to populate this topic with other transforms if that's convenient.

For more information on how to setup ArduPilot with an external odometry source, see the :ref:`cartographer SLAM example<ros2-cartographer-slam>`.

Configuring Interfaces at Compile Time
======================================

ArduPilot strives to only consume the resources it needs.
The DDS interface is no exception.

Every topic and service can be individually enabled or disabled
at compile time. See 
:ref:`common-oem-customizations`.

Refer to the `AP_DDS_Config.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/AP_DDS_config.h>`_
file on what is exposed.

Additionally, publish rates form ArduPilot also all indivually configurable at compile time.

When deploying ArduPilot on a resource constrained autopilot, developers
can disable interfaces they do not use and tune data rates to only
what their applications need.

Adding New Interfaces
=====================

ArduPilot's DDS interface is designed to be extensible.
Interfaces for pub/sub and services are easily added.

If the interfaces are generic, consider contributing them to ArduPilot.
Custom application-specific interfaces are also easy to add and maintain
on private forks of ArduPilot.

Interfaces that use custom messages that aren't already used commonly in
the ROS 2 ecosystem are typically added with an ``experimental`` designator.


ABI Stability Guarantees
========================

ArduPilot's DDS interface is intended to be ABI stable within an ArduPilot minor release.
Common interfaces such as ``NavSatFix`` are unlikely to change, however experimental interfaces
such as ``IMU`` may change, and are denoted with the ``experimental`` topic prefix.
In extenuating circumstances, non-experimental topics may require bugfixes that
change behavior, however message definitions will be kept compatible.

Because the ROS way of updating messages is to not change messages
within a ROS release, many tools do not cope well with messages.
Different versions of the message between an autopilot and companion computer
can lead to receiving junk data without any errors, or cryptic serialization
errors. Thus, ArduPilot will do it's best to avoid changing messages.
If breaking changes are required, the ArduPilot release notes will make that clear.

Developers should not expect ABI stability on ``exerimental`` interfaces.

Because ArduPilot does not follow the same release timeline as ROS 2, and
the development team for the ROS interface is still in its early stages,
ArduPilot does not yet support a stable ABI across multiple ROS distributions.
The current ROS version supported is ``humble``.
