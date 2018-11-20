.. _ros-timesync:

=========================
Clock/Time Syncronisation
=========================

The flight controller and companion computer clocks can be syncronised using MAVLink's `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`__ and `TIMESYNC <https://mavlink.io/en/messages/common.html#TIMESYNC>`__ messages.  This is important so that each side can blend data from the other side into its EKF.

- `mavros's sys_time plugin <https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/sys_time.cpp>`__ implements the mavros/ROS side
- ArduPilot's `AP_RTC <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_RTC>`__ library implements the ArduPilot side

Configuration
-------------

Mavros's `apm_config.yaml <https://github.com/mavlink/mavros/blob/master/mavros/launch/apm_config.yaml>`__ file can be modified to configure the rate of the `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`__ and `TIMESYNC <https://mavlink.io/en/messages/common.html#TIMESYNC>`__ messages.  Below are the recommended settings.

::

    conn:
      timesync_rate: 10.0
      system_time_rate: 1.0

    time:
      timesync_mode: MAVLINK

The flight controller's time is set from the GPS by default but this can be modified to be set from the `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`__ command by setting RTC_TYPES = 1 (for "MAVLINK_SYSTEM_TIME")

Testing
-------

A MAVLink inspector (available using QGC or MissionPlanner) can be used to check that the system times are syncronised.  If using Mission Planner, press Ctrl-F and push the "MAVLink Inspector" button.

.. image:: ../images/ros-time-sync-testing.png
    :target: ../_images/ros-time-sync-testing.png
    :width: 450px
