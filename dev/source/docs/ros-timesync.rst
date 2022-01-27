.. _ros-timesync:

==========================
Clock/Time Synchronisation
==========================

The flight controller and companion computer clocks can be synchronised using MAVLink's `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`__ and `TIMESYNC <https://mavlink.io/en/messages/common.html#TIMESYNC>`__ messages.  This is important so that each side can blend data from the other side into its EKF.

- `mavros's sys_time plugin <https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/sys_time.cpp>`__ implements the mavros/ROS side
- ArduPilot's `AP_RTC <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_RTC>`__ library implements the ArduPilot side

Mavros Configuration
--------------------

Mavros's `apm_config.yaml <https://github.com/mavlink/mavros/blob/master/mavros/launch/apm_config.yaml>`__ file can be modified to configure the rate of the `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`__ and `TIMESYNC <https://mavlink.io/en/messages/common.html#TIMESYNC>`__ messages.  Below are the recommended settings.

::

    conn:
      timesync_rate: 10.0
      system_time_rate: 1.0

    time:
      timesync_mode: MAVLINK

ArduPilot Configuration
-----------------------

The flight controller's time is set from the GPS by default but can be set from one of three sources(GPS, MAVLINK, onboard HW clock) using the :ref:`BRD_RTC_TYPES <BRD_RTC_TYPES>` parameter as a bit mask for the time syncronisation source. When using mavros set ``BRD_RTC_TYPES`` = 2 to only allow the "MAVLINK_SYSTEM_TIME" source which uses the `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`__ MAVLINK message.

If the ``RTT too high for timesync`` message is shown in the Mavros console (most commonly when using ArduRover), the :ref:`SCHED_LOOP_RATE <SCHED_LOOP_RATE>` parameter should be increased. This is because Mavros ignores TIMESYNC messages if the round trip time is more than 10ms.

Testing
-------

A MAVLink inspector (available using QGC or MissionPlanner) can be used to check that the system times are synchronised.  If using Mission Planner, press Ctrl-F and push the "MAVLink Inspector" button.

.. image:: ../images/ros-time-sync-testing.png
    :target: ../_images/ros-time-sync-testing.png
    :width: 450px
