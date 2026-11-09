.. _mavlink-other-commands:

==================================
MAVLink Interface's Other Commands
==================================

This article lists other commonly used MAVLink commands that do not directly affect how a vehicle moves and that are not covered by other pages.

MAV_CMDs
=========

These MAV_CMDs should be packaged within a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ message.

- :ref:`MAV_CMD_DO_AUTOTUNE_ENABLE <plane:mav_cmd_do_autotune_enable>` (Plane only)
- :ref:`MAV_CMD_DO_DIGICAM_CONFIGURE <copter:mav_cmd_do_digicam_configure>`
- :ref:`MAV_CMD_DO_DIGICAM_CONTROL <copter:mav_cmd_do_digicam_control>`
- :ref:`MAV_CMD_DO_FENCE_ENABLE <copter:mav_cmd_do_fence_enable>`
- `MAV_CMD_DO_GO_AROUND <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GO_AROUND>`__ (Plane only)
- `MAV_CMD_DO_LAND_START <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START>`__ (Plane only)
- :ref:`MAV_CMD_DO_GRIPPER <copter:mav_cmd_do_gripper>`
- `MAV_CMD_DO_MOTOR_TEST <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_MOTOR_TEST>`__
- :ref:`MAV_CMD_DO_PARACHUTE <copter:mav_cmd_do_parachute>`
- :ref:`MAV_CMD_DO_REPEAT_SERVO <copter:mav_cmd_do_repeat_servo>`
- :ref:`MAV_CMD_DO_REPEAT_RELAY <copter:mav_cmd_do_repeat_relay>`
- `MAV_CMD_DO_SEND_BANNER <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SEND_BANNER>`__
- :ref:`MAV_CMD_DO_SET_RELAY <copter:mav_cmd_do_set_relay>`
- :ref:`MAV_CMD_DO_SET_SERVO <copter:mav_cmd_do_set_servo>`
- `MAV_CMD_DO_MOUNT_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE>`__
- :ref:`MAV_CMD_DO_MOUNT_CONTROL <copter:mav_cmd_do_mount_control>`
- :ref:`MAV_CMD_MISSION_START <copter:mav_cmd_mission_start>`
- `MAV_CMD_PREFLIGHT_CALIBRATION <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_CALIBRATION>`__
- `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS>`__
- `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN>`__
- `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES>`__
- `MAV_CMD_START_RX_PAIR <https://mavlink.io/en/messages/common.html#MAV_CMD_START_RX_PAIR>`__ - starts receiver pairing
- `MAV_CMD_DO_START_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_START_MAG_CAL>`__
- `MAV_CMD_DO_ACCEPT_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_ACCEPT_MAG_CAL>`__
- `MAV_CMD_DO_CANCEL_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_CANCEL_MAG_CAL>`__

.. _mavlink-other-commands-power-off:

Remote Power Off
================

In firmware 4.8 and later, an autopilot whose hwdef.dat defines a power control pin can switch its own power off when it receives `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN>`__ with param1 = 2 (shutdown). Autopilots without the pin reject the shutdown request. The board's hwdef.dat must set the GPIO number of the output pin that controls the power, and the pin level that turns the power off:

::

    define AP_POWER_CONTROL_GPIO 12
    define AP_POWER_CONTROL_GPIO_VALUE_OFF 1

The command is refused while the vehicle is armed. When it is accepted, the autopilot:

- stops its servo and motor outputs and forces the safety state on
- saves any parameter changes not yet written to storage
- plays the power off tones
- sends a "Vehicle powering off" message to the ground station
- sends a `MAV_CMD_POWER_OFF_INITIATED <https://mavlink.io/en/messages/common.html#MAV_CMD_POWER_OFF_INITIATED>`__ command to the other MAVLink components with the same system ID (for example a companion computer), so they can shut down cleanly
- refuses to arm, even if arming is forced, with a "Power off pending" pre-arm failure

5 seconds later it sets the power control pin to its off level. The delay can be changed with ``define AP_POWEROFF_DELAY_MS`` in the hwdef.dat. A delay of 0 sets the pin straight away, without the steps above.

SITL defines a power control pin, and exits when sent the shutdown command, so this can be tested in simulation.

Other commands
==============

Below are other commands that will be handled by Copter

- `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`__
- `AUTOPILOT_VERSION_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#AUTOPILOT_VERSION_REQUEST>`__
- `COMMAND_ACK <https://mavlink.io/en/messages/common.html#COMMAND_ACK>`__
- `FENCE_POINT <https://mavlink.io/en/messages/ardupilotmega.html#FENCE_POINT>`__
- `FENCE_FETCH_POINT <https://mavlink.io/en/messages/ardupilotmega.html#FENCE_FETCH_POINT>`__
- `GIMBAL_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_REPORT>`__
- `GPS_INJECT_DATA <https://mavlink.io/en/messages/common.html#GPS_INJECT_DATA>`__
- `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`__
- `HIL_STATE <https://mavlink.io/en/messages/common.html#HIL_STATE>`__
- `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`__
- `LED_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#LED_CONTROL>`__
- `LOG_ERASE <https://mavlink.io/en/messages/common.html#LOG_ERASE>`__
- `LOG_REQUEST_DATA <https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA>`__
- `LOG_REQUEST_END <https://mavlink.io/en/messages/common.html#LOG_REQUEST_END>`__
- `LOG_REQUEST_LIST <https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST>`__
- `RADIO <https://mavlink.io/en/messages/ardupilotmega.html#RADIO>`__
- `RADIO_STATUS <https://mavlink.io/en/messages/common.html#RADIO_STATUS>`__
- `RALLY_FETCH_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_FETCH_POINT>`__
- `RALLY_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_POINT>`__
- `RC_CHANNELS_OVERRIDE <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`__
- `REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`__
- `REMOTE_LOG_BLOCK_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_BLOCK_STATUS>`__
- `SERIAL_CONTROL <https://mavlink.io/en/messages/common.html#SERIAL_CONTROL>`__
- `TERRAIN_DATA <https://mavlink.io/en/messages/common.html#TERRAIN_DATA>`__
- `TERRAIN_CHECK <https://mavlink.io/en/messages/common.html#TERRAIN_CHECK>`__
