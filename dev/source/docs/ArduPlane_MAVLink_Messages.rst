.. _ArduPlane_MAVLink_messages:

================
MAVLink Messages
================


The `MAVLink <https://mavlink.io/en/>`_ protocol supports a variety of features and functionalities, but not all `messages <https://mavlink.io/en/messages/>`_ or `commands <https://mavlink.io/en/services/command.html>`_ are implemented by the ArduPilot ecosystem, or relevant to a particular autopilot firmware.

This page is auto-generated from analysing the ArduPlane source code, and provides an indication of which messages (and commands) are handled by, requestable from, and sent from the firmware. A message being handled does not guarantee full support, but at least shows that the autopilot is aware it exists, and will try to do something meaningful with it.

Known :ref:`unsupported messages <ArduPlane_mavlink_missing_messages>` (and commands) are shown at the end.

The autopilot includes a set of :ref:`ArduPlane_mavlink_stream_groups` for convenience, which allow configuring the stream rates of groups of requestable messages by setting parameter values. It is also possible to manually request messages, and request individual messages be streamed at a specified rate. 


.. _ArduPlane_mavlink_incoming_messages:

Incoming Messages
=================

Messages the autopilot handles when received.

.. csv-table::
  :header: MAVLink Message, Code Source, MAVLink Dialect


  `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, common
  `ATT_POS_MOCAP <https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `AUTOPILOT_VERSION_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#AUTOPILOT_VERSION_REQUEST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `CANFD_FRAME <https://mavlink.io/en/messages/common.html#CANFD_FRAME>`_, `AP_CANManager/AP_CANManager.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CANManager/AP_CANManager.cpp>`_, common
  `CAN_FILTER_MODIFY <https://mavlink.io/en/messages/common.html#CAN_FILTER_MODIFY>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `CAN_FRAME <https://mavlink.io/en/messages/common.html#CAN_FRAME>`_, `AP_CANManager/AP_CANManager.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CANManager/AP_CANManager.cpp>`_, common
  `COMMAND_ACK <https://mavlink.io/en/messages/common.html#COMMAND_ACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `DATA96 <https://mavlink.io/en/messages/ardupilotmega.html#DATA96>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `DEVICE_OP_READ <https://mavlink.io/en/messages/ardupilotmega.html#DEVICE_OP_READ>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `DEVICE_OP_WRITE <https://mavlink.io/en/messages/ardupilotmega.html#DEVICE_OP_WRITE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `DIGICAM_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#DIGICAM_CONTROL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `EFI_STATUS <https://mavlink.io/en/messages/common.html#EFI_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `FENCE_FETCH_POINT <https://mavlink.io/en/messages/ardupilotmega.html#FENCE_FETCH_POINT>`_, `AC_Fence/AC_PolyFence_loader.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AC_Fence/AC_PolyFence_loader.cpp>`_, ardupilotmega
  `FENCE_POINT <https://mavlink.io/en/messages/ardupilotmega.html#FENCE_POINT>`_, `AC_Fence/AC_PolyFence_loader.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AC_Fence/AC_PolyFence_loader.cpp>`_, ardupilotmega
  `FILE_TRANSFER_PROTOCOL <https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `FOLLOW_TARGET <https://mavlink.io/en/messages/common.html#FOLLOW_TARGET>`_, `AP_Follow/AP_Follow.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Follow/AP_Follow.cpp>`_, common
  `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  `GIMBAL_DEVICE_INFORMATION <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  `GIMBAL_MANAGER_SET_ATTITUDE <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_ATTITUDE>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  `GIMBAL_MANAGER_SET_PITCHYAW <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  `GIMBAL_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_REPORT>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, ardupilotmega
  `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`_, `AP_Follow/AP_Follow.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Follow/AP_Follow.cpp>`_, common
  `GLOBAL_VISION_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GOPRO_HEARTBEAT <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_HEARTBEAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `GPS_INJECT_DATA <https://mavlink.io/en/messages/common.html#GPS_INJECT_DATA>`_, `AP_GPS/AP_GPS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS.cpp>`_, common
  `GPS_INPUT <https://mavlink.io/en/messages/common.html#GPS_INPUT>`_, `AP_GPS/AP_GPS_MAV.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS_MAV.cpp>`_, common
  `GPS_RTCM_DATA <https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA>`_, `AP_GPS/AP_GPS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS.cpp>`_, common
  `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HIL_GPS <https://mavlink.io/en/messages/common.html#HIL_GPS>`_, `AP_GPS/AP_GPS_MAV.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS_MAV.cpp>`_, common
  `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `LED_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#LED_CONTROL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `LOG_ERASE <https://mavlink.io/en/messages/common.html#LOG_ERASE>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  `LOG_REQUEST_DATA <https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  `LOG_REQUEST_END <https://mavlink.io/en/messages/common.html#LOG_REQUEST_END>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  `LOG_REQUEST_LIST <https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  `MANUAL_CONTROL <https://mavlink.io/en/messages/common.html#MANUAL_CONTROL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_ACK <https://mavlink.io/en/messages/common.html#MISSION_ACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_CLEAR_ALL <https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_ITEM <https://mavlink.io/en/messages/common.html#MISSION_ITEM>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_ITEM_INT <https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_REQUEST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_REQUEST_INT <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_REQUEST_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_SET_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_WRITE_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MOUNT_CONFIGURE <https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_CONFIGURE>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, ardupilotmega
  `MOUNT_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_CONTROL>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, ardupilotmega
  `NAMED_VALUE_FLOAT <https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `OBSTACLE_DISTANCE <https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `OBSTACLE_DISTANCE_3D <https://mavlink.io/en/messages/ardupilotmega.html#OBSTACLE_DISTANCE_3D>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `ODOMETRY <https://mavlink.io/en/messages/common.html#ODOMETRY>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `OPEN_DRONE_ID_ARM_STATUS <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_ARM_STATUS>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  `OPEN_DRONE_ID_BASIC_ID <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  `OPEN_DRONE_ID_OPERATOR_ID <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_OPERATOR_ID>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  `OPEN_DRONE_ID_SELF_ID <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SELF_ID>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  `OPEN_DRONE_ID_SYSTEM <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SYSTEM>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  `OPEN_DRONE_ID_SYSTEM_UPDATE <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SYSTEM_UPDATE>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `OSD_PARAM_CONFIG <https://mavlink.io/en/messages/ardupilotmega.html#OSD_PARAM_CONFIG>`_, `AP_OSD/AP_OSD.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OSD/AP_OSD.cpp>`_, ardupilotmega
  `OSD_PARAM_SHOW_CONFIG <https://mavlink.io/en/messages/ardupilotmega.html#OSD_PARAM_SHOW_CONFIG>`_, `AP_OSD/AP_OSD.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OSD/AP_OSD.cpp>`_, ardupilotmega
  `PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `PLAY_TUNE <https://mavlink.io/en/messages/common.html#PLAY_TUNE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RADIO <https://mavlink.io/en/messages/ardupilotmega.html#RADIO>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `RADIO_RC_CHANNELS <https://mavlink.io/en/messages/development.html#RADIO_RC_CHANNELS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, development
  `RADIO_STATUS <https://mavlink.io/en/messages/common.html#RADIO_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RALLY_FETCH_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_FETCH_POINT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `RALLY_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_POINT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `RC_CHANNELS_OVERRIDE <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `REMOTE_LOG_BLOCK_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_BLOCK_STATUS>`_, `AP_Logger/AP_Logger.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger.cpp>`_, ardupilotmega
  `REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SECURE_COMMAND <https://mavlink.io/en/messages/ardupilotmega.html#SECURE_COMMAND>`_, `AP_CheckFirmware/AP_CheckFirmware_secure_command.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CheckFirmware/AP_CheckFirmware_secure_command.cpp>`_, ardupilotmega
  `SECURE_COMMAND_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#SECURE_COMMAND_REPLY>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `SERIAL_CONTROL <https://mavlink.io/en/messages/common.html#SERIAL_CONTROL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SETUP_SIGNING <https://mavlink.io/en/messages/common.html#SETUP_SIGNING>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SET_ATTITUDE_TARGET <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `SET_GPS_GLOBAL_ORIGIN <https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SET_MODE <https://mavlink.io/en/messages/common.html#SET_MODE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `SET_POSITION_TARGET_LOCAL_NED <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `STATUSTEXT <https://mavlink.io/en/messages/common.html#STATUSTEXT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `TERRAIN_CHECK <https://mavlink.io/en/messages/common.html#TERRAIN_CHECK>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `TERRAIN_DATA <https://mavlink.io/en/messages/common.html#TERRAIN_DATA>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `TIMESYNC <https://mavlink.io/en/messages/common.html#TIMESYNC>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `UAVIONIX_ADSB_OUT_CFG <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CFG>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, uAvionix
  `UAVIONIX_ADSB_OUT_CONTROL <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CONTROL>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, uAvionix
  `UAVIONIX_ADSB_OUT_DYNAMIC <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_DYNAMIC>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, uAvionix
  `UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, uAvionix
  `VICON_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#VICON_POSITION_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `VISION_POSITION_DELTA <https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `VISION_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `VISION_SPEED_ESTIMATE <https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common

.. _ArduPlane_mavlink_incoming_commands:

Incoming Commands
=================

Commands the autopilot handles when received.

.. csv-table::
  :header: MAVLink Message, Code Source, MAVLink Dialect


  `MAV_CMD_ACCELCAL_VEHICLE_POS <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_ACCELCAL_VEHICLE_POS>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, ardupilotmega
  `MAV_CMD_AIRFRAME_CONFIGURATION <https://mavlink.io/en/messages/common.html#MAV_CMD_AIRFRAME_CONFIGURATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_BATTERY_RESET <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_BATTERY_RESET>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, ardupilotmega
  `MAV_CMD_CAMERA_STOP_TRACKING <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_CAMERA_TRACK_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_CAMERA_TRACK_RECTANGLE <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_CAN_FORWARD <https://mavlink.io/en/messages/common.html#MAV_CMD_CAN_FORWARD>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_COMPONENT_ARM_DISARM <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_CONDITION_DELAY <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_DELAY>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_CONDITION_DISTANCE <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_DISTANCE>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_CONDITION_YAW <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_YAW>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_CONTROL_HIGH_LATENCY <https://mavlink.io/en/messages/common.html#MAV_CMD_CONTROL_HIGH_LATENCY>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_DEBUG_TRAP <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DEBUG_TRAP>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MAV_CMD_DO_ACCEPT_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_ACCEPT_MAG_CAL>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, ardupilotmega
  `MAV_CMD_DO_ADSB_OUT_IDENT <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ADSB_OUT_IDENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_DO_AUTOTUNE_ENABLE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_AUTOTUNE_ENABLE>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_AUX_FUNCTION <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_AUX_FUNCTION>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  `MAV_CMD_DO_CANCEL_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_CANCEL_MAG_CAL>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, ardupilotmega
  `MAV_CMD_DO_CHANGE_SPEED <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_DIGICAM_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_DIGICAM_CONFIGURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_DO_DIGICAM_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_DIGICAM_CONTROL>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_DO_ENGINE_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ENGINE_CONTROL>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_FENCE_ENABLE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FENCE_ENABLE>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_DO_FLIGHTTERMINATION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_DO_FOLLOW <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FOLLOW>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  `MAV_CMD_DO_GO_AROUND <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GO_AROUND>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_GRIPPER <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GRIPPER>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_DO_GUIDED_LIMITS <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GUIDED_LIMITS>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_DO_INVERTED_FLIGHT <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_INVERTED_FLIGHT>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_DO_JUMP <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_JUMP>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_DO_JUMP_TAG <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_JUMP_TAG>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_DO_LAND_START <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_MOTOR_TEST <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOTOR_TEST>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_MOUNT_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  `MAV_CMD_DO_MOUNT_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_DO_PARACHUTE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PARACHUTE>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_PAUSE_CONTINUE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_DO_REPEAT_RELAY <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPEAT_RELAY>`_, `AP_Mission/AP_Mission_Commands.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission_Commands.cpp>`_, common
  `MAV_CMD_DO_REPEAT_SERVO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPEAT_SERVO>`_, `AP_Mission/AP_Mission_Commands.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission_Commands.cpp>`_, common
  `MAV_CMD_DO_REPOSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_RETURN_PATH_START <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_RETURN_PATH_START>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_DO_SEND_BANNER <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SEND_BANNER>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MAV_CMD_DO_SEND_SCRIPT_MESSAGE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SEND_SCRIPT_MESSAGE>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  `MAV_CMD_DO_SET_CAM_TRIGG_DIST <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_DIST>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_DO_SET_HOME <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_DO_SET_MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MISSION_CURRENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_DO_SET_MODE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_DO_SET_RELAY <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_RELAY>`_, `AP_Mission/AP_Mission_Commands.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission_Commands.cpp>`_, common
  `MAV_CMD_DO_SET_RESUME_REPEAT_DIST <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SET_RESUME_REPEAT_DIST>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  `MAV_CMD_DO_SET_REVERSE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_REVERSE>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_DO_SET_ROI <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_DO_SET_ROI_LOCATION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_DO_SET_ROI_NONE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_DO_SET_ROI_SYSID <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  `MAV_CMD_DO_SET_SAFETY_SWITCH_STATE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SAFETY_SWITCH_STATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_DO_SET_SERVO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO>`_, `AP_Mission/AP_Mission_Commands.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission_Commands.cpp>`_, common
  `MAV_CMD_DO_SPRAYER <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SPRAYER>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  `MAV_CMD_DO_START_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_START_MAG_CAL>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, ardupilotmega
  `MAV_CMD_DO_VTOL_TRANSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_DO_WINCH <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_WINCH>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_EXTERNAL_POSITION_ESTIMATE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_EXTERNAL_POSITION_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MAV_CMD_EXTERNAL_WIND_ESTIMATE <https://mavlink.io/en/messages/development.html#MAV_CMD_EXTERNAL_WIND_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, development
  `MAV_CMD_FIXED_MAG_CAL_YAW <https://mavlink.io/en/messages/common.html#MAV_CMD_FIXED_MAG_CAL_YAW>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_FLASH_BOOTLOADER <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_FLASH_BOOTLOADER>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, ardupilotmega
  `MAV_CMD_GET_HOME_POSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_GET_HOME_POSITION>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_GET_MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_CMD_GET_MESSAGE_INTERVAL>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_GUIDED_CHANGE_ALTITUDE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_ALTITUDE>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, ardupilotmega
  `MAV_CMD_GUIDED_CHANGE_HEADING <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_HEADING>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, ardupilotmega
  `MAV_CMD_GUIDED_CHANGE_SPEED <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_SPEED>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, ardupilotmega
  `MAV_CMD_IMAGE_START_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_IMAGE_STOP_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_STOP_CAPTURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_JUMP_TAG <https://mavlink.io/en/messages/common.html#MAV_CMD_JUMP_TAG>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_MISSION_START <https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_NAV_ALTITUDE_WAIT <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_NAV_ALTITUDE_WAIT>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, ardupilotmega
  `MAV_CMD_NAV_ATTITUDE_TIME <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_NAV_ATTITUDE_TIME>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  `MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_NAV_DELAY <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_DELAY>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  `MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  `MAV_CMD_NAV_FENCE_RETURN_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_RETURN_POINT>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  `MAV_CMD_NAV_GUIDED_ENABLE <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_GUIDED_ENABLE>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_NAV_LAND <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND>`_, `ArduPlane/quadplane.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/quadplane.cpp>`_, common
  `MAV_CMD_NAV_LOITER_TIME <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TIME>`_, `ArduPlane/quadplane.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/quadplane.cpp>`_, common
  `MAV_CMD_NAV_LOITER_TO_ALT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT>`_, `ArduPlane/quadplane.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/quadplane.cpp>`_, common
  `MAV_CMD_NAV_LOITER_TURNS <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TURNS>`_, `ArduPlane/quadplane.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/quadplane.cpp>`_, common
  `MAV_CMD_NAV_LOITER_UNLIM <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_UNLIM>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_NAV_PAYLOAD_PLACE <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_PAYLOAD_PLACE>`_, `ArduPlane/quadplane.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/quadplane.cpp>`_, common
  `MAV_CMD_NAV_RETURN_TO_LAUNCH <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RETURN_TO_LAUNCH>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_NAV_SCRIPT_TIME <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_NAV_SCRIPT_TIME>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, ardupilotmega
  `MAV_CMD_NAV_SET_YAW_SPEED <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_SET_YAW_SPEED>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_NAV_SPLINE_WAYPOINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_SPLINE_WAYPOINT>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_NAV_TAKEOFF <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `MAV_CMD_NAV_TAKEOFF_LOCAL <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF_LOCAL>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  `MAV_CMD_NAV_VTOL_LAND <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND>`_, `ArduPlane/quadplane.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/quadplane.cpp>`_, common
  `MAV_CMD_NAV_VTOL_TAKEOFF <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_TAKEOFF>`_, `ArduPlane/quadplane.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/quadplane.cpp>`_, common
  `MAV_CMD_NAV_WAYPOINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT>`_, `ArduPlane/commands_logic.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/commands_logic.cpp>`_, common
  `MAV_CMD_PREFLIGHT_CALIBRATION <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_CALIBRATION>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_PREFLIGHT_STORAGE <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_STORAGE>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_PREFLIGHT_UAVCAN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_UAVCAN>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_REQUEST_MESSAGE <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_RUN_PREARM_CHECKS <https://mavlink.io/en/messages/common.html#MAV_CMD_RUN_PREARM_CHECKS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_SCRIPTING <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SCRIPTING>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MAV_CMD_SET_CAMERA_FOCUS <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_FOCUS>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_SET_CAMERA_SOURCE <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_SOURCE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_SET_CAMERA_ZOOM <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_ZOOM>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_SET_EKF_SOURCE_SET <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SET_EKF_SOURCE_SET>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MAV_CMD_SET_HAGL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SET_HAGL>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, ardupilotmega
  `MAV_CMD_SET_MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_START_RX_PAIR <https://mavlink.io/en/messages/common.html#MAV_CMD_START_RX_PAIR>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  `MAV_CMD_STORAGE_FORMAT <https://mavlink.io/en/messages/common.html#MAV_CMD_STORAGE_FORMAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAV_CMD_VIDEO_START_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_START_CAPTURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  `MAV_CMD_VIDEO_STOP_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_STOP_CAPTURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common

.. _ArduPlane_mavlink_requestable_messages:

Requestable Messages
====================

Messages that can be requested/streamed from the autopilot.

.. csv-table::
  :header: MAVLink Message, Code Source, MAVLink Dialect


  `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `AHRS <https://mavlink.io/en/messages/ardupilotmega.html#AHRS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `AHRS2 <https://mavlink.io/en/messages/ardupilotmega.html#AHRS2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `AIRSPEED <https://mavlink.io/en/messages/development.html#AIRSPEED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, development
  `AIS_VESSEL <https://mavlink.io/en/messages/common.html#AIS_VESSEL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `AOA_SSA <https://mavlink.io/en/messages/ardupilotmega.html#AOA_SSA>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `ATTITUDE_QUATERNION <https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `ATTITUDE_TARGET <https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE <https://mavlink.io/en/messages/common.html#AUTOPILOT_STATE_FOR_GIMBAL_DEVICE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `AUTOPILOT_VERSION <https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `BATTERY2 <https://mavlink.io/en/messages/ardupilotmega.html#BATTERY2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `BATTERY_STATUS <https://mavlink.io/en/messages/common.html#BATTERY_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `CAMERA_CAPTURE_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_CAPTURE_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `CAMERA_FEEDBACK <https://mavlink.io/en/messages/ardupilotmega.html#CAMERA_FEEDBACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `CAMERA_FOV_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_FOV_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#CAMERA_SETTINGS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `CAMERA_THERMAL_RANGE <https://mavlink.io/en/messages/common.html#CAMERA_THERMAL_RANGE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `DEEPSTALL <https://mavlink.io/en/messages/ardupilotmega.html#DEEPSTALL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `EFI_STATUS <https://mavlink.io/en/messages/common.html#EFI_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `EKF_STATUS_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `ESC_TELEMETRY_1_TO_4 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `EXTENDED_SYS_STATE <https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `FENCE_STATUS <https://mavlink.io/en/messages/common.html#FENCE_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GENERATOR_STATUS <https://mavlink.io/en/messages/common.html#GENERATOR_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GIMBAL_MANAGER_INFORMATION <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GIMBAL_MANAGER_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GPS2_RAW <https://mavlink.io/en/messages/common.html#GPS2_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GPS2_RTK <https://mavlink.io/en/messages/common.html#GPS2_RTK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GPS_GLOBAL_ORIGIN <https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GPS_RAW_INT <https://mavlink.io/en/messages/common.html#GPS_RAW_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GPS_RTK <https://mavlink.io/en/messages/common.html#GPS_RTK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HIGHRES_IMU <https://mavlink.io/en/messages/common.html#HIGHRES_IMU>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HIGH_LATENCY2 <https://mavlink.io/en/messages/common.html#HIGH_LATENCY2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HOME_POSITION <https://mavlink.io/en/messages/common.html#HOME_POSITION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HWSTATUS <https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `LOCAL_POSITION_NED <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MAG_CAL_PROGRESS <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MAG_CAL_REPORT <https://mavlink.io/en/messages/common.html#MAG_CAL_REPORT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MCU_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MCU_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MEMINFO <https://mavlink.io/en/messages/ardupilotmega.html#MEMINFO>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_ITEM_REACHED <https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `NAV_CONTROLLER_OUTPUT <https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `PID_TUNING <https://mavlink.io/en/messages/ardupilotmega.html#PID_TUNING>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `POSITION_TARGET_LOCAL_NED <https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `POWER_STATUS <https://mavlink.io/en/messages/common.html#POWER_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RANGEFINDER <https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `RAW_IMU <https://mavlink.io/en/messages/common.html#RAW_IMU>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RC_CHANNELS_RAW <https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RC_CHANNELS_SCALED <https://mavlink.io/en/messages/common.html#RC_CHANNELS_SCALED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RELAY_STATUS <https://mavlink.io/en/messages/common.html#RELAY_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RPM <https://mavlink.io/en/messages/ardupilotmega.html#RPM>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `SCALED_IMU <https://mavlink.io/en/messages/common.html#SCALED_IMU>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SCALED_IMU2 <https://mavlink.io/en/messages/common.html#SCALED_IMU2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SCALED_IMU3 <https://mavlink.io/en/messages/common.html#SCALED_IMU3>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SCALED_PRESSURE <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SCALED_PRESSURE2 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SCALED_PRESSURE3 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE3>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SERVO_OUTPUT_RAW <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SIMSTATE <https://mavlink.io/en/messages/ardupilotmega.html#SIMSTATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `SIM_STATE <https://mavlink.io/en/messages/common.html#SIM_STATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SYS_STATUS <https://mavlink.io/en/messages/common.html#SYS_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `TERRAIN_REPORT <https://mavlink.io/en/messages/common.html#TERRAIN_REPORT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `TERRAIN_REQUEST <https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `UAVIONIX_ADSB_OUT_STATUS <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, uAvionix
  `VFR_HUD <https://mavlink.io/en/messages/common.html#VFR_HUD>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `VIBRATION <https://mavlink.io/en/messages/common.html#VIBRATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `VIDEO_STREAM_INFORMATION <https://mavlink.io/en/messages/common.html#VIDEO_STREAM_INFORMATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `WATER_DEPTH <https://mavlink.io/en/messages/ardupilotmega.html#WATER_DEPTH>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `WINCH_STATUS <https://mavlink.io/en/messages/common.html#WINCH_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `WIND <https://mavlink.io/en/messages/ardupilotmega.html#WIND>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega

.. _ArduPlane_mavlink_outgoing_messages:

Outgoing Messages
=================

Messages the autopilot will send automatically (unrequested).

.. csv-table::
  :header: MAVLink Message, Code Source, MAVLink Dialect


  `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, common
  `AHRS <https://mavlink.io/en/messages/ardupilotmega.html#AHRS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `AHRS2 <https://mavlink.io/en/messages/ardupilotmega.html#AHRS2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `AOA_SSA <https://mavlink.io/en/messages/ardupilotmega.html#AOA_SSA>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, ardupilotmega
  `AP_ADC <https://mavlink.io/en/messages/ardupilotmega.html#AP_ADC>`_, `AP_HAL_ESP32/AnalogIn.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ESP32/AnalogIn.cpp>`_, ardupilotmega
  `ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `ATTITUDE_QUATERNION <https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `ATTITUDE_TARGET <https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE <https://mavlink.io/en/messages/common.html#AUTOPILOT_STATE_FOR_GIMBAL_DEVICE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `AUTOPILOT_VERSION <https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `BATTERY2 <https://mavlink.io/en/messages/ardupilotmega.html#BATTERY2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `BATTERY_STATUS <https://mavlink.io/en/messages/common.html#BATTERY_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `CAMERA_CAPTURE_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_CAPTURE_STATUS>`_, `AP_Camera/AP_Camera_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_Backend.cpp>`_, common
  `CAMERA_FEEDBACK <https://mavlink.io/en/messages/ardupilotmega.html#CAMERA_FEEDBACK>`_, `AP_Camera/AP_Camera_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_Backend.cpp>`_, ardupilotmega
  `CAMERA_FOV_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_FOV_STATUS>`_, `AP_Camera/AP_Camera_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_Backend.cpp>`_, common
  `CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION>`_, `AP_Camera/AP_Camera_MAVLinkCamV2.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_MAVLinkCamV2.cpp>`_, common
  `CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#CAMERA_SETTINGS>`_, `AP_Camera/AP_Camera_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_Backend.cpp>`_, common
  `CAMERA_THERMAL_RANGE <https://mavlink.io/en/messages/common.html#CAMERA_THERMAL_RANGE>`_, `AP_Mount/AP_Mount_Siyi.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_Siyi.cpp>`_, common
  `CANFD_FRAME <https://mavlink.io/en/messages/common.html#CANFD_FRAME>`_, `AP_CANManager/AP_CANManager.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CANManager/AP_CANManager.cpp>`_, common
  `CAN_FRAME <https://mavlink.io/en/messages/common.html#CAN_FRAME>`_, `AP_CANManager/AP_CANManager.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CANManager/AP_CANManager.cpp>`_, common
  `COMMAND_ACK <https://mavlink.io/en/messages/common.html#COMMAND_ACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`_, `AP_Mount/AP_Mount_SToRM32.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_SToRM32.cpp>`_, common
  `DATA16 <https://mavlink.io/en/messages/ardupilotmega.html#DATA16>`_, `AP_Radio/AP_Radio_cc2500.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Radio/AP_Radio_cc2500.cpp>`_, ardupilotmega
  `DEEPSTALL <https://mavlink.io/en/messages/ardupilotmega.html#DEEPSTALL>`_, `AP_Landing/AP_Landing_Deepstall.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Landing/AP_Landing_Deepstall.cpp>`_, ardupilotmega
  `DEVICE_OP_READ_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#DEVICE_OP_READ_REPLY>`_, `GCS_MAVLink/GCS_DeviceOp.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_DeviceOp.cpp>`_, ardupilotmega
  `DEVICE_OP_WRITE_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#DEVICE_OP_WRITE_REPLY>`_, `GCS_MAVLink/GCS_DeviceOp.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_DeviceOp.cpp>`_, ardupilotmega
  `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `EFI_STATUS <https://mavlink.io/en/messages/common.html#EFI_STATUS>`_, `AP_EFI/AP_EFI.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_EFI/AP_EFI.cpp>`_, common
  `EKF_STATUS_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT>`_, `AP_ExternalAHRS/AP_ExternalAHRS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ExternalAHRS/AP_ExternalAHRS.cpp>`_, ardupilotmega
  `EXTENDED_SYS_STATE <https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `FENCE_STATUS <https://mavlink.io/en/messages/common.html#FENCE_STATUS>`_, `GCS_MAVLink/GCS_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Fence.cpp>`_, common
  `FILE_TRANSFER_PROTOCOL <https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL>`_, `GCS_MAVLink/GCS_FTP.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_FTP.cpp>`_, common
  `GENERATOR_STATUS <https://mavlink.io/en/messages/common.html#GENERATOR_STATUS>`_, `AP_Generator/AP_Generator_RichenPower.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Generator/AP_Generator_RichenPower.cpp>`_, common
  `GIMBAL_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_CONTROL>`_, `AP_Mount/SoloGimbal.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/SoloGimbal.cpp>`_, ardupilotmega
  `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`_, `AP_Mount/AP_Mount_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_Backend.cpp>`_, common
  `GIMBAL_MANAGER_INFORMATION <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION>`_, `AP_Mount/AP_Mount_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_Backend.cpp>`_, common
  `GIMBAL_MANAGER_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS>`_, `AP_Mount/AP_Mount_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_Backend.cpp>`_, common
  `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GOPRO_SET_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_SET_REQUEST>`_, `AP_Camera/AP_Camera_SoloGimbal.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_SoloGimbal.cpp>`_, ardupilotmega
  `GPS2_RAW <https://mavlink.io/en/messages/common.html#GPS2_RAW>`_, `AP_GPS/AP_GPS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS.cpp>`_, common
  `GPS2_RTK <https://mavlink.io/en/messages/common.html#GPS2_RTK>`_, `AP_GPS/GPS_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/GPS_Backend.cpp>`_, common
  `GPS_GLOBAL_ORIGIN <https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `GPS_RAW_INT <https://mavlink.io/en/messages/common.html#GPS_RAW_INT>`_, `AP_GPS/AP_GPS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS.cpp>`_, common
  `GPS_RTK <https://mavlink.io/en/messages/common.html#GPS_RTK>`_, `AP_GPS/GPS_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/GPS_Backend.cpp>`_, common
  `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HIGH_LATENCY2 <https://mavlink.io/en/messages/common.html#HIGH_LATENCY2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HOME_POSITION <https://mavlink.io/en/messages/common.html#HOME_POSITION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `HWSTATUS <https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `HYGROMETER_SENSOR <https://mavlink.io/en/messages/common.html#HYGROMETER_SENSOR>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `LOCAL_POSITION_NED <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `LOG_ENTRY <https://mavlink.io/en/messages/common.html#LOG_ENTRY>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  `MAG_CAL_PROGRESS <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, ardupilotmega
  `MAG_CAL_REPORT <https://mavlink.io/en/messages/common.html#MAG_CAL_REPORT>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, common
  `MCU_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MCU_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MEMINFO <https://mavlink.io/en/messages/ardupilotmega.html#MEMINFO>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MESSAGE_INTERVAL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_ACK <https://mavlink.io/en/messages/common.html#MISSION_ACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`_, `GCS_MAVLink/MissionItemProtocol_Waypoints.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Waypoints.cpp>`_, common
  `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_ITEM_REACHED <https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `MISSION_REQUEST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST>`_, `GCS_MAVLink/MissionItemProtocol.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol.cpp>`_, common
  `NAMED_VALUE_FLOAT <https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `NAV_CONTROLLER_OUTPUT <https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `OSD_PARAM_CONFIG_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#OSD_PARAM_CONFIG_REPLY>`_, `AP_OSD/AP_OSD.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OSD/AP_OSD.cpp>`_, ardupilotmega
  `OSD_PARAM_SHOW_CONFIG_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#OSD_PARAM_SHOW_CONFIG_REPLY>`_, `AP_OSD/AP_OSD.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OSD/AP_OSD.cpp>`_, ardupilotmega
  `PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`_, `AP_Mount/SoloGimbal_Parameters.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/SoloGimbal_Parameters.cpp>`_, common
  `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`_, `AP_Mount/SoloGimbal_Parameters.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/SoloGimbal_Parameters.cpp>`_, common
  `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`_, `GCS_MAVLink/GCS_Param.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Param.cpp>`_, common
  `PID_TUNING <https://mavlink.io/en/messages/ardupilotmega.html#PID_TUNING>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, ardupilotmega
  `POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, common
  `POWER_STATUS <https://mavlink.io/en/messages/common.html#POWER_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RALLY_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_POINT>`_, `GCS_MAVLink/GCS_Rally.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Rally.cpp>`_, ardupilotmega
  `RANGEFINDER <https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `RAW_IMU <https://mavlink.io/en/messages/common.html#RAW_IMU>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RC_CHANNELS_RAW <https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `RELAY_STATUS <https://mavlink.io/en/messages/common.html#RELAY_STATUS>`_, `AP_Relay/AP_Relay.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Relay/AP_Relay.cpp>`_, common
  `RPM <https://mavlink.io/en/messages/ardupilotmega.html#RPM>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `SERVO_OUTPUT_RAW <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `STATUSTEXT <https://mavlink.io/en/messages/common.html#STATUSTEXT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `SYS_STATUS <https://mavlink.io/en/messages/common.html#SYS_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `TERRAIN_REPORT <https://mavlink.io/en/messages/common.html#TERRAIN_REPORT>`_, `AP_Terrain/TerrainGCS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Terrain/TerrainGCS.cpp>`_, common
  `TERRAIN_REQUEST <https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST>`_, `AP_Terrain/TerrainGCS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Terrain/TerrainGCS.cpp>`_, common
  `TIMESYNC <https://mavlink.io/en/messages/common.html#TIMESYNC>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `UAVIONIX_ADSB_OUT_CFG <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CFG>`_, `AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp>`_, uAvionix
  `UAVIONIX_ADSB_OUT_DYNAMIC <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_DYNAMIC>`_, `AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp>`_, uAvionix
  `VFR_HUD <https://mavlink.io/en/messages/common.html#VFR_HUD>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `VIBRATION <https://mavlink.io/en/messages/common.html#VIBRATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  `WATER_DEPTH <https://mavlink.io/en/messages/ardupilotmega.html#WATER_DEPTH>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  `WINCH_STATUS <https://mavlink.io/en/messages/common.html#WINCH_STATUS>`_, `AP_Winch/AP_Winch_Daiwa.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Winch/AP_Winch_Daiwa.cpp>`_, common
  `WIND <https://mavlink.io/en/messages/ardupilotmega.html#WIND>`_, `ArduPlane/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane/GCS_Mavlink.cpp>`_, ardupilotmega

.. _ArduPlane_mavlink_stream_groups:

Stream Groups
=============

Message groups with stream rates requestable by ``SRn_*`` parameters. Messages in a group are only sent if the corresponding feature is active.

.. csv-table::
  :header: MAVLink Message, Stream Group Parameter, MAVLink Dialect


  `AIRSPEED <https://mavlink.io/en/messages/development.html#AIRSPEED>`_, SRn_RAW_SENSORS, development
  `RAW_IMU <https://mavlink.io/en/messages/common.html#RAW_IMU>`_, SRn_RAW_SENSORS, common
  `SCALED_IMU2 <https://mavlink.io/en/messages/common.html#SCALED_IMU2>`_, SRn_RAW_SENSORS, common
  `SCALED_IMU3 <https://mavlink.io/en/messages/common.html#SCALED_IMU3>`_, SRn_RAW_SENSORS, common
  `SCALED_PRESSURE <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE>`_, SRn_RAW_SENSORS, common
  `SCALED_PRESSURE2 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE2>`_, SRn_RAW_SENSORS, common
  `SCALED_PRESSURE3 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE3>`_, SRn_RAW_SENSORS, common
  `FENCE_STATUS <https://mavlink.io/en/messages/common.html#FENCE_STATUS>`_, SRn_EXTENDED_STATUS, common
  `GPS2_RAW <https://mavlink.io/en/messages/common.html#GPS2_RAW>`_, SRn_EXTENDED_STATUS, common
  `GPS2_RTK <https://mavlink.io/en/messages/common.html#GPS2_RTK>`_, SRn_EXTENDED_STATUS, common
  `GPS_RAW_INT <https://mavlink.io/en/messages/common.html#GPS_RAW_INT>`_, SRn_EXTENDED_STATUS, common
  `GPS_RTK <https://mavlink.io/en/messages/common.html#GPS_RTK>`_, SRn_EXTENDED_STATUS, common
  `MCU_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MCU_STATUS>`_, SRn_EXTENDED_STATUS, ardupilotmega
  `MEMINFO <https://mavlink.io/en/messages/ardupilotmega.html#MEMINFO>`_, SRn_EXTENDED_STATUS, ardupilotmega
  `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`_, SRn_EXTENDED_STATUS, common
  `NAV_CONTROLLER_OUTPUT <https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT>`_, SRn_EXTENDED_STATUS, common
  `POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT>`_, SRn_EXTENDED_STATUS, common
  `POWER_STATUS <https://mavlink.io/en/messages/common.html#POWER_STATUS>`_, SRn_EXTENDED_STATUS, common
  `SYS_STATUS <https://mavlink.io/en/messages/common.html#SYS_STATUS>`_, SRn_EXTENDED_STATUS, common
  `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`_, SRn_POSITION, common
  `LOCAL_POSITION_NED <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED>`_, SRn_POSITION, common
  `RC_CHANNELS_SCALED <https://mavlink.io/en/messages/common.html#RC_CHANNELS_SCALED>`_, SRn_RAW_CONTROLLER, common
  `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`_, SRn_RC_CHANNELS, common
  `RC_CHANNELS_RAW <https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW>`_, SRn_RC_CHANNELS, common
  RC_CHANNELS_RAW_ENABLED, SRn_RC_CHANNELS, UNKNOWN
  `SERVO_OUTPUT_RAW <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>`_, SRn_RC_CHANNELS, common
  `AHRS2 <https://mavlink.io/en/messages/ardupilotmega.html#AHRS2>`_, SRn_EXTRA1, ardupilotmega
  `AOA_SSA <https://mavlink.io/en/messages/ardupilotmega.html#AOA_SSA>`_, SRn_EXTRA1, ardupilotmega
  `ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`_, SRn_EXTRA1, common
  `DEEPSTALL <https://mavlink.io/en/messages/ardupilotmega.html#DEEPSTALL>`_, SRn_EXTRA1, ardupilotmega
  `EFI_STATUS <https://mavlink.io/en/messages/common.html#EFI_STATUS>`_, SRn_EXTRA1, common
  `ESC_TELEMETRY_1_TO_4 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4>`_, SRn_EXTRA1, ardupilotmega
  HYGROMETER, SRn_EXTRA1, UNKNOWN
  `PID_TUNING <https://mavlink.io/en/messages/ardupilotmega.html#PID_TUNING>`_, SRn_EXTRA1, ardupilotmega
  `RPM <https://mavlink.io/en/messages/ardupilotmega.html#RPM>`_, SRn_EXTRA1, ardupilotmega
  `SIMSTATE <https://mavlink.io/en/messages/ardupilotmega.html#SIMSTATE>`_, SRn_EXTRA1, ardupilotmega
  `VFR_HUD <https://mavlink.io/en/messages/common.html#VFR_HUD>`_, SRn_EXTRA2, common
  `AHRS <https://mavlink.io/en/messages/ardupilotmega.html#AHRS>`_, SRn_EXTRA3, ardupilotmega
  `BATTERY_STATUS <https://mavlink.io/en/messages/common.html#BATTERY_STATUS>`_, SRn_EXTRA3, common
  `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`_, SRn_EXTRA3, common
  `EKF_STATUS_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT>`_, SRn_EXTRA3, ardupilotmega
  `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`_, SRn_EXTRA3, common
  `MAG_CAL_PROGRESS <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS>`_, SRn_EXTRA3, ardupilotmega
  `MAG_CAL_REPORT <https://mavlink.io/en/messages/common.html#MAG_CAL_REPORT>`_, SRn_EXTRA3, common
  `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`_, SRn_EXTRA3, common
  `RANGEFINDER <https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER>`_, SRn_EXTRA3, ardupilotmega
  `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`_, SRn_EXTRA3, common
  `TERRAIN_REPORT <https://mavlink.io/en/messages/common.html#TERRAIN_REPORT>`_, SRn_EXTRA3, common
  `TERRAIN_REQUEST <https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST>`_, SRn_EXTRA3, common
  `VIBRATION <https://mavlink.io/en/messages/common.html#VIBRATION>`_, SRn_EXTRA3, common
  `WIND <https://mavlink.io/en/messages/ardupilotmega.html#WIND>`_, SRn_EXTRA3, ardupilotmega
  `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`_, SRn_PARAMS, common
  `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`_, SRn_ADSB, common
  `AIS_VESSEL <https://mavlink.io/en/messages/common.html#AIS_VESSEL>`_, SRn_ADSB, common

.. _ArduPlane_mavlink_missing_messages:

Missing Messages
================

Unsupported / unhandled messages.

.. csv-table::
  :header: MAVLink Message, Code Source, MAVLink Dialect


  `ACTUATOR_CONTROL_TARGET <https://mavlink.io/en/messages/common.html#ACTUATOR_CONTROL_TARGET>`_, UNSUPPORTED, common
  `ACTUATOR_OUTPUT_STATUS <https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_STATUS>`_, UNSUPPORTED, common
  `ADAP_TUNING <https://mavlink.io/en/messages/ardupilotmega.html#ADAP_TUNING>`_, UNSUPPORTED, ardupilotmega
  `AHRS3 <https://mavlink.io/en/messages/ardupilotmega.html#AHRS3>`_, UNSUPPORTED, ardupilotmega
  `AIRLINK_AUTH <https://mavlink.io/en/messages/ardupilotmega.html#AIRLINK_AUTH>`_, UNSUPPORTED, ardupilotmega
  `AIRLINK_AUTH_RESPONSE <https://mavlink.io/en/messages/ardupilotmega.html#AIRLINK_AUTH_RESPONSE>`_, UNSUPPORTED, ardupilotmega
  `AIRSPEED_AUTOCAL <https://mavlink.io/en/messages/ardupilotmega.html#AIRSPEED_AUTOCAL>`_, UNSUPPORTED, ardupilotmega
  `ALTITUDE <https://mavlink.io/en/messages/common.html#ALTITUDE>`_, UNSUPPORTED, common
  `ATTITUDE_QUATERNION_COV <https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION_COV>`_, UNSUPPORTED, common
  `AUTH_KEY <https://mavlink.io/en/messages/common.html#AUTH_KEY>`_, UNSUPPORTED, common
  `BAD_DATA <https://mavlink.io/en/messages/common.html#BAD_DATA>`_, UNSUPPORTED, common
  `BUTTON_CHANGE <https://mavlink.io/en/messages/common.html#BUTTON_CHANGE>`_, UNSUPPORTED, common
  `CAMERA_IMAGE_CAPTURED <https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED>`_, UNSUPPORTED, common
  `CAMERA_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#CAMERA_STATUS>`_, UNSUPPORTED, ardupilotmega
  `CAMERA_TRACKING_GEO_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_GEO_STATUS>`_, UNSUPPORTED, common
  `CAMERA_TRACKING_IMAGE_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_IMAGE_STATUS>`_, UNSUPPORTED, common
  `CAMERA_TRIGGER <https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER>`_, UNSUPPORTED, common
  `CHANGE_OPERATOR_CONTROL <https://mavlink.io/en/messages/common.html#CHANGE_OPERATOR_CONTROL>`_, UNSUPPORTED, common
  `CHANGE_OPERATOR_CONTROL_ACK <https://mavlink.io/en/messages/common.html#CHANGE_OPERATOR_CONTROL_ACK>`_, UNSUPPORTED, common
  `COLLISION <https://mavlink.io/en/messages/common.html#COLLISION>`_, UNSUPPORTED, common
  `COMPASSMOT_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#COMPASSMOT_STATUS>`_, UNSUPPORTED, ardupilotmega
  `CONTROL_SYSTEM_STATE <https://mavlink.io/en/messages/common.html#CONTROL_SYSTEM_STATE>`_, UNSUPPORTED, common
  `CUBEPILOT_FIRMWARE_UPDATE_RESP <https://mavlink.io/en/messages/cubepilot.html#CUBEPILOT_FIRMWARE_UPDATE_RESP>`_, UNSUPPORTED, cubepilot
  `CUBEPILOT_FIRMWARE_UPDATE_START <https://mavlink.io/en/messages/cubepilot.html#CUBEPILOT_FIRMWARE_UPDATE_START>`_, UNSUPPORTED, cubepilot
  `CUBEPILOT_RAW_RC <https://mavlink.io/en/messages/cubepilot.html#CUBEPILOT_RAW_RC>`_, UNSUPPORTED, cubepilot
  `DATA32 <https://mavlink.io/en/messages/ardupilotmega.html#DATA32>`_, UNSUPPORTED, ardupilotmega
  `DATA64 <https://mavlink.io/en/messages/ardupilotmega.html#DATA64>`_, UNSUPPORTED, ardupilotmega
  `DATA_STREAM <https://mavlink.io/en/messages/common.html#DATA_STREAM>`_, UNSUPPORTED, common
  `DATA_TRANSMISSION_HANDSHAKE <https://mavlink.io/en/messages/common.html#DATA_TRANSMISSION_HANDSHAKE>`_, UNSUPPORTED, common
  `DEBUG <https://mavlink.io/en/messages/common.html#DEBUG>`_, UNSUPPORTED, common
  `DEBUG_FLOAT_ARRAY <https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY>`_, UNSUPPORTED, common
  `DEBUG_VECT <https://mavlink.io/en/messages/common.html#DEBUG_VECT>`_, UNSUPPORTED, common
  `DIGICAM_CONFIGURE <https://mavlink.io/en/messages/ardupilotmega.html#DIGICAM_CONFIGURE>`_, UNSUPPORTED, ardupilotmega
  `ENCAPSULATED_DATA <https://mavlink.io/en/messages/common.html#ENCAPSULATED_DATA>`_, UNSUPPORTED, common
  `ESC_TELEMETRY_13_TO_16 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_13_TO_16>`_, UNSUPPORTED, ardupilotmega
  `ESC_TELEMETRY_17_TO_20 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_17_TO_20>`_, UNSUPPORTED, ardupilotmega
  `ESC_TELEMETRY_21_TO_24 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_21_TO_24>`_, UNSUPPORTED, ardupilotmega
  `ESC_TELEMETRY_25_TO_28 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_25_TO_28>`_, UNSUPPORTED, ardupilotmega
  `ESC_TELEMETRY_29_TO_32 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_29_TO_32>`_, UNSUPPORTED, ardupilotmega
  `ESC_TELEMETRY_5_TO_8 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_5_TO_8>`_, UNSUPPORTED, ardupilotmega
  `ESC_TELEMETRY_9_TO_12 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_9_TO_12>`_, UNSUPPORTED, ardupilotmega
  `ESTIMATOR_STATUS <https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS>`_, UNSUPPORTED, common
  `FLIGHT_INFORMATION <https://mavlink.io/en/messages/common.html#FLIGHT_INFORMATION>`_, UNSUPPORTED, common
  `GIMBAL_DEVICE_SET_ATTITUDE <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE>`_, UNSUPPORTED, common
  `GIMBAL_MANAGER_SET_MANUAL_CONTROL <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL>`_, UNSUPPORTED, common
  `GIMBAL_TORQUE_CMD_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_TORQUE_CMD_REPORT>`_, UNSUPPORTED, ardupilotmega
  `GLOBAL_POSITION_INT_COV <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT_COV>`_, UNSUPPORTED, common
  `GNSS_INTEGRITY <https://mavlink.io/en/messages/development.html#GNSS_INTEGRITY>`_, UNSUPPORTED, development
  `GOPRO_GET_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_GET_REQUEST>`_, UNSUPPORTED, ardupilotmega
  `GOPRO_GET_RESPONSE <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_GET_RESPONSE>`_, UNSUPPORTED, ardupilotmega
  `GOPRO_SET_RESPONSE <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_SET_RESPONSE>`_, UNSUPPORTED, ardupilotmega
  `GPS_STATUS <https://mavlink.io/en/messages/common.html#GPS_STATUS>`_, UNSUPPORTED, common
  `HERELINK_TELEM <https://mavlink.io/en/messages/cubepilot.html#HERELINK_TELEM>`_, UNSUPPORTED, cubepilot
  `HERELINK_VIDEO_STREAM_INFORMATION <https://mavlink.io/en/messages/cubepilot.html#HERELINK_VIDEO_STREAM_INFORMATION>`_, UNSUPPORTED, cubepilot
  `HIGH_LATENCY <https://mavlink.io/en/messages/common.html#HIGH_LATENCY>`_, UNSUPPORTED, common
  `HIL_ACTUATOR_CONTROLS <https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS>`_, UNSUPPORTED, common
  `HIL_CONTROLS <https://mavlink.io/en/messages/common.html#HIL_CONTROLS>`_, UNSUPPORTED, common
  `HIL_OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW>`_, UNSUPPORTED, common
  `HIL_RC_INPUTS_RAW <https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW>`_, UNSUPPORTED, common
  `HIL_SENSOR <https://mavlink.io/en/messages/common.html#HIL_SENSOR>`_, UNSUPPORTED, common
  `HIL_STATE <https://mavlink.io/en/messages/common.html#HIL_STATE>`_, UNSUPPORTED, common
  `HIL_STATE_QUATERNION <https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION>`_, UNSUPPORTED, common
  `ICAROUS_HEARTBEAT <https://mavlink.io/en/messages/icarous.html#ICAROUS_HEARTBEAT>`_, UNSUPPORTED, icarous
  `ICAROUS_KINEMATIC_BANDS <https://mavlink.io/en/messages/icarous.html#ICAROUS_KINEMATIC_BANDS>`_, UNSUPPORTED, icarous
  `ISBD_LINK_STATUS <https://mavlink.io/en/messages/common.html#ISBD_LINK_STATUS>`_, UNSUPPORTED, common
  `LIMITS_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#LIMITS_STATUS>`_, UNSUPPORTED, ardupilotmega
  `LOCAL_POSITION_NED_COV <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_COV>`_, UNSUPPORTED, common
  `LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET>`_, UNSUPPORTED, common
  `LOGGING_ACK <https://mavlink.io/en/messages/common.html#LOGGING_ACK>`_, UNSUPPORTED, common
  `LOGGING_DATA <https://mavlink.io/en/messages/common.html#LOGGING_DATA>`_, UNSUPPORTED, common
  `LOGGING_DATA_ACKED <https://mavlink.io/en/messages/common.html#LOGGING_DATA_ACKED>`_, UNSUPPORTED, common
  `LOG_DATA <https://mavlink.io/en/messages/common.html#LOG_DATA>`_, UNSUPPORTED, common
  `LOWEHEISER_GOV_EFI <https://mavlink.io/en/messages/ardupilotmega.html#LOWEHEISER_GOV_EFI>`_, UNSUPPORTED, ardupilotmega
  `MANUAL_SETPOINT <https://mavlink.io/en/messages/common.html#MANUAL_SETPOINT>`_, UNSUPPORTED, common
  `MEMORY_VECT <https://mavlink.io/en/messages/common.html#MEMORY_VECT>`_, UNSUPPORTED, common
  `MISSION_CHECKSUM <https://mavlink.io/en/messages/development.html#MISSION_CHECKSUM>`_, UNSUPPORTED, development
  `MISSION_REQUEST_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_PARTIAL_LIST>`_, UNSUPPORTED, common
  `MOUNT_ORIENTATION <https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION>`_, UNSUPPORTED, common
  `MOUNT_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_STATUS>`_, UNSUPPORTED, ardupilotmega
  `NAMED_VALUE_INT <https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT>`_, UNSUPPORTED, common
  `OPEN_DRONE_ID_AUTHENTICATION <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_AUTHENTICATION>`_, UNSUPPORTED, common
  `OPEN_DRONE_ID_LOCATION <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_LOCATION>`_, UNSUPPORTED, common
  `OPEN_DRONE_ID_MESSAGE_PACK <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_MESSAGE_PACK>`_, UNSUPPORTED, common
  `OPTICAL_FLOW_RAD <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD>`_, UNSUPPORTED, common
  `PARAM_EXT_ACK <https://mavlink.io/en/messages/common.html#PARAM_EXT_ACK>`_, UNSUPPORTED, common
  `PARAM_EXT_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_EXT_REQUEST_LIST>`_, UNSUPPORTED, common
  `PARAM_EXT_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_EXT_REQUEST_READ>`_, UNSUPPORTED, common
  `PARAM_EXT_SET <https://mavlink.io/en/messages/common.html#PARAM_EXT_SET>`_, UNSUPPORTED, common
  `PARAM_EXT_VALUE <https://mavlink.io/en/messages/common.html#PARAM_EXT_VALUE>`_, UNSUPPORTED, common
  `PARAM_MAP_RC <https://mavlink.io/en/messages/common.html#PARAM_MAP_RC>`_, UNSUPPORTED, common
  `PING <https://mavlink.io/en/messages/common.html#PING>`_, UNSUPPORTED, common
  `RAW_PRESSURE <https://mavlink.io/en/messages/common.html#RAW_PRESSURE>`_, UNSUPPORTED, common
  `RAW_RPM <https://mavlink.io/en/messages/common.html#RAW_RPM>`_, UNSUPPORTED, common
  `REMOTE_LOG_DATA_BLOCK <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_DATA_BLOCK>`_, UNSUPPORTED, ardupilotmega
  `RESOURCE_REQUEST <https://mavlink.io/en/messages/common.html#RESOURCE_REQUEST>`_, UNSUPPORTED, common
  `SAFETY_ALLOWED_AREA <https://mavlink.io/en/messages/common.html#SAFETY_ALLOWED_AREA>`_, UNSUPPORTED, common
  `SAFETY_SET_ALLOWED_AREA <https://mavlink.io/en/messages/common.html#SAFETY_SET_ALLOWED_AREA>`_, UNSUPPORTED, common
  `SENSOR_OFFSETS <https://mavlink.io/en/messages/ardupilotmega.html#SENSOR_OFFSETS>`_, UNSUPPORTED, ardupilotmega
  `SET_ACTUATOR_CONTROL_TARGET <https://mavlink.io/en/messages/common.html#SET_ACTUATOR_CONTROL_TARGET>`_, UNSUPPORTED, common
  `SET_HOME_POSITION <https://mavlink.io/en/messages/common.html#SET_HOME_POSITION>`_, UNSUPPORTED, common
  `SET_MAG_OFFSETS <https://mavlink.io/en/messages/ardupilotmega.html#SET_MAG_OFFSETS>`_, UNSUPPORTED, ardupilotmega
  `SMART_BATTERY_INFO <https://mavlink.io/en/messages/common.html#SMART_BATTERY_INFO>`_, UNSUPPORTED, common
  `STORAGE_INFORMATION <https://mavlink.io/en/messages/common.html#STORAGE_INFORMATION>`_, UNSUPPORTED, common
  `TRAJECTORY_REPRESENTATION_BEZIER <https://mavlink.io/en/messages/common.html#TRAJECTORY_REPRESENTATION_BEZIER>`_, UNSUPPORTED, common
  `TRAJECTORY_REPRESENTATION_WAYPOINTS <https://mavlink.io/en/messages/common.html#TRAJECTORY_REPRESENTATION_WAYPOINTS>`_, UNSUPPORTED, common
  `TUNNEL <https://mavlink.io/en/messages/common.html#TUNNEL>`_, UNSUPPORTED, common
  `UAVCAN_NODE_INFO <https://mavlink.io/en/messages/common.html#UAVCAN_NODE_INFO>`_, UNSUPPORTED, common
  `UAVCAN_NODE_STATUS <https://mavlink.io/en/messages/common.html#UAVCAN_NODE_STATUS>`_, UNSUPPORTED, common
  `UAVIONIX_ADSB_GET <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_GET>`_, UNSUPPORTED, uAvionix
  `UAVIONIX_ADSB_OUT_CFG_FLIGHTID <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CFG_FLIGHTID>`_, UNSUPPORTED, uAvionix
  `UAVIONIX_ADSB_OUT_CFG_REGISTRATION <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CFG_REGISTRATION>`_, UNSUPPORTED, uAvionix
  `UNKNOWN <https://mavlink.io/en/messages/common.html#UNKNOWN>`_, UNSUPPORTED, common
  `UTM_GLOBAL_POSITION <https://mavlink.io/en/messages/common.html#UTM_GLOBAL_POSITION>`_, UNSUPPORTED, common
  `V2_EXTENSION <https://mavlink.io/en/messages/common.html#V2_EXTENSION>`_, UNSUPPORTED, common
  `VIDEO_STREAM_STATUS <https://mavlink.io/en/messages/common.html#VIDEO_STREAM_STATUS>`_, UNSUPPORTED, common
  `WHEEL_DISTANCE <https://mavlink.io/en/messages/common.html#WHEEL_DISTANCE>`_, UNSUPPORTED, common
  `WIFI_CONFIG_AP <https://mavlink.io/en/messages/common.html#WIFI_CONFIG_AP>`_, UNSUPPORTED, common
  `WIND_COV <https://mavlink.io/en/messages/common.html#WIND_COV>`_, UNSUPPORTED, common

.. _ArduPlane_mavlink_missing_commands:

Missing Commands
================

Unsupported / unhandled commands.

.. csv-table::
  :header: MAVLink Message, Code Source, MAVLink Dialect


  `MAV_CMD_ARM_AUTHORIZATION_REQUEST <https://mavlink.io/en/messages/common.html#MAV_CMD_ARM_AUTHORIZATION_REQUEST>`_, UNSUPPORTED, common
  `MAV_CMD_CONDITION_CHANGE_ALT <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_CHANGE_ALT>`_, UNSUPPORTED, common
  `MAV_CMD_CONDITION_LAST <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_LAST>`_, UNSUPPORTED, common
  `MAV_CMD_DO_CHANGE_ALTITUDE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_ALTITUDE>`_, UNSUPPORTED, common
  `MAV_CMD_DO_CONTROL_VIDEO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CONTROL_VIDEO>`_, UNSUPPORTED, common
  `MAV_CMD_DO_FOLLOW_REPOSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FOLLOW_REPOSITION>`_, UNSUPPORTED, common
  `MAV_CMD_DO_GUIDED_MASTER <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GUIDED_MASTER>`_, UNSUPPORTED, common
  `MAV_CMD_DO_LAST <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAST>`_, UNSUPPORTED, common
  `MAV_CMD_DO_MOUNT_CONTROL_QUAT <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL_QUAT>`_, UNSUPPORTED, common
  `MAV_CMD_DO_RALLY_LAND <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_RALLY_LAND>`_, UNSUPPORTED, common
  `MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL>`_, UNSUPPORTED, common
  `MAV_CMD_DO_SET_PARAMETER <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_PARAMETER>`_, UNSUPPORTED, common
  `MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET>`_, UNSUPPORTED, common
  `MAV_CMD_DO_SET_SYS_CMP_ID <https://mavlink.io/en/messages/development.html#MAV_CMD_DO_SET_SYS_CMP_ID>`_, UNSUPPORTED, development
  `MAV_CMD_DO_TRIGGER_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_TRIGGER_CONTROL>`_, UNSUPPORTED, common
  `MAV_CMD_ENUM_END <https://mavlink.io/en/messages/common.html#MAV_CMD_ENUM_END>`_, UNSUPPORTED, common
  `MAV_CMD_FIXED_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_FIXED_MAG_CAL>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_FIXED_MAG_CAL_FIELD <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_FIXED_MAG_CAL_FIELD>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_GIMBAL_FULL_RESET <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GIMBAL_FULL_RESET>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_GIMBAL_RESET <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GIMBAL_RESET>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_LOGGING_START <https://mavlink.io/en/messages/common.html#MAV_CMD_LOGGING_START>`_, UNSUPPORTED, common
  `MAV_CMD_LOGGING_STOP <https://mavlink.io/en/messages/common.html#MAV_CMD_LOGGING_STOP>`_, UNSUPPORTED, common
  `MAV_CMD_LOWEHEISER_SET_STATE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_LOWEHEISER_SET_STATE>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_NAV_FOLLOW <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FOLLOW>`_, UNSUPPORTED, common
  `MAV_CMD_NAV_LAND_LOCAL <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND_LOCAL>`_, UNSUPPORTED, common
  `MAV_CMD_NAV_LAST <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAST>`_, UNSUPPORTED, common
  `MAV_CMD_NAV_PATHPLANNING <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_PATHPLANNING>`_, UNSUPPORTED, common
  `MAV_CMD_NAV_RALLY_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RALLY_POINT>`_, UNSUPPORTED, common
  `MAV_CMD_NAV_ROI <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_ROI>`_, UNSUPPORTED, common
  `MAV_CMD_OBLIQUE_SURVEY <https://mavlink.io/en/messages/common.html#MAV_CMD_OBLIQUE_SURVEY>`_, UNSUPPORTED, common
  `MAV_CMD_OVERRIDE_GOTO <https://mavlink.io/en/messages/common.html#MAV_CMD_OVERRIDE_GOTO>`_, UNSUPPORTED, common
  `MAV_CMD_PANORAMA_CREATE <https://mavlink.io/en/messages/common.html#MAV_CMD_PANORAMA_CREATE>`_, UNSUPPORTED, common
  `MAV_CMD_PAYLOAD_CONTROL_DEPLOY <https://mavlink.io/en/messages/common.html#MAV_CMD_PAYLOAD_CONTROL_DEPLOY>`_, UNSUPPORTED, common
  `MAV_CMD_PAYLOAD_PREPARE_DEPLOY <https://mavlink.io/en/messages/common.html#MAV_CMD_PAYLOAD_PREPARE_DEPLOY>`_, UNSUPPORTED, common
  `MAV_CMD_POWER_OFF_INITIATED <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_POWER_OFF_INITIATED>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS>`_, UNSUPPORTED, common
  `MAV_CMD_REQUEST_CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_INFORMATION>`_, UNSUPPORTED, common
  `MAV_CMD_REQUEST_CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_SETTINGS>`_, UNSUPPORTED, common
  `MAV_CMD_REQUEST_FLIGHT_INFORMATION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_FLIGHT_INFORMATION>`_, UNSUPPORTED, common
  `MAV_CMD_REQUEST_PROTOCOL_VERSION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_PROTOCOL_VERSION>`_, UNSUPPORTED, common
  `MAV_CMD_REQUEST_STORAGE_INFORMATION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_STORAGE_INFORMATION>`_, UNSUPPORTED, common
  `MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION>`_, UNSUPPORTED, common
  `MAV_CMD_REQUEST_VIDEO_STREAM_STATUS <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_VIDEO_STREAM_STATUS>`_, UNSUPPORTED, common
  `MAV_CMD_RESET_CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#MAV_CMD_RESET_CAMERA_SETTINGS>`_, UNSUPPORTED, common
  `MAV_CMD_SET_CAMERA_MODE <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_MODE>`_, UNSUPPORTED, common
  `MAV_CMD_SET_FACTORY_TEST_MODE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SET_FACTORY_TEST_MODE>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE>`_, UNSUPPORTED, common
  `MAV_CMD_SET_GUIDED_SUBMODE_STANDARD <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_GUIDED_SUBMODE_STANDARD>`_, UNSUPPORTED, common
  `MAV_CMD_SET_STORAGE_USAGE <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_STORAGE_USAGE>`_, UNSUPPORTED, common
  `MAV_CMD_SOLO_BTN_FLY_CLICK <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SOLO_BTN_FLY_CLICK>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_SOLO_BTN_FLY_HOLD <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SOLO_BTN_FLY_HOLD>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_SOLO_BTN_PAUSE_CLICK <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SOLO_BTN_PAUSE_CLICK>`_, UNSUPPORTED, ardupilotmega
  `MAV_CMD_SPATIAL_USER_1 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_1>`_, UNSUPPORTED, common
  `MAV_CMD_SPATIAL_USER_2 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_2>`_, UNSUPPORTED, common
  `MAV_CMD_SPATIAL_USER_3 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_3>`_, UNSUPPORTED, common
  `MAV_CMD_SPATIAL_USER_4 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_4>`_, UNSUPPORTED, common
  `MAV_CMD_SPATIAL_USER_5 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_5>`_, UNSUPPORTED, common
  `MAV_CMD_UAVCAN_GET_NODE_INFO <https://mavlink.io/en/messages/common.html#MAV_CMD_UAVCAN_GET_NODE_INFO>`_, UNSUPPORTED, common
  `MAV_CMD_USER_1 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_1>`_, UNSUPPORTED, common
  `MAV_CMD_USER_2 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_2>`_, UNSUPPORTED, common
  `MAV_CMD_USER_3 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_3>`_, UNSUPPORTED, common
  `MAV_CMD_USER_4 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_4>`_, UNSUPPORTED, common
  `MAV_CMD_USER_5 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_5>`_, UNSUPPORTED, common
  `MAV_CMD_VIDEO_START_STREAMING <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_START_STREAMING>`_, UNSUPPORTED, common
  `MAV_CMD_VIDEO_STOP_STREAMING <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_STOP_STREAMING>`_, UNSUPPORTED, common
  `MAV_CMD_WAYPOINT_USER_1 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_1>`_, UNSUPPORTED, common
  `MAV_CMD_WAYPOINT_USER_2 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_2>`_, UNSUPPORTED, common
  `MAV_CMD_WAYPOINT_USER_3 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_3>`_, UNSUPPORTED, common
  `MAV_CMD_WAYPOINT_USER_4 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_4>`_, UNSUPPORTED, common
  `MAV_CMD_WAYPOINT_USER_5 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_5>`_, UNSUPPORTED, common
