.. _ArduCopter_MAVLink_messages:

================
MAVLink Messages
================


The `MAVLink <https://mavlink.io/en/>`_ protocol supports a variety of features and functionalities, but not all `messages <https://mavlink.io/en/messages/>`_ or `commands <https://mavlink.io/en/services/command.html>`_ are implemented by the ArduPilot ecosystem, or relevant to a particular autopilot firmware.

This page is auto-generated from analysing the ArduCopter source code, and provides an indication of which messages (and commands) are handled by, requestable from, and sent from the firmware. A message being handled does not guarantee full support, but at least shows that the autopilot is aware it exists, and will try to do something meaningful with it.

Known :ref:`unsupported messages <ArduCopter_mavlink_missing_messages>` (and commands) are shown at the end.

The autopilot includes a set of :ref:`ArduCopter_mavlink_stream_groups` for convenience, which allow configuring the stream rates of groups of requestable messages by setting parameter values. It is also possible to manually request messages, and request individual messages be streamed at a specified rate. 


.. _ArduCopter_mavlink_incoming_messages:

Incoming Messages
=================

Messages the autopilot handles when received.

.. csv-table::
  :header: MAVLink number, MAVLink Message, Code Source, MAVLink Dialect


  #246,  `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, common
  #138,  `ATT_POS_MOCAP <https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #183,  `AUTOPILOT_VERSION_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#AUTOPILOT_VERSION_REQUEST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #259,  `CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #387,  `CANFD_FRAME <https://mavlink.io/en/messages/common.html#CANFD_FRAME>`_, `AP_CANManager/AP_CANManager.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CANManager/AP_CANManager.cpp>`_, common
  #388,  `CAN_FILTER_MODIFY <https://mavlink.io/en/messages/common.html#CAN_FILTER_MODIFY>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #386,  `CAN_FRAME <https://mavlink.io/en/messages/common.html#CAN_FRAME>`_, `AP_CANManager/AP_CANManager.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CANManager/AP_CANManager.cpp>`_, common
  #77,  `COMMAND_ACK <https://mavlink.io/en/messages/common.html#COMMAND_ACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #75,  `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #76,  `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #172,  `DATA96 <https://mavlink.io/en/messages/ardupilotmega.html#DATA96>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #11000,  `DEVICE_OP_READ <https://mavlink.io/en/messages/ardupilotmega.html#DEVICE_OP_READ>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #11002,  `DEVICE_OP_WRITE <https://mavlink.io/en/messages/ardupilotmega.html#DEVICE_OP_WRITE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #155,  `DIGICAM_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#DIGICAM_CONTROL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #132,  `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #225,  `EFI_STATUS <https://mavlink.io/en/messages/common.html#EFI_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #161,  `FENCE_FETCH_POINT <https://mavlink.io/en/messages/ardupilotmega.html#FENCE_FETCH_POINT>`_, `AC_Fence/AC_PolyFence_loader.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AC_Fence/AC_PolyFence_loader.cpp>`_, ardupilotmega
  #160,  `FENCE_POINT <https://mavlink.io/en/messages/ardupilotmega.html#FENCE_POINT>`_, `AC_Fence/AC_PolyFence_loader.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AC_Fence/AC_PolyFence_loader.cpp>`_, ardupilotmega
  #110,  `FILE_TRANSFER_PROTOCOL <https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #144,  `FOLLOW_TARGET <https://mavlink.io/en/messages/common.html#FOLLOW_TARGET>`_, `AP_Follow/AP_Follow.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Follow/AP_Follow.cpp>`_, common
  #285,  `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  #283,  `GIMBAL_DEVICE_INFORMATION <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  #282,  `GIMBAL_MANAGER_SET_ATTITUDE <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_ATTITUDE>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  #287,  `GIMBAL_MANAGER_SET_PITCHYAW <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  #200,  `GIMBAL_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_REPORT>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, ardupilotmega
  #33,  `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`_, `AP_Follow/AP_Follow.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Follow/AP_Follow.cpp>`_, common
  #101,  `GLOBAL_VISION_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #215,  `GOPRO_HEARTBEAT <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_HEARTBEAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #123,  `GPS_INJECT_DATA <https://mavlink.io/en/messages/common.html#GPS_INJECT_DATA>`_, `AP_GPS/AP_GPS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS.cpp>`_, common
  #232,  `GPS_INPUT <https://mavlink.io/en/messages/common.html#GPS_INPUT>`_, `AP_GPS/AP_GPS_MAV.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS_MAV.cpp>`_, common
  #233,  `GPS_RTCM_DATA <https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA>`_, `AP_GPS/AP_GPS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS.cpp>`_, common
  #0,  `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #113,  `HIL_GPS <https://mavlink.io/en/messages/common.html#HIL_GPS>`_, `AP_GPS/AP_GPS_MAV.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS_MAV.cpp>`_, common
  #149,  `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #186,  `LED_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#LED_CONTROL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #121,  `LOG_ERASE <https://mavlink.io/en/messages/common.html#LOG_ERASE>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  #119,  `LOG_REQUEST_DATA <https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  #122,  `LOG_REQUEST_END <https://mavlink.io/en/messages/common.html#LOG_REQUEST_END>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  #117,  `LOG_REQUEST_LIST <https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  #69,  `MANUAL_CONTROL <https://mavlink.io/en/messages/common.html#MANUAL_CONTROL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #47,  `MISSION_ACK <https://mavlink.io/en/messages/common.html#MISSION_ACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #45,  `MISSION_CLEAR_ALL <https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #44,  `MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #39,  `MISSION_ITEM <https://mavlink.io/en/messages/common.html#MISSION_ITEM>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #73,  `MISSION_ITEM_INT <https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #40,  `MISSION_REQUEST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #51,  `MISSION_REQUEST_INT <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #43,  `MISSION_REQUEST_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #41,  `MISSION_SET_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #38,  `MISSION_WRITE_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #156,  `MOUNT_CONFIGURE <https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_CONFIGURE>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, ardupilotmega
  #157,  `MOUNT_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_CONTROL>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, ardupilotmega
  #251,  `NAMED_VALUE_FLOAT <https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #252,  `NAMED_VALUE_INT <https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #330,  `OBSTACLE_DISTANCE <https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11037,  `OBSTACLE_DISTANCE_3D <https://mavlink.io/en/messages/ardupilotmega.html#OBSTACLE_DISTANCE_3D>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #331,  `ODOMETRY <https://mavlink.io/en/messages/common.html#ODOMETRY>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #12918,  `OPEN_DRONE_ID_ARM_STATUS <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_ARM_STATUS>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  #12900,  `OPEN_DRONE_ID_BASIC_ID <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  #12905,  `OPEN_DRONE_ID_OPERATOR_ID <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_OPERATOR_ID>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  #12903,  `OPEN_DRONE_ID_SELF_ID <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SELF_ID>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  #12904,  `OPEN_DRONE_ID_SYSTEM <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SYSTEM>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  #12919,  `OPEN_DRONE_ID_SYSTEM_UPDATE <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SYSTEM_UPDATE>`_, `AP_OpenDroneID/AP_OpenDroneID.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OpenDroneID/AP_OpenDroneID.cpp>`_, common
  #100,  `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11033,  `OSD_PARAM_CONFIG <https://mavlink.io/en/messages/ardupilotmega.html#OSD_PARAM_CONFIG>`_, `AP_OSD/AP_OSD.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OSD/AP_OSD.cpp>`_, ardupilotmega
  #11035,  `OSD_PARAM_SHOW_CONFIG <https://mavlink.io/en/messages/ardupilotmega.html#OSD_PARAM_SHOW_CONFIG>`_, `AP_OSD/AP_OSD.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OSD/AP_OSD.cpp>`_, ardupilotmega
  #21,  `PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #20,  `PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #23,  `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #22,  `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #258,  `PLAY_TUNE <https://mavlink.io/en/messages/common.html#PLAY_TUNE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #166,  `RADIO <https://mavlink.io/en/messages/ardupilotmega.html#RADIO>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #420,  `RADIO_RC_CHANNELS <https://mavlink.io/en/messages/development.html#RADIO_RC_CHANNELS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, development
  #109,  `RADIO_STATUS <https://mavlink.io/en/messages/common.html#RADIO_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #176,  `RALLY_FETCH_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_FETCH_POINT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #175,  `RALLY_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_POINT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #70,  `RC_CHANNELS_OVERRIDE <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #185,  `REMOTE_LOG_BLOCK_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_BLOCK_STATUS>`_, `AP_Logger/AP_Logger.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger.cpp>`_, ardupilotmega
  #66,  `REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11004,  `SECURE_COMMAND <https://mavlink.io/en/messages/ardupilotmega.html#SECURE_COMMAND>`_, `AP_CheckFirmware/AP_CheckFirmware_secure_command.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CheckFirmware/AP_CheckFirmware_secure_command.cpp>`_, ardupilotmega
  #11005,  `SECURE_COMMAND_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#SECURE_COMMAND_REPLY>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #126,  `SERIAL_CONTROL <https://mavlink.io/en/messages/common.html#SERIAL_CONTROL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #256,  `SETUP_SIGNING <https://mavlink.io/en/messages/common.html#SETUP_SIGNING>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #82,  `SET_ATTITUDE_TARGET <https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #48,  `SET_GPS_GLOBAL_ORIGIN <https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11,  `SET_MODE <https://mavlink.io/en/messages/common.html#SET_MODE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #86,  `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #84,  `SET_POSITION_TARGET_LOCAL_NED <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #253,  `STATUSTEXT <https://mavlink.io/en/messages/common.html#STATUSTEXT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #2,  `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #135,  `TERRAIN_CHECK <https://mavlink.io/en/messages/common.html#TERRAIN_CHECK>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #134,  `TERRAIN_DATA <https://mavlink.io/en/messages/common.html#TERRAIN_DATA>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #111,  `TIMESYNC <https://mavlink.io/en/messages/common.html#TIMESYNC>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #10001,  `UAVIONIX_ADSB_OUT_CFG <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CFG>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, uAvionix
  #10007,  `UAVIONIX_ADSB_OUT_CONTROL <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CONTROL>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, uAvionix
  #10002,  `UAVIONIX_ADSB_OUT_DYNAMIC <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_DYNAMIC>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, uAvionix
  #10003,  `UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, uAvionix
  #104,  `VICON_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#VICON_POSITION_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11011,  `VISION_POSITION_DELTA <https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #101,  `VISION_POSITION_ESTIMATE <https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #103,  `VISION_SPEED_ESTIMATE <https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common

.. _ArduCopter_mavlink_incoming_commands:

Incoming Commands
=================

Commands the autopilot handles when received.

.. csv-table::
  :header: MAVLink number, MAVLink Message, Code Source, MAVLink Dialect


  #42429,  `MAV_CMD_ACCELCAL_VEHICLE_POS <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_ACCELCAL_VEHICLE_POS>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, ardupilotmega
  #2520,  `MAV_CMD_AIRFRAME_CONFIGURATION <https://mavlink.io/en/messages/common.html#MAV_CMD_AIRFRAME_CONFIGURATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #42651,  `MAV_CMD_BATTERY_RESET <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_BATTERY_RESET>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, ardupilotmega
  #2010,  `MAV_CMD_CAMERA_STOP_TRACKING <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #2004,  `MAV_CMD_CAMERA_TRACK_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #2005,  `MAV_CMD_CAMERA_TRACK_RECTANGLE <https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #32000,  `MAV_CMD_CAN_FORWARD <https://mavlink.io/en/messages/common.html#MAV_CMD_CAN_FORWARD>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #400,  `MAV_CMD_COMPONENT_ARM_DISARM <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #112,  `MAV_CMD_CONDITION_DELAY <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_DELAY>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #114,  `MAV_CMD_CONDITION_DISTANCE <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_DISTANCE>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #115,  `MAV_CMD_CONDITION_YAW <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_YAW>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #2600,  `MAV_CMD_CONTROL_HIGH_LATENCY <https://mavlink.io/en/messages/common.html#MAV_CMD_CONTROL_HIGH_LATENCY>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #42700,  `MAV_CMD_DEBUG_TRAP <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DEBUG_TRAP>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #42425,  `MAV_CMD_DO_ACCEPT_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_ACCEPT_MAG_CAL>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, ardupilotmega
  #10001,  `MAV_CMD_DO_ADSB_OUT_IDENT <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ADSB_OUT_IDENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #212,  `MAV_CMD_DO_AUTOTUNE_ENABLE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_AUTOTUNE_ENABLE>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #218,  `MAV_CMD_DO_AUX_FUNCTION <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_AUX_FUNCTION>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  #42426,  `MAV_CMD_DO_CANCEL_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_CANCEL_MAG_CAL>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, ardupilotmega
  #178,  `MAV_CMD_DO_CHANGE_SPEED <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #202,  `MAV_CMD_DO_DIGICAM_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_DIGICAM_CONFIGURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #203,  `MAV_CMD_DO_DIGICAM_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_DIGICAM_CONTROL>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #223,  `MAV_CMD_DO_ENGINE_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ENGINE_CONTROL>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #207,  `MAV_CMD_DO_FENCE_ENABLE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FENCE_ENABLE>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #185,  `MAV_CMD_DO_FLIGHTTERMINATION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #32,  `MAV_CMD_DO_FOLLOW <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FOLLOW>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #1001,  `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  #1000,  `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  #191,  `MAV_CMD_DO_GO_AROUND <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GO_AROUND>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #211,  `MAV_CMD_DO_GRIPPER <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GRIPPER>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #222,  `MAV_CMD_DO_GUIDED_LIMITS <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GUIDED_LIMITS>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #210,  `MAV_CMD_DO_INVERTED_FLIGHT <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_INVERTED_FLIGHT>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #177,  `MAV_CMD_DO_JUMP <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_JUMP>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #601,  `MAV_CMD_DO_JUMP_TAG <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_JUMP_TAG>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #189,  `MAV_CMD_DO_LAND_START <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #209,  `MAV_CMD_DO_MOTOR_TEST <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOTOR_TEST>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #204,  `MAV_CMD_DO_MOUNT_CONFIGURE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  #205,  `MAV_CMD_DO_MOUNT_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #208,  `MAV_CMD_DO_PARACHUTE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PARACHUTE>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #193,  `MAV_CMD_DO_PAUSE_CONTINUE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #182,  `MAV_CMD_DO_REPEAT_RELAY <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPEAT_RELAY>`_, `AP_Mission/AP_Mission_Commands.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission_Commands.cpp>`_, common
  #184,  `MAV_CMD_DO_REPEAT_SERVO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPEAT_SERVO>`_, `AP_Mission/AP_Mission_Commands.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission_Commands.cpp>`_, common
  #192,  `MAV_CMD_DO_REPOSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #188,  `MAV_CMD_DO_RETURN_PATH_START <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_RETURN_PATH_START>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #42428,  `MAV_CMD_DO_SEND_BANNER <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SEND_BANNER>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #217,  `MAV_CMD_DO_SEND_SCRIPT_MESSAGE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SEND_SCRIPT_MESSAGE>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  #206,  `MAV_CMD_DO_SET_CAM_TRIGG_DIST <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_DIST>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #179,  `MAV_CMD_DO_SET_HOME <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #224,  `MAV_CMD_DO_SET_MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MISSION_CURRENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #176,  `MAV_CMD_DO_SET_MODE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #181,  `MAV_CMD_DO_SET_RELAY <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_RELAY>`_, `AP_Mission/AP_Mission_Commands.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission_Commands.cpp>`_, common
  #215,  `MAV_CMD_DO_SET_RESUME_REPEAT_DIST <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SET_RESUME_REPEAT_DIST>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  #194,  `MAV_CMD_DO_SET_REVERSE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_REVERSE>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #201,  `MAV_CMD_DO_SET_ROI <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #195,  `MAV_CMD_DO_SET_ROI_LOCATION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #197,  `MAV_CMD_DO_SET_ROI_NONE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #198,  `MAV_CMD_DO_SET_ROI_SYSID <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID>`_, `AP_Mount/AP_Mount.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount.cpp>`_, common
  #5300,  `MAV_CMD_DO_SET_SAFETY_SWITCH_STATE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SAFETY_SWITCH_STATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #183,  `MAV_CMD_DO_SET_SERVO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO>`_, `AP_Mission/AP_Mission_Commands.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission_Commands.cpp>`_, common
  #216,  `MAV_CMD_DO_SPRAYER <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SPRAYER>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  #42424,  `MAV_CMD_DO_START_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_START_MAG_CAL>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, ardupilotmega
  #3000,  `MAV_CMD_DO_VTOL_TRANSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #42600,  `MAV_CMD_DO_WINCH <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_WINCH>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  ,  `MAV_CMD_EXTERNAL_POSITION_ESTIMATE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_EXTERNAL_POSITION_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #43004,  `MAV_CMD_EXTERNAL_WIND_ESTIMATE <https://mavlink.io/en/messages/development.html#MAV_CMD_EXTERNAL_WIND_ESTIMATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, development
  #42006,  `MAV_CMD_FIXED_MAG_CAL_YAW <https://mavlink.io/en/messages/common.html#MAV_CMD_FIXED_MAG_CAL_YAW>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #42650,  `MAV_CMD_FLASH_BOOTLOADER <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_FLASH_BOOTLOADER>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, ardupilotmega
  #410,  `MAV_CMD_GET_HOME_POSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_GET_HOME_POSITION>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #510,  `MAV_CMD_GET_MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_CMD_GET_MESSAGE_INTERVAL>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #2000,  `MAV_CMD_IMAGE_START_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #2001,  `MAV_CMD_IMAGE_STOP_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_STOP_CAPTURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #600,  `MAV_CMD_JUMP_TAG <https://mavlink.io/en/messages/common.html#MAV_CMD_JUMP_TAG>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #300,  `MAV_CMD_MISSION_START <https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #83,  `MAV_CMD_NAV_ALTITUDE_WAIT <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_NAV_ALTITUDE_WAIT>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, ardupilotmega
  #42703,  `MAV_CMD_NAV_ATTITUDE_TIME <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_NAV_ATTITUDE_TIME>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, ardupilotmega
  #30,  `MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #93,  `MAV_CMD_NAV_DELAY <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_DELAY>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #5004,  `MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  #5003,  `MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  #5002,  `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  #5001,  `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  #5000,  `MAV_CMD_NAV_FENCE_RETURN_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_RETURN_POINT>`_, `GCS_MAVLink/MissionItemProtocol_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp>`_, common
  #92,  `MAV_CMD_NAV_GUIDED_ENABLE <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_GUIDED_ENABLE>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #21,  `MAV_CMD_NAV_LAND <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #19,  `MAV_CMD_NAV_LOITER_TIME <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TIME>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #31,  `MAV_CMD_NAV_LOITER_TO_ALT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #18,  `MAV_CMD_NAV_LOITER_TURNS <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TURNS>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #17,  `MAV_CMD_NAV_LOITER_UNLIM <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_UNLIM>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #94,  `MAV_CMD_NAV_PAYLOAD_PLACE <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_PAYLOAD_PLACE>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #20,  `MAV_CMD_NAV_RETURN_TO_LAUNCH <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RETURN_TO_LAUNCH>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #42702,  `MAV_CMD_NAV_SCRIPT_TIME <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_NAV_SCRIPT_TIME>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, ardupilotmega
  #213,  `MAV_CMD_NAV_SET_YAW_SPEED <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_SET_YAW_SPEED>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #82,  `MAV_CMD_NAV_SPLINE_WAYPOINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_SPLINE_WAYPOINT>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #22,  `MAV_CMD_NAV_TAKEOFF <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #24,  `MAV_CMD_NAV_TAKEOFF_LOCAL <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF_LOCAL>`_, `AP_Mission/AP_Mission.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mission/AP_Mission.cpp>`_, common
  #85,  `MAV_CMD_NAV_VTOL_LAND <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #84,  `MAV_CMD_NAV_VTOL_TAKEOFF <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_TAKEOFF>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #16,  `MAV_CMD_NAV_WAYPOINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT>`_, `ArduCopter/mode_auto.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_auto.cpp>`_, common
  #241,  `MAV_CMD_PREFLIGHT_CALIBRATION <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_CALIBRATION>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #246,  `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #242,  `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #245,  `MAV_CMD_PREFLIGHT_STORAGE <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_STORAGE>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #243,  `MAV_CMD_PREFLIGHT_UAVCAN <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_UAVCAN>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #520,  `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #512,  `MAV_CMD_REQUEST_MESSAGE <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #401,  `MAV_CMD_RUN_PREARM_CHECKS <https://mavlink.io/en/messages/common.html#MAV_CMD_RUN_PREARM_CHECKS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #42701,  `MAV_CMD_SCRIPTING <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SCRIPTING>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #532,  `MAV_CMD_SET_CAMERA_FOCUS <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_FOCUS>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #534,  `MAV_CMD_SET_CAMERA_SOURCE <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_SOURCE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #531,  `MAV_CMD_SET_CAMERA_ZOOM <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_ZOOM>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #42007,  `MAV_CMD_SET_EKF_SOURCE_SET <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SET_EKF_SOURCE_SET>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #511,  `MAV_CMD_SET_MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #42001,  `MAV_CMD_SOLO_BTN_FLY_CLICK <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SOLO_BTN_FLY_CLICK>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, ardupilotmega
  #42002,  `MAV_CMD_SOLO_BTN_FLY_HOLD <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SOLO_BTN_FLY_HOLD>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, ardupilotmega
  #42003,  `MAV_CMD_SOLO_BTN_PAUSE_CLICK <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SOLO_BTN_PAUSE_CLICK>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, ardupilotmega
  #500,  `MAV_CMD_START_RX_PAIR <https://mavlink.io/en/messages/common.html#MAV_CMD_START_RX_PAIR>`_, `AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp>`_, common
  #526,  `MAV_CMD_STORAGE_FORMAT <https://mavlink.io/en/messages/common.html#MAV_CMD_STORAGE_FORMAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #2500,  `MAV_CMD_VIDEO_START_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_START_CAPTURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common
  #2501,  `MAV_CMD_VIDEO_STOP_CAPTURE <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_STOP_CAPTURE>`_, `AP_Camera/AP_Camera.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera.cpp>`_, common

.. _ArduCopter_mavlink_requestable_messages:

Requestable Messages
====================

Messages that can be requested/streamed from the autopilot.

.. csv-table::
  :header: MAVLink number, MAVLink Message, Code Source, MAVLink Dialect


  #246,  `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #163,  `AHRS <https://mavlink.io/en/messages/ardupilotmega.html#AHRS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #178,  `AHRS2 <https://mavlink.io/en/messages/ardupilotmega.html#AHRS2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #295,  `AIRSPEED <https://mavlink.io/en/messages/development.html#AIRSPEED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, development
  #301,  `AIS_VESSEL <https://mavlink.io/en/messages/common.html#AIS_VESSEL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11020,  `AOA_SSA <https://mavlink.io/en/messages/ardupilotmega.html#AOA_SSA>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #30,  `ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #31,  `ATTITUDE_QUATERNION <https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #82,  `ATTITUDE_TARGET <https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #286,  `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE <https://mavlink.io/en/messages/common.html#AUTOPILOT_STATE_FOR_GIMBAL_DEVICE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #148,  `AUTOPILOT_VERSION <https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #181,  `BATTERY2 <https://mavlink.io/en/messages/ardupilotmega.html#BATTERY2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #147,  `BATTERY_STATUS <https://mavlink.io/en/messages/common.html#BATTERY_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #262,  `CAMERA_CAPTURE_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_CAPTURE_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #180,  `CAMERA_FEEDBACK <https://mavlink.io/en/messages/ardupilotmega.html#CAMERA_FEEDBACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #271,  `CAMERA_FOV_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_FOV_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #259,  `CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #260,  `CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#CAMERA_SETTINGS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #277,  `CAMERA_THERMAL_RANGE <https://mavlink.io/en/messages/common.html#CAMERA_THERMAL_RANGE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #195,  `DEEPSTALL <https://mavlink.io/en/messages/ardupilotmega.html#DEEPSTALL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #132,  `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #225,  `EFI_STATUS <https://mavlink.io/en/messages/common.html#EFI_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #193,  `EKF_STATUS_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #11030,  `ESC_TELEMETRY_1_TO_4 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #245,  `EXTENDED_SYS_STATE <https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #162,  `FENCE_STATUS <https://mavlink.io/en/messages/common.html#FENCE_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #373,  `GENERATOR_STATUS <https://mavlink.io/en/messages/common.html#GENERATOR_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #285,  `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #280,  `GIMBAL_MANAGER_INFORMATION <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #281,  `GIMBAL_MANAGER_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #33,  `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #124,  `GPS2_RAW <https://mavlink.io/en/messages/common.html#GPS2_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #128,  `GPS2_RTK <https://mavlink.io/en/messages/common.html#GPS2_RTK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #48,  `GPS_GLOBAL_ORIGIN <https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #24,  `GPS_RAW_INT <https://mavlink.io/en/messages/common.html#GPS_RAW_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #127,  `GPS_RTK <https://mavlink.io/en/messages/common.html#GPS_RTK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #0,  `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #105,  `HIGHRES_IMU <https://mavlink.io/en/messages/common.html#HIGHRES_IMU>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #235,  `HIGH_LATENCY2 <https://mavlink.io/en/messages/common.html#HIGH_LATENCY2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #242,  `HOME_POSITION <https://mavlink.io/en/messages/common.html#HOME_POSITION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #165,  `HWSTATUS <https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #32,  `LOCAL_POSITION_NED <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #191,  `MAG_CAL_PROGRESS <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #192,  `MAG_CAL_REPORT <https://mavlink.io/en/messages/common.html#MAG_CAL_REPORT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11039,  `MCU_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MCU_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #152,  `MEMINFO <https://mavlink.io/en/messages/ardupilotmega.html#MEMINFO>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #42,  `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #46,  `MISSION_ITEM_REACHED <https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #62,  `NAV_CONTROLLER_OUTPUT <https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #100,  `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #22,  `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #194,  `PID_TUNING <https://mavlink.io/en/messages/ardupilotmega.html#PID_TUNING>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #86,  `POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #84,  `POSITION_TARGET_LOCAL_NED <https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #125,  `POWER_STATUS <https://mavlink.io/en/messages/common.html#POWER_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #173,  `RANGEFINDER <https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #27,  `RAW_IMU <https://mavlink.io/en/messages/common.html#RAW_IMU>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #65,  `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #35,  `RC_CHANNELS_RAW <https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #34,  `RC_CHANNELS_SCALED <https://mavlink.io/en/messages/common.html#RC_CHANNELS_SCALED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  ,  `RELAY_STATUS <https://mavlink.io/en/messages/common.html#RELAY_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #226,  `RPM <https://mavlink.io/en/messages/ardupilotmega.html#RPM>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #26,  `SCALED_IMU <https://mavlink.io/en/messages/common.html#SCALED_IMU>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #116,  `SCALED_IMU2 <https://mavlink.io/en/messages/common.html#SCALED_IMU2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #129,  `SCALED_IMU3 <https://mavlink.io/en/messages/common.html#SCALED_IMU3>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #29,  `SCALED_PRESSURE <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #137,  `SCALED_PRESSURE2 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #143,  `SCALED_PRESSURE3 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE3>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #36,  `SERVO_OUTPUT_RAW <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #164,  `SIMSTATE <https://mavlink.io/en/messages/ardupilotmega.html#SIMSTATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #108,  `SIM_STATE <https://mavlink.io/en/messages/common.html#SIM_STATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #2,  `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #1,  `SYS_STATUS <https://mavlink.io/en/messages/common.html#SYS_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #136,  `TERRAIN_REPORT <https://mavlink.io/en/messages/common.html#TERRAIN_REPORT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #133,  `TERRAIN_REQUEST <https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #10008,  `UAVIONIX_ADSB_OUT_STATUS <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, uAvionix
  #74,  `VFR_HUD <https://mavlink.io/en/messages/common.html#VFR_HUD>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #241,  `VIBRATION <https://mavlink.io/en/messages/common.html#VIBRATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #269,  `VIDEO_STREAM_INFORMATION <https://mavlink.io/en/messages/common.html#VIDEO_STREAM_INFORMATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11038,  `WATER_DEPTH <https://mavlink.io/en/messages/ardupilotmega.html#WATER_DEPTH>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #9005,  `WINCH_STATUS <https://mavlink.io/en/messages/common.html#WINCH_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #168,  `WIND <https://mavlink.io/en/messages/ardupilotmega.html#WIND>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega

.. _ArduCopter_mavlink_outgoing_messages:

Outgoing Messages
=================

Messages the autopilot will send automatically (unrequested).

.. csv-table::
  :header: MAVLink number, MAVLink Message, Code Source, MAVLink Dialect


  #246,  `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`_, `AP_ADSB/AP_ADSB.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB.cpp>`_, common
  #163,  `AHRS <https://mavlink.io/en/messages/ardupilotmega.html#AHRS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #178,  `AHRS2 <https://mavlink.io/en/messages/ardupilotmega.html#AHRS2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #153,  `AP_ADC <https://mavlink.io/en/messages/ardupilotmega.html#AP_ADC>`_, `AP_HAL_ESP32/AnalogIn.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ESP32/AnalogIn.cpp>`_, ardupilotmega
  #30,  `ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #31,  `ATTITUDE_QUATERNION <https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #82,  `ATTITUDE_TARGET <https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #286,  `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE <https://mavlink.io/en/messages/common.html#AUTOPILOT_STATE_FOR_GIMBAL_DEVICE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #148,  `AUTOPILOT_VERSION <https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #181,  `BATTERY2 <https://mavlink.io/en/messages/ardupilotmega.html#BATTERY2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #147,  `BATTERY_STATUS <https://mavlink.io/en/messages/common.html#BATTERY_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #262,  `CAMERA_CAPTURE_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_CAPTURE_STATUS>`_, `AP_Camera/AP_Camera_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_Backend.cpp>`_, common
  #180,  `CAMERA_FEEDBACK <https://mavlink.io/en/messages/ardupilotmega.html#CAMERA_FEEDBACK>`_, `AP_Camera/AP_Camera_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_Backend.cpp>`_, ardupilotmega
  #271,  `CAMERA_FOV_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_FOV_STATUS>`_, `AP_Camera/AP_Camera_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_Backend.cpp>`_, common
  #259,  `CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION>`_, `AP_Camera/AP_Camera_MAVLinkCamV2.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_MAVLinkCamV2.cpp>`_, common
  #260,  `CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#CAMERA_SETTINGS>`_, `AP_Camera/AP_Camera_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_Backend.cpp>`_, common
  #277,  `CAMERA_THERMAL_RANGE <https://mavlink.io/en/messages/common.html#CAMERA_THERMAL_RANGE>`_, `AP_Mount/AP_Mount_Siyi.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_Siyi.cpp>`_, common
  #387,  `CANFD_FRAME <https://mavlink.io/en/messages/common.html#CANFD_FRAME>`_, `AP_CANManager/AP_CANManager.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CANManager/AP_CANManager.cpp>`_, common
  #386,  `CAN_FRAME <https://mavlink.io/en/messages/common.html#CAN_FRAME>`_, `AP_CANManager/AP_CANManager.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_CANManager/AP_CANManager.cpp>`_, common
  #77,  `COMMAND_ACK <https://mavlink.io/en/messages/common.html#COMMAND_ACK>`_, `ArduCopter/compassmot.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/compassmot.cpp>`_, common
  #76,  `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`_, `AP_Mount/AP_Mount_SToRM32.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_SToRM32.cpp>`_, common
  #177,  `COMPASSMOT_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#COMPASSMOT_STATUS>`_, `ArduCopter/compassmot.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/compassmot.cpp>`_, ardupilotmega
  #169,  `DATA16 <https://mavlink.io/en/messages/ardupilotmega.html#DATA16>`_, `AP_Radio/AP_Radio_cc2500.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Radio/AP_Radio_cc2500.cpp>`_, ardupilotmega
  #195,  `DEEPSTALL <https://mavlink.io/en/messages/ardupilotmega.html#DEEPSTALL>`_, `AP_Landing/AP_Landing_Deepstall.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Landing/AP_Landing_Deepstall.cpp>`_, ardupilotmega
  #11001,  `DEVICE_OP_READ_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#DEVICE_OP_READ_REPLY>`_, `GCS_MAVLink/GCS_DeviceOp.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_DeviceOp.cpp>`_, ardupilotmega
  #11003,  `DEVICE_OP_WRITE_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#DEVICE_OP_WRITE_REPLY>`_, `GCS_MAVLink/GCS_DeviceOp.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_DeviceOp.cpp>`_, ardupilotmega
  #132,  `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #225,  `EFI_STATUS <https://mavlink.io/en/messages/common.html#EFI_STATUS>`_, `AP_EFI/AP_EFI.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_EFI/AP_EFI.cpp>`_, common
  #193,  `EKF_STATUS_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT>`_, `AP_ExternalAHRS/AP_ExternalAHRS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ExternalAHRS/AP_ExternalAHRS.cpp>`_, ardupilotmega
  #245,  `EXTENDED_SYS_STATE <https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #162,  `FENCE_STATUS <https://mavlink.io/en/messages/common.html#FENCE_STATUS>`_, `GCS_MAVLink/GCS_Fence.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Fence.cpp>`_, common
  #110,  `FILE_TRANSFER_PROTOCOL <https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL>`_, `GCS_MAVLink/GCS_FTP.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_FTP.cpp>`_, common
  #373,  `GENERATOR_STATUS <https://mavlink.io/en/messages/common.html#GENERATOR_STATUS>`_, `AP_Generator/AP_Generator_RichenPower.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Generator/AP_Generator_RichenPower.cpp>`_, common
  #201,  `GIMBAL_CONTROL <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_CONTROL>`_, `AP_Mount/SoloGimbal.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/SoloGimbal.cpp>`_, ardupilotmega
  #285,  `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`_, `AP_Mount/AP_Mount_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_Backend.cpp>`_, common
  #280,  `GIMBAL_MANAGER_INFORMATION <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION>`_, `AP_Mount/AP_Mount_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_Backend.cpp>`_, common
  #281,  `GIMBAL_MANAGER_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS>`_, `AP_Mount/AP_Mount_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/AP_Mount_Backend.cpp>`_, common
  #33,  `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #218,  `GOPRO_SET_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_SET_REQUEST>`_, `AP_Camera/AP_Camera_SoloGimbal.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Camera/AP_Camera_SoloGimbal.cpp>`_, ardupilotmega
  #124,  `GPS2_RAW <https://mavlink.io/en/messages/common.html#GPS2_RAW>`_, `AP_GPS/AP_GPS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS.cpp>`_, common
  #128,  `GPS2_RTK <https://mavlink.io/en/messages/common.html#GPS2_RTK>`_, `AP_GPS/GPS_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/GPS_Backend.cpp>`_, common
  #48,  `GPS_GLOBAL_ORIGIN <https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #24,  `GPS_RAW_INT <https://mavlink.io/en/messages/common.html#GPS_RAW_INT>`_, `AP_GPS/AP_GPS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/AP_GPS.cpp>`_, common
  #127,  `GPS_RTK <https://mavlink.io/en/messages/common.html#GPS_RTK>`_, `AP_GPS/GPS_Backend.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS/GPS_Backend.cpp>`_, common
  #0,  `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #235,  `HIGH_LATENCY2 <https://mavlink.io/en/messages/common.html#HIGH_LATENCY2>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #242,  `HOME_POSITION <https://mavlink.io/en/messages/common.html#HOME_POSITION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #165,  `HWSTATUS <https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #32,  `LOCAL_POSITION_NED <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #118,  `LOG_ENTRY <https://mavlink.io/en/messages/common.html#LOG_ENTRY>`_, `AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp>`_, common
  #191,  `MAG_CAL_PROGRESS <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, ardupilotmega
  #192,  `MAG_CAL_REPORT <https://mavlink.io/en/messages/common.html#MAG_CAL_REPORT>`_, `AP_Compass/AP_Compass_Calibration.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass/AP_Compass_Calibration.cpp>`_, common
  #11039,  `MCU_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MCU_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #152,  `MEMINFO <https://mavlink.io/en/messages/ardupilotmega.html#MEMINFO>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #244,  `MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MESSAGE_INTERVAL>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #47,  `MISSION_ACK <https://mavlink.io/en/messages/common.html#MISSION_ACK>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #44,  `MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`_, `GCS_MAVLink/MissionItemProtocol_Waypoints.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol_Waypoints.cpp>`_, common
  #42,  `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #46,  `MISSION_ITEM_REACHED <https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #40,  `MISSION_REQUEST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST>`_, `GCS_MAVLink/MissionItemProtocol.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/MissionItemProtocol.cpp>`_, common
  #251,  `NAMED_VALUE_FLOAT <https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #252,  `NAMED_VALUE_INT <https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT>`_, `ArduCopter/toy_mode.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/toy_mode.cpp>`_, common
  #62,  `NAV_CONTROLLER_OUTPUT <https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #100,  `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11034,  `OSD_PARAM_CONFIG_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#OSD_PARAM_CONFIG_REPLY>`_, `AP_OSD/AP_OSD.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OSD/AP_OSD.cpp>`_, ardupilotmega
  #11036,  `OSD_PARAM_SHOW_CONFIG_REPLY <https://mavlink.io/en/messages/ardupilotmega.html#OSD_PARAM_SHOW_CONFIG_REPLY>`_, `AP_OSD/AP_OSD.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_OSD/AP_OSD.cpp>`_, ardupilotmega
  #21,  `PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`_, `AP_Mount/SoloGimbal_Parameters.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/SoloGimbal_Parameters.cpp>`_, common
  #23,  `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`_, `AP_Mount/SoloGimbal_Parameters.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Mount/SoloGimbal_Parameters.cpp>`_, common
  #22,  `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`_, `GCS_MAVLink/GCS_Param.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Param.cpp>`_, common
  #194,  `PID_TUNING <https://mavlink.io/en/messages/ardupilotmega.html#PID_TUNING>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, ardupilotmega
  #86,  `POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #84,  `POSITION_TARGET_LOCAL_NED <https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, common
  #125,  `POWER_STATUS <https://mavlink.io/en/messages/common.html#POWER_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #175,  `RALLY_POINT <https://mavlink.io/en/messages/ardupilotmega.html#RALLY_POINT>`_, `GCS_MAVLink/GCS_Rally.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Rally.cpp>`_, ardupilotmega
  #173,  `RANGEFINDER <https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #27,  `RAW_IMU <https://mavlink.io/en/messages/common.html#RAW_IMU>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #65,  `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #35,  `RC_CHANNELS_RAW <https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  ,  `RELAY_STATUS <https://mavlink.io/en/messages/common.html#RELAY_STATUS>`_, `AP_Relay/AP_Relay.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Relay/AP_Relay.cpp>`_, common
  #226,  `RPM <https://mavlink.io/en/messages/ardupilotmega.html#RPM>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #36,  `SERVO_OUTPUT_RAW <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #86,  `SET_POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #253,  `STATUSTEXT <https://mavlink.io/en/messages/common.html#STATUSTEXT>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #2,  `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #1,  `SYS_STATUS <https://mavlink.io/en/messages/common.html#SYS_STATUS>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #136,  `TERRAIN_REPORT <https://mavlink.io/en/messages/common.html#TERRAIN_REPORT>`_, `AP_Terrain/TerrainGCS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Terrain/TerrainGCS.cpp>`_, common
  #133,  `TERRAIN_REQUEST <https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST>`_, `AP_Terrain/TerrainGCS.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Terrain/TerrainGCS.cpp>`_, common
  #111,  `TIMESYNC <https://mavlink.io/en/messages/common.html#TIMESYNC>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #10001,  `UAVIONIX_ADSB_OUT_CFG <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CFG>`_, `AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp>`_, uAvionix
  #10002,  `UAVIONIX_ADSB_OUT_DYNAMIC <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_DYNAMIC>`_, `AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp>`_, uAvionix
  #74,  `VFR_HUD <https://mavlink.io/en/messages/common.html#VFR_HUD>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #241,  `VIBRATION <https://mavlink.io/en/messages/common.html#VIBRATION>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, common
  #11038,  `WATER_DEPTH <https://mavlink.io/en/messages/ardupilotmega.html#WATER_DEPTH>`_, `GCS_MAVLink/GCS_Common.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/GCS_Common.cpp>`_, ardupilotmega
  #9005,  `WINCH_STATUS <https://mavlink.io/en/messages/common.html#WINCH_STATUS>`_, `AP_Winch/AP_Winch_Daiwa.cpp <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Winch/AP_Winch_Daiwa.cpp>`_, common
  #168,  `WIND <https://mavlink.io/en/messages/ardupilotmega.html#WIND>`_, `ArduCopter/GCS_Mavlink.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/GCS_Mavlink.cpp>`_, ardupilotmega

.. _ArduCopter_mavlink_named_floats:

Named Floats
============

Breakout of named floating-point (numerical) values sent by the autopilot.

.. csv-table::
  :header: MAVLink number, MAVLink Message, Code Source, MAVLink Dialect


  ,  `NAMED_VALUE_FLOAT:HEST <https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT>`_, `ArduCopter/mode_flowhold.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/mode_flowhold.cpp>`_, common

.. _ArduCopter_mavlink_named_ints:

Named Ints
==========

Breakout of named integer values sent by the autopilot.

.. csv-table::
  :header: MAVLink number, MAVLink Message, Code Source, MAVLink Dialect


  ,  `NAMED_VALUE_INT:SNAPSHOT <https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT>`_, `ArduCopter/toy_mode.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/toy_mode.cpp>`_, common
  ,  `NAMED_VALUE_INT:VIDEOTOG <https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT>`_, `ArduCopter/toy_mode.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/toy_mode.cpp>`_, common
  ,  `NAMED_VALUE_INT:WIFIRESET <https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT>`_, `ArduCopter/toy_mode.cpp <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter/toy_mode.cpp>`_, common

.. _ArduCopter_mavlink_stream_groups:

Stream Groups
=============

Message groups with stream rates requestable by ``SRn_*`` parameters. Messages in a group are only sent if the corresponding feature is active.

.. csv-table::
  :header: MAVLink number, MAVLink Message, Stream Group Parameter, MAVLink Dialect


  #295,  `AIRSPEED <https://mavlink.io/en/messages/development.html#AIRSPEED>`_, SRn_RAW_SENSORS, development
  #27,  `RAW_IMU <https://mavlink.io/en/messages/common.html#RAW_IMU>`_, SRn_RAW_SENSORS, common
  #116,  `SCALED_IMU2 <https://mavlink.io/en/messages/common.html#SCALED_IMU2>`_, SRn_RAW_SENSORS, common
  #129,  `SCALED_IMU3 <https://mavlink.io/en/messages/common.html#SCALED_IMU3>`_, SRn_RAW_SENSORS, common
  #29,  `SCALED_PRESSURE <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE>`_, SRn_RAW_SENSORS, common
  #137,  `SCALED_PRESSURE2 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE2>`_, SRn_RAW_SENSORS, common
  #143,  `SCALED_PRESSURE3 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE3>`_, SRn_RAW_SENSORS, common
  #162,  `FENCE_STATUS <https://mavlink.io/en/messages/common.html#FENCE_STATUS>`_, SRn_EXTENDED_STATUS, common
  #124,  `GPS2_RAW <https://mavlink.io/en/messages/common.html#GPS2_RAW>`_, SRn_EXTENDED_STATUS, common
  #128,  `GPS2_RTK <https://mavlink.io/en/messages/common.html#GPS2_RTK>`_, SRn_EXTENDED_STATUS, common
  #24,  `GPS_RAW_INT <https://mavlink.io/en/messages/common.html#GPS_RAW_INT>`_, SRn_EXTENDED_STATUS, common
  #127,  `GPS_RTK <https://mavlink.io/en/messages/common.html#GPS_RTK>`_, SRn_EXTENDED_STATUS, common
  #11039,  `MCU_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MCU_STATUS>`_, SRn_EXTENDED_STATUS, ardupilotmega
  #152,  `MEMINFO <https://mavlink.io/en/messages/ardupilotmega.html#MEMINFO>`_, SRn_EXTENDED_STATUS, ardupilotmega
  #42,  `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`_, SRn_EXTENDED_STATUS, common
  #62,  `NAV_CONTROLLER_OUTPUT <https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT>`_, SRn_EXTENDED_STATUS, common
  #86,  `POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT>`_, SRn_EXTENDED_STATUS, common
  #125,  `POWER_STATUS <https://mavlink.io/en/messages/common.html#POWER_STATUS>`_, SRn_EXTENDED_STATUS, common
  #1,  `SYS_STATUS <https://mavlink.io/en/messages/common.html#SYS_STATUS>`_, SRn_EXTENDED_STATUS, common
  #33,  `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`_, SRn_POSITION, common
  #32,  `LOCAL_POSITION_NED <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED>`_, SRn_POSITION, common
  #65,  `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`_, SRn_RC_CHANNELS, common
  #35,  `RC_CHANNELS_RAW <https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW>`_, SRn_RC_CHANNELS, common
  RC_CHANNELS_RAW_ENABLED, SRn_RC_CHANNELS, UNKNOWN
  #36,  `SERVO_OUTPUT_RAW <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>`_, SRn_RC_CHANNELS, common
  #178,  `AHRS2 <https://mavlink.io/en/messages/ardupilotmega.html#AHRS2>`_, SRn_EXTRA1, ardupilotmega
  #30,  `ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`_, SRn_EXTRA1, common
  #194,  `PID_TUNING <https://mavlink.io/en/messages/ardupilotmega.html#PID_TUNING>`_, SRn_EXTRA1, ardupilotmega
  #164,  `SIMSTATE <https://mavlink.io/en/messages/ardupilotmega.html#SIMSTATE>`_, SRn_EXTRA1, ardupilotmega
  #74,  `VFR_HUD <https://mavlink.io/en/messages/common.html#VFR_HUD>`_, SRn_EXTRA2, common
  #163,  `AHRS <https://mavlink.io/en/messages/ardupilotmega.html#AHRS>`_, SRn_EXTRA3, ardupilotmega
  #147,  `BATTERY_STATUS <https://mavlink.io/en/messages/common.html#BATTERY_STATUS>`_, SRn_EXTRA3, common
  #132,  `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`_, SRn_EXTRA3, common
  #225,  `EFI_STATUS <https://mavlink.io/en/messages/common.html#EFI_STATUS>`_, SRn_EXTRA3, common
  #193,  `EKF_STATUS_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT>`_, SRn_EXTRA3, ardupilotmega
  #11030,  `ESC_TELEMETRY_1_TO_4 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4>`_, SRn_EXTRA3, ardupilotmega
  #373,  `GENERATOR_STATUS <https://mavlink.io/en/messages/common.html#GENERATOR_STATUS>`_, SRn_EXTRA3, common
  #285,  `GIMBAL_DEVICE_ATTITUDE_STATUS <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS>`_, SRn_EXTRA3, common
  #191,  `MAG_CAL_PROGRESS <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS>`_, SRn_EXTRA3, ardupilotmega
  #192,  `MAG_CAL_REPORT <https://mavlink.io/en/messages/common.html#MAG_CAL_REPORT>`_, SRn_EXTRA3, common
  #100,  `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`_, SRn_EXTRA3, common
  #173,  `RANGEFINDER <https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER>`_, SRn_EXTRA3, ardupilotmega
  #226,  `RPM <https://mavlink.io/en/messages/ardupilotmega.html#RPM>`_, SRn_EXTRA3, ardupilotmega
  #2,  `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`_, SRn_EXTRA3, common
  #136,  `TERRAIN_REPORT <https://mavlink.io/en/messages/common.html#TERRAIN_REPORT>`_, SRn_EXTRA3, common
  #133,  `TERRAIN_REQUEST <https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST>`_, SRn_EXTRA3, common
  #241,  `VIBRATION <https://mavlink.io/en/messages/common.html#VIBRATION>`_, SRn_EXTRA3, common
  #9005,  `WINCH_STATUS <https://mavlink.io/en/messages/common.html#WINCH_STATUS>`_, SRn_EXTRA3, common
  #168,  `WIND <https://mavlink.io/en/messages/ardupilotmega.html#WIND>`_, SRn_EXTRA3, ardupilotmega
  #22,  `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`_, SRn_PARAMS, common
  #246,  `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`_, SRn_ADSB, common
  #301,  `AIS_VESSEL <https://mavlink.io/en/messages/common.html#AIS_VESSEL>`_, SRn_ADSB, common

.. _ArduCopter_mavlink_missing_messages:

Missing Messages
================

Unsupported / unhandled messages.

.. csv-table::
  :header: MAVLink number, MAVLink Message, Code Source, MAVLink Dialect


  #139,  `ACTUATOR_CONTROL_TARGET <https://mavlink.io/en/messages/common.html#ACTUATOR_CONTROL_TARGET>`_, UNSUPPORTED, common
  #375,  `ACTUATOR_OUTPUT_STATUS <https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_STATUS>`_, UNSUPPORTED, common
  #11010,  `ADAP_TUNING <https://mavlink.io/en/messages/ardupilotmega.html#ADAP_TUNING>`_, UNSUPPORTED, ardupilotmega
  #182,  `AHRS3 <https://mavlink.io/en/messages/ardupilotmega.html#AHRS3>`_, UNSUPPORTED, ardupilotmega
  ,  `AIRLINK_AUTH <https://mavlink.io/en/messages/ardupilotmega.html#AIRLINK_AUTH>`_, UNSUPPORTED, ardupilotmega
  ,  `AIRLINK_AUTH_RESPONSE <https://mavlink.io/en/messages/ardupilotmega.html#AIRLINK_AUTH_RESPONSE>`_, UNSUPPORTED, ardupilotmega
  #174,  `AIRSPEED_AUTOCAL <https://mavlink.io/en/messages/ardupilotmega.html#AIRSPEED_AUTOCAL>`_, UNSUPPORTED, ardupilotmega
  #141,  `ALTITUDE <https://mavlink.io/en/messages/common.html#ALTITUDE>`_, UNSUPPORTED, common
  #61,  `ATTITUDE_QUATERNION_COV <https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION_COV>`_, UNSUPPORTED, common
  #7,  `AUTH_KEY <https://mavlink.io/en/messages/common.html#AUTH_KEY>`_, UNSUPPORTED, common
  ,  `BAD_DATA <https://mavlink.io/en/messages/common.html#BAD_DATA>`_, UNSUPPORTED, common
  #257,  `BUTTON_CHANGE <https://mavlink.io/en/messages/common.html#BUTTON_CHANGE>`_, UNSUPPORTED, common
  #263,  `CAMERA_IMAGE_CAPTURED <https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED>`_, UNSUPPORTED, common
  #179,  `CAMERA_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#CAMERA_STATUS>`_, UNSUPPORTED, ardupilotmega
  #276,  `CAMERA_TRACKING_GEO_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_GEO_STATUS>`_, UNSUPPORTED, common
  #275,  `CAMERA_TRACKING_IMAGE_STATUS <https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_IMAGE_STATUS>`_, UNSUPPORTED, common
  #112,  `CAMERA_TRIGGER <https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER>`_, UNSUPPORTED, common
  #5,  `CHANGE_OPERATOR_CONTROL <https://mavlink.io/en/messages/common.html#CHANGE_OPERATOR_CONTROL>`_, UNSUPPORTED, common
  #6,  `CHANGE_OPERATOR_CONTROL_ACK <https://mavlink.io/en/messages/common.html#CHANGE_OPERATOR_CONTROL_ACK>`_, UNSUPPORTED, common
  #247,  `COLLISION <https://mavlink.io/en/messages/common.html#COLLISION>`_, UNSUPPORTED, common
  #146,  `CONTROL_SYSTEM_STATE <https://mavlink.io/en/messages/common.html#CONTROL_SYSTEM_STATE>`_, UNSUPPORTED, common
  #50005,  `CUBEPILOT_FIRMWARE_UPDATE_RESP <https://mavlink.io/en/messages/cubepilot.html#CUBEPILOT_FIRMWARE_UPDATE_RESP>`_, UNSUPPORTED, cubepilot
  #50004,  `CUBEPILOT_FIRMWARE_UPDATE_START <https://mavlink.io/en/messages/cubepilot.html#CUBEPILOT_FIRMWARE_UPDATE_START>`_, UNSUPPORTED, cubepilot
  #50001,  `CUBEPILOT_RAW_RC <https://mavlink.io/en/messages/cubepilot.html#CUBEPILOT_RAW_RC>`_, UNSUPPORTED, cubepilot
  #170,  `DATA32 <https://mavlink.io/en/messages/ardupilotmega.html#DATA32>`_, UNSUPPORTED, ardupilotmega
  #171,  `DATA64 <https://mavlink.io/en/messages/ardupilotmega.html#DATA64>`_, UNSUPPORTED, ardupilotmega
  #66,  `DATA_STREAM <https://mavlink.io/en/messages/common.html#DATA_STREAM>`_, UNSUPPORTED, common
  #130,  `DATA_TRANSMISSION_HANDSHAKE <https://mavlink.io/en/messages/common.html#DATA_TRANSMISSION_HANDSHAKE>`_, UNSUPPORTED, common
  #254,  `DEBUG <https://mavlink.io/en/messages/common.html#DEBUG>`_, UNSUPPORTED, common
  #350,  `DEBUG_FLOAT_ARRAY <https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY>`_, UNSUPPORTED, common
  #250,  `DEBUG_VECT <https://mavlink.io/en/messages/common.html#DEBUG_VECT>`_, UNSUPPORTED, common
  #154,  `DIGICAM_CONFIGURE <https://mavlink.io/en/messages/ardupilotmega.html#DIGICAM_CONFIGURE>`_, UNSUPPORTED, ardupilotmega
  #131,  `ENCAPSULATED_DATA <https://mavlink.io/en/messages/common.html#ENCAPSULATED_DATA>`_, UNSUPPORTED, common
  #11040,  `ESC_TELEMETRY_13_TO_16 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_13_TO_16>`_, UNSUPPORTED, ardupilotmega
  #11041,  `ESC_TELEMETRY_17_TO_20 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_17_TO_20>`_, UNSUPPORTED, ardupilotmega
  #11042,  `ESC_TELEMETRY_21_TO_24 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_21_TO_24>`_, UNSUPPORTED, ardupilotmega
  #11043,  `ESC_TELEMETRY_25_TO_28 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_25_TO_28>`_, UNSUPPORTED, ardupilotmega
  #11044,  `ESC_TELEMETRY_29_TO_32 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_29_TO_32>`_, UNSUPPORTED, ardupilotmega
  #11031,  `ESC_TELEMETRY_5_TO_8 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_5_TO_8>`_, UNSUPPORTED, ardupilotmega
  #11032,  `ESC_TELEMETRY_9_TO_12 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_9_TO_12>`_, UNSUPPORTED, ardupilotmega
  #230,  `ESTIMATOR_STATUS <https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS>`_, UNSUPPORTED, common
  #264,  `FLIGHT_INFORMATION <https://mavlink.io/en/messages/common.html#FLIGHT_INFORMATION>`_, UNSUPPORTED, common
  #284,  `GIMBAL_DEVICE_SET_ATTITUDE <https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE>`_, UNSUPPORTED, common
  #288,  `GIMBAL_MANAGER_SET_MANUAL_CONTROL <https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL>`_, UNSUPPORTED, common
  #214,  `GIMBAL_TORQUE_CMD_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_TORQUE_CMD_REPORT>`_, UNSUPPORTED, ardupilotmega
  #63,  `GLOBAL_POSITION_INT_COV <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT_COV>`_, UNSUPPORTED, common
  #441,  `GNSS_INTEGRITY <https://mavlink.io/en/messages/development.html#GNSS_INTEGRITY>`_, UNSUPPORTED, development
  #216,  `GOPRO_GET_REQUEST <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_GET_REQUEST>`_, UNSUPPORTED, ardupilotmega
  #217,  `GOPRO_GET_RESPONSE <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_GET_RESPONSE>`_, UNSUPPORTED, ardupilotmega
  #219,  `GOPRO_SET_RESPONSE <https://mavlink.io/en/messages/ardupilotmega.html#GOPRO_SET_RESPONSE>`_, UNSUPPORTED, ardupilotmega
  #25,  `GPS_STATUS <https://mavlink.io/en/messages/common.html#GPS_STATUS>`_, UNSUPPORTED, common
  #50003,  `HERELINK_TELEM <https://mavlink.io/en/messages/cubepilot.html#HERELINK_TELEM>`_, UNSUPPORTED, cubepilot
  #50002,  `HERELINK_VIDEO_STREAM_INFORMATION <https://mavlink.io/en/messages/cubepilot.html#HERELINK_VIDEO_STREAM_INFORMATION>`_, UNSUPPORTED, cubepilot
  #234,  `HIGH_LATENCY <https://mavlink.io/en/messages/common.html#HIGH_LATENCY>`_, UNSUPPORTED, common
  #93,  `HIL_ACTUATOR_CONTROLS <https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS>`_, UNSUPPORTED, common
  #91,  `HIL_CONTROLS <https://mavlink.io/en/messages/common.html#HIL_CONTROLS>`_, UNSUPPORTED, common
  #114,  `HIL_OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW>`_, UNSUPPORTED, common
  #92,  `HIL_RC_INPUTS_RAW <https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW>`_, UNSUPPORTED, common
  #107,  `HIL_SENSOR <https://mavlink.io/en/messages/common.html#HIL_SENSOR>`_, UNSUPPORTED, common
  #90,  `HIL_STATE <https://mavlink.io/en/messages/common.html#HIL_STATE>`_, UNSUPPORTED, common
  #115,  `HIL_STATE_QUATERNION <https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION>`_, UNSUPPORTED, common
  #12920,  `HYGROMETER_SENSOR <https://mavlink.io/en/messages/common.html#HYGROMETER_SENSOR>`_, UNSUPPORTED, common
  #42000,  `ICAROUS_HEARTBEAT <https://mavlink.io/en/messages/icarous.html#ICAROUS_HEARTBEAT>`_, UNSUPPORTED, icarous
  #42001,  `ICAROUS_KINEMATIC_BANDS <https://mavlink.io/en/messages/icarous.html#ICAROUS_KINEMATIC_BANDS>`_, UNSUPPORTED, icarous
  #335,  `ISBD_LINK_STATUS <https://mavlink.io/en/messages/common.html#ISBD_LINK_STATUS>`_, UNSUPPORTED, common
  #167,  `LIMITS_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#LIMITS_STATUS>`_, UNSUPPORTED, ardupilotmega
  #64,  `LOCAL_POSITION_NED_COV <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_COV>`_, UNSUPPORTED, common
  #89,  `LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET>`_, UNSUPPORTED, common
  #268,  `LOGGING_ACK <https://mavlink.io/en/messages/common.html#LOGGING_ACK>`_, UNSUPPORTED, common
  #266,  `LOGGING_DATA <https://mavlink.io/en/messages/common.html#LOGGING_DATA>`_, UNSUPPORTED, common
  #267,  `LOGGING_DATA_ACKED <https://mavlink.io/en/messages/common.html#LOGGING_DATA_ACKED>`_, UNSUPPORTED, common
  #120,  `LOG_DATA <https://mavlink.io/en/messages/common.html#LOG_DATA>`_, UNSUPPORTED, common
  ,  `LOWEHEISER_GOV_EFI <https://mavlink.io/en/messages/ardupilotmega.html#LOWEHEISER_GOV_EFI>`_, UNSUPPORTED, ardupilotmega
  #81,  `MANUAL_SETPOINT <https://mavlink.io/en/messages/common.html#MANUAL_SETPOINT>`_, UNSUPPORTED, common
  #249,  `MEMORY_VECT <https://mavlink.io/en/messages/common.html#MEMORY_VECT>`_, UNSUPPORTED, common
  ,  `MISSION_CHECKSUM <https://mavlink.io/en/messages/development.html#MISSION_CHECKSUM>`_, UNSUPPORTED, development
  #37,  `MISSION_REQUEST_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_PARTIAL_LIST>`_, UNSUPPORTED, common
  #265,  `MOUNT_ORIENTATION <https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION>`_, UNSUPPORTED, common
  #158,  `MOUNT_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_STATUS>`_, UNSUPPORTED, ardupilotmega
  #12902,  `OPEN_DRONE_ID_AUTHENTICATION <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_AUTHENTICATION>`_, UNSUPPORTED, common
  #12901,  `OPEN_DRONE_ID_LOCATION <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_LOCATION>`_, UNSUPPORTED, common
  #12915,  `OPEN_DRONE_ID_MESSAGE_PACK <https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_MESSAGE_PACK>`_, UNSUPPORTED, common
  #106,  `OPTICAL_FLOW_RAD <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD>`_, UNSUPPORTED, common
  #324,  `PARAM_EXT_ACK <https://mavlink.io/en/messages/common.html#PARAM_EXT_ACK>`_, UNSUPPORTED, common
  #321,  `PARAM_EXT_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_EXT_REQUEST_LIST>`_, UNSUPPORTED, common
  #320,  `PARAM_EXT_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_EXT_REQUEST_READ>`_, UNSUPPORTED, common
  #323,  `PARAM_EXT_SET <https://mavlink.io/en/messages/common.html#PARAM_EXT_SET>`_, UNSUPPORTED, common
  #322,  `PARAM_EXT_VALUE <https://mavlink.io/en/messages/common.html#PARAM_EXT_VALUE>`_, UNSUPPORTED, common
  #50,  `PARAM_MAP_RC <https://mavlink.io/en/messages/common.html#PARAM_MAP_RC>`_, UNSUPPORTED, common
  #4,  `PING <https://mavlink.io/en/messages/common.html#PING>`_, UNSUPPORTED, common
  #28,  `RAW_PRESSURE <https://mavlink.io/en/messages/common.html#RAW_PRESSURE>`_, UNSUPPORTED, common
  #339,  `RAW_RPM <https://mavlink.io/en/messages/common.html#RAW_RPM>`_, UNSUPPORTED, common
  #184,  `REMOTE_LOG_DATA_BLOCK <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_DATA_BLOCK>`_, UNSUPPORTED, ardupilotmega
  #142,  `RESOURCE_REQUEST <https://mavlink.io/en/messages/common.html#RESOURCE_REQUEST>`_, UNSUPPORTED, common
  #55,  `SAFETY_ALLOWED_AREA <https://mavlink.io/en/messages/common.html#SAFETY_ALLOWED_AREA>`_, UNSUPPORTED, common
  #54,  `SAFETY_SET_ALLOWED_AREA <https://mavlink.io/en/messages/common.html#SAFETY_SET_ALLOWED_AREA>`_, UNSUPPORTED, common
  #150,  `SENSOR_OFFSETS <https://mavlink.io/en/messages/ardupilotmega.html#SENSOR_OFFSETS>`_, UNSUPPORTED, ardupilotmega
  #139,  `SET_ACTUATOR_CONTROL_TARGET <https://mavlink.io/en/messages/common.html#SET_ACTUATOR_CONTROL_TARGET>`_, UNSUPPORTED, common
  #243,  `SET_HOME_POSITION <https://mavlink.io/en/messages/common.html#SET_HOME_POSITION>`_, UNSUPPORTED, common
  #151,  `SET_MAG_OFFSETS <https://mavlink.io/en/messages/ardupilotmega.html#SET_MAG_OFFSETS>`_, UNSUPPORTED, ardupilotmega
  #370,  `SMART_BATTERY_INFO <https://mavlink.io/en/messages/common.html#SMART_BATTERY_INFO>`_, UNSUPPORTED, common
  #261,  `STORAGE_INFORMATION <https://mavlink.io/en/messages/common.html#STORAGE_INFORMATION>`_, UNSUPPORTED, common
  #333,  `TRAJECTORY_REPRESENTATION_BEZIER <https://mavlink.io/en/messages/common.html#TRAJECTORY_REPRESENTATION_BEZIER>`_, UNSUPPORTED, common
  #332,  `TRAJECTORY_REPRESENTATION_WAYPOINTS <https://mavlink.io/en/messages/common.html#TRAJECTORY_REPRESENTATION_WAYPOINTS>`_, UNSUPPORTED, common
  #385,  `TUNNEL <https://mavlink.io/en/messages/common.html#TUNNEL>`_, UNSUPPORTED, common
  #311,  `UAVCAN_NODE_INFO <https://mavlink.io/en/messages/common.html#UAVCAN_NODE_INFO>`_, UNSUPPORTED, common
  #310,  `UAVCAN_NODE_STATUS <https://mavlink.io/en/messages/common.html#UAVCAN_NODE_STATUS>`_, UNSUPPORTED, common
  #10006,  `UAVIONIX_ADSB_GET <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_GET>`_, UNSUPPORTED, uAvionix
  #10005,  `UAVIONIX_ADSB_OUT_CFG_FLIGHTID <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CFG_FLIGHTID>`_, UNSUPPORTED, uAvionix
  #10004,  `UAVIONIX_ADSB_OUT_CFG_REGISTRATION <https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_OUT_CFG_REGISTRATION>`_, UNSUPPORTED, uAvionix
  ,  `UNKNOWN <https://mavlink.io/en/messages/common.html#UNKNOWN>`_, UNSUPPORTED, common
  #340,  `UTM_GLOBAL_POSITION <https://mavlink.io/en/messages/common.html#UTM_GLOBAL_POSITION>`_, UNSUPPORTED, common
  #248,  `V2_EXTENSION <https://mavlink.io/en/messages/common.html#V2_EXTENSION>`_, UNSUPPORTED, common
  #270,  `VIDEO_STREAM_STATUS <https://mavlink.io/en/messages/common.html#VIDEO_STREAM_STATUS>`_, UNSUPPORTED, common
  #9000,  `WHEEL_DISTANCE <https://mavlink.io/en/messages/common.html#WHEEL_DISTANCE>`_, UNSUPPORTED, common
  #299,  `WIFI_CONFIG_AP <https://mavlink.io/en/messages/common.html#WIFI_CONFIG_AP>`_, UNSUPPORTED, common
  #231,  `WIND_COV <https://mavlink.io/en/messages/common.html#WIND_COV>`_, UNSUPPORTED, common

.. _ArduCopter_mavlink_missing_commands:

Missing Commands
================

Unsupported / unhandled commands.

.. csv-table::
  :header: MAVLink number, MAVLink Message, Code Source, MAVLink Dialect


  #3001,  `MAV_CMD_ARM_AUTHORIZATION_REQUEST <https://mavlink.io/en/messages/common.html#MAV_CMD_ARM_AUTHORIZATION_REQUEST>`_, UNSUPPORTED, common
  #113,  `MAV_CMD_CONDITION_CHANGE_ALT <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_CHANGE_ALT>`_, UNSUPPORTED, common
  #159,  `MAV_CMD_CONDITION_LAST <https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_LAST>`_, UNSUPPORTED, common
  #186,  `MAV_CMD_DO_CHANGE_ALTITUDE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_ALTITUDE>`_, UNSUPPORTED, common
  #200,  `MAV_CMD_DO_CONTROL_VIDEO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CONTROL_VIDEO>`_, UNSUPPORTED, common
  #33,  `MAV_CMD_DO_FOLLOW_REPOSITION <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FOLLOW_REPOSITION>`_, UNSUPPORTED, common
  #221,  `MAV_CMD_DO_GUIDED_MASTER <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GUIDED_MASTER>`_, UNSUPPORTED, common
  #240,  `MAV_CMD_DO_LAST <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAST>`_, UNSUPPORTED, common
  #220,  `MAV_CMD_DO_MOUNT_CONTROL_QUAT <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL_QUAT>`_, UNSUPPORTED, common
  #190,  `MAV_CMD_DO_RALLY_LAND <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_RALLY_LAND>`_, UNSUPPORTED, common
  #214,  `MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL>`_, UNSUPPORTED, common
  #180,  `MAV_CMD_DO_SET_PARAMETER <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_PARAMETER>`_, UNSUPPORTED, common
  #196,  `MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET>`_, UNSUPPORTED, common
  #610,  `MAV_CMD_DO_SET_SYS_CMP_ID <https://mavlink.io/en/messages/development.html#MAV_CMD_DO_SET_SYS_CMP_ID>`_, UNSUPPORTED, development
  #2003,  `MAV_CMD_DO_TRIGGER_CONTROL <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_TRIGGER_CONTROL>`_, UNSUPPORTED, common
  ,  `MAV_CMD_ENUM_END <https://mavlink.io/en/messages/common.html#MAV_CMD_ENUM_END>`_, UNSUPPORTED, common
  #42004,  `MAV_CMD_FIXED_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_FIXED_MAG_CAL>`_, UNSUPPORTED, ardupilotmega
  #42005,  `MAV_CMD_FIXED_MAG_CAL_FIELD <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_FIXED_MAG_CAL_FIELD>`_, UNSUPPORTED, ardupilotmega
  #42502,  `MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS>`_, UNSUPPORTED, ardupilotmega
  #42505,  `MAV_CMD_GIMBAL_FULL_RESET <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GIMBAL_FULL_RESET>`_, UNSUPPORTED, ardupilotmega
  #42503,  `MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION>`_, UNSUPPORTED, ardupilotmega
  #42501,  `MAV_CMD_GIMBAL_RESET <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GIMBAL_RESET>`_, UNSUPPORTED, ardupilotmega
  #43001,  `MAV_CMD_GUIDED_CHANGE_ALTITUDE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_ALTITUDE>`_, UNSUPPORTED, ardupilotmega
  #43002,  `MAV_CMD_GUIDED_CHANGE_HEADING <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_HEADING>`_, UNSUPPORTED, ardupilotmega
  #43000,  `MAV_CMD_GUIDED_CHANGE_SPEED <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_SPEED>`_, UNSUPPORTED, ardupilotmega
  #2510,  `MAV_CMD_LOGGING_START <https://mavlink.io/en/messages/common.html#MAV_CMD_LOGGING_START>`_, UNSUPPORTED, common
  #2511,  `MAV_CMD_LOGGING_STOP <https://mavlink.io/en/messages/common.html#MAV_CMD_LOGGING_STOP>`_, UNSUPPORTED, common
  ,  `MAV_CMD_LOWEHEISER_SET_STATE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_LOWEHEISER_SET_STATE>`_, UNSUPPORTED, ardupilotmega
  #25,  `MAV_CMD_NAV_FOLLOW <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FOLLOW>`_, UNSUPPORTED, common
  #23,  `MAV_CMD_NAV_LAND_LOCAL <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND_LOCAL>`_, UNSUPPORTED, common
  #95,  `MAV_CMD_NAV_LAST <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAST>`_, UNSUPPORTED, common
  #81,  `MAV_CMD_NAV_PATHPLANNING <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_PATHPLANNING>`_, UNSUPPORTED, common
  #5100,  `MAV_CMD_NAV_RALLY_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RALLY_POINT>`_, UNSUPPORTED, common
  #80,  `MAV_CMD_NAV_ROI <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_ROI>`_, UNSUPPORTED, common
  #260,  `MAV_CMD_OBLIQUE_SURVEY <https://mavlink.io/en/messages/common.html#MAV_CMD_OBLIQUE_SURVEY>`_, UNSUPPORTED, common
  #252,  `MAV_CMD_OVERRIDE_GOTO <https://mavlink.io/en/messages/common.html#MAV_CMD_OVERRIDE_GOTO>`_, UNSUPPORTED, common
  #2800,  `MAV_CMD_PANORAMA_CREATE <https://mavlink.io/en/messages/common.html#MAV_CMD_PANORAMA_CREATE>`_, UNSUPPORTED, common
  #30002,  `MAV_CMD_PAYLOAD_CONTROL_DEPLOY <https://mavlink.io/en/messages/common.html#MAV_CMD_PAYLOAD_CONTROL_DEPLOY>`_, UNSUPPORTED, common
  #30001,  `MAV_CMD_PAYLOAD_PREPARE_DEPLOY <https://mavlink.io/en/messages/common.html#MAV_CMD_PAYLOAD_PREPARE_DEPLOY>`_, UNSUPPORTED, common
  #42000,  `MAV_CMD_POWER_OFF_INITIATED <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_POWER_OFF_INITIATED>`_, UNSUPPORTED, ardupilotmega
  #527,  `MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS>`_, UNSUPPORTED, common
  #521,  `MAV_CMD_REQUEST_CAMERA_INFORMATION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_INFORMATION>`_, UNSUPPORTED, common
  #522,  `MAV_CMD_REQUEST_CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_CAMERA_SETTINGS>`_, UNSUPPORTED, common
  #528,  `MAV_CMD_REQUEST_FLIGHT_INFORMATION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_FLIGHT_INFORMATION>`_, UNSUPPORTED, common
  #519,  `MAV_CMD_REQUEST_PROTOCOL_VERSION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_PROTOCOL_VERSION>`_, UNSUPPORTED, common
  #525,  `MAV_CMD_REQUEST_STORAGE_INFORMATION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_STORAGE_INFORMATION>`_, UNSUPPORTED, common
  #2504,  `MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION>`_, UNSUPPORTED, common
  #2505,  `MAV_CMD_REQUEST_VIDEO_STREAM_STATUS <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_VIDEO_STREAM_STATUS>`_, UNSUPPORTED, common
  #529,  `MAV_CMD_RESET_CAMERA_SETTINGS <https://mavlink.io/en/messages/common.html#MAV_CMD_RESET_CAMERA_SETTINGS>`_, UNSUPPORTED, common
  #530,  `MAV_CMD_SET_CAMERA_MODE <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_MODE>`_, UNSUPPORTED, common
  #42427,  `MAV_CMD_SET_FACTORY_TEST_MODE <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SET_FACTORY_TEST_MODE>`_, UNSUPPORTED, ardupilotmega
  #4001,  `MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE>`_, UNSUPPORTED, common
  #4000,  `MAV_CMD_SET_GUIDED_SUBMODE_STANDARD <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_GUIDED_SUBMODE_STANDARD>`_, UNSUPPORTED, common
  #43005,  `MAV_CMD_SET_HAGL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SET_HAGL>`_, UNSUPPORTED, ardupilotmega
  #533,  `MAV_CMD_SET_STORAGE_USAGE <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_STORAGE_USAGE>`_, UNSUPPORTED, common
  #31005,  `MAV_CMD_SPATIAL_USER_1 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_1>`_, UNSUPPORTED, common
  #31006,  `MAV_CMD_SPATIAL_USER_2 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_2>`_, UNSUPPORTED, common
  #31007,  `MAV_CMD_SPATIAL_USER_3 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_3>`_, UNSUPPORTED, common
  #31008,  `MAV_CMD_SPATIAL_USER_4 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_4>`_, UNSUPPORTED, common
  #31009,  `MAV_CMD_SPATIAL_USER_5 <https://mavlink.io/en/messages/common.html#MAV_CMD_SPATIAL_USER_5>`_, UNSUPPORTED, common
  #5200,  `MAV_CMD_UAVCAN_GET_NODE_INFO <https://mavlink.io/en/messages/common.html#MAV_CMD_UAVCAN_GET_NODE_INFO>`_, UNSUPPORTED, common
  #31010,  `MAV_CMD_USER_1 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_1>`_, UNSUPPORTED, common
  #31011,  `MAV_CMD_USER_2 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_2>`_, UNSUPPORTED, common
  #31012,  `MAV_CMD_USER_3 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_3>`_, UNSUPPORTED, common
  #31013,  `MAV_CMD_USER_4 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_4>`_, UNSUPPORTED, common
  #31014,  `MAV_CMD_USER_5 <https://mavlink.io/en/messages/common.html#MAV_CMD_USER_5>`_, UNSUPPORTED, common
  #2502,  `MAV_CMD_VIDEO_START_STREAMING <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_START_STREAMING>`_, UNSUPPORTED, common
  #2503,  `MAV_CMD_VIDEO_STOP_STREAMING <https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_STOP_STREAMING>`_, UNSUPPORTED, common
  #31000,  `MAV_CMD_WAYPOINT_USER_1 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_1>`_, UNSUPPORTED, common
  #31001,  `MAV_CMD_WAYPOINT_USER_2 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_2>`_, UNSUPPORTED, common
  #31002,  `MAV_CMD_WAYPOINT_USER_3 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_3>`_, UNSUPPORTED, common
  #31003,  `MAV_CMD_WAYPOINT_USER_4 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_4>`_, UNSUPPORTED, common
  #31004,  `MAV_CMD_WAYPOINT_USER_5 <https://mavlink.io/en/messages/common.html#MAV_CMD_WAYPOINT_USER_5>`_, UNSUPPORTED, common
