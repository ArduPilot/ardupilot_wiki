.. _code-overview-adding-support-for-a-new-mavlink-gimbal:

=======================================
Adding Support for a new MAVLink Gimbal
=======================================

This page covers how ArduPilot interacts with a MAVLink enabled gimbal. 
It is meant more as a guide on how gimbal manufacturers can make their
gimbals work with ArduPilot with minimum code changes to ArduPilot .

The :ref:`SToRM32 gimbal <copter:common-storm32-gimbal>` is a
good reference as it supports MAVLink.

Messages the gimbal should support
==================================

#. The gimbal should listen on the serial port for a
   `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`__ message
   from the vehicle. It can generally assume that the first heart beat
   it receives will be from the vehicle but to be certain you can check
   the "type" field to be sure it's something sensible (i.e. MAV_TYPE =
   1 for fixed wing, 2 for quadcopters but 6 for GCSs should be
   ignored).

   This heart beat will contain the vehicle's system-id and
   component-id.  The gimbal should adopt the same system-id but should
   use a component-id of MAV_COMP_ID_GIMBAL (i.e. 154) for all its
   future messages.  In fact, any unique component-id can be used as
   long as it's not zero nor the component id of the vehicle or any
   other device on the vehicle.
#. The gimbal should send a
   `HEARTBEAT <https://mavlink.io/en/messages/common.html#HEARTBEAT>`__ message
   out the serial port at approximately 1hz

   -  the system-id and component-id should be as mentioned above.
   -  "type" should be MAV_TYPE_GIMBAL (i.e. 26).
   -  "autopilot" is not used so can be set to anything including
      MAV_AUTOPILOT_GENERIC (i.e. 0)
   -  "base_mode" and "custom_mode" are not used so can be set to
      anything (perhaps 0 is best)
   -  "system_status" should be set to "MAV_STATE_ACTIVE" once the
      gimbal is ready to accept attitude targets

#. To support reading/writing parameter values from the ground station
   the gimbal should implement these message:

   -  `PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`__
      - if the gimbal receives this message and "target_system" and
      "target_component" values match the gimbal's system-id and
      component-id, it should respond with a
      `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__
      message which contains the value of the parameter specified by the
      "param_id" field (simply an enum, the gimbal can assign whatever
      enum it wishes to each of its internal paramters)
   -  `PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`__
      - respond to this message by sending a
      `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__
      message for every parameter within the gimbal.
   -  `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`__ -
      respond to this by setting the internal variable to the value in
      the "param_value" field.

#. ArduPilot will send angle requests to the gimbal via MAVLink which
   will arrive as
   `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__
   messages with the "command" field set to MAV_CMD_DO_MOUNT_CONTROL
   (i.e 205).

   -  Param #1 contains desired pitch in degrees
   -  Param #2 contains desired roll in degrees
   -  Param #3 contains desired yaw in degrees but note that the yaw is
      in relation to the front of the vehicle so "0" is straight ahead,
      "90" is to the right, "-90" is to the left.

#. If the gimbal needs extra data from the vehicle it can request it
   using the
   `REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`__
   message.

   -  the target system and component id should be for the vehicle.
   -  "req_stream_id" can be any values in the MAV_DATA_STREAM enum.
      Some useful values are:

      -  MAV_DATA_STREAM_POSITION will cause the vehicle to send
         `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`__
         messages which includes lat, lon, alt, velocity (3d) and
         heading)
      -  MAV_DATA_STREAM_EXTRA1 will send the
         `MSG_ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`__
         which includes euler angles for roll, pitch, yaw

Testing
=======

To test the system, you should be able to connect the gimbal to a
Pixhawk as if it's a SToRM32 gimbal. In particular check the `"Set-up
through the Mission Planner (MALVink
protocol)" <http://copter.ardupilot.com/wiki/common-storm32-gimbal/#set-up_through_the_mission_planner_mavlink_protocol>`__
section.
