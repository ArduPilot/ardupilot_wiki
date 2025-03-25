.. _mavlink-requesting-data:

==================================
Requesting Data From The Autopilot
==================================

The ground station or companion computer can request the data it wants (and the rate) using one of the following methods:

   - Set the ``SRx_`` parameters to cause the autopilot to pro-actively send groups of messages on start-up.  This method is easy to set-up for a small number of drones but is not recommended for most applications
   - Send `REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`__ messages to set the rate for groups of messages
   - Send a `SET_MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL>`__ command (within a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ message) to precisely control the rate of an individual message.  Note this is only supported on ArduPilot 4.0 and higher
   - Send a `REQUEST_MESSAGE <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE>`__ command (within a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ message) to request a single instance of a message.  Note this is only supported on ArduPilot 4.0 and higher
   - Create configuration files stored on the SD card or in ROMFS specifying message stream rates on a per-channel basis
   - Install a Lua script to set individual messages rates (e.g. Set Message Interval via a Lua script).  See "Using a LUA Script" section below.

Note that the stream rates will temporarily be 4 or more times slower than requested while parameters and waypoints are being sent.

More details on the above methods can be found below.

.. note::

   If you find your message rates are reverting to some fixed value after you set them, see the "Rates Reverting to some Fixed value" section at the bottom of this page

Using SRx Parameters
--------------------

Setting the ``SRx_`` parameters (and then rebooting the autopilot) will cause the autopilot to pro-actively send groups of messages to the ground station.  This is not the recommended method because the ground station has no way to determine what "x" should be.

- Connect to the autopilot with a ground station
- Determine which telemetry connection is being used.  For example if the ground station is connected to the autopilot's "Telem1" port (perhaps using a wifi or telemetry radio) then the ``SR1_`` parameters should be modified
- Set the ``SRx_`` parameter to the rate (in Hz) you wish the group of messages to be sent.  The exact messages contained in each group can be determined by inspecting the STREAM_xxx arrays in each vehicle's GCS_Mavlink.cpp file (see here for `Copter <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/GCS_Mavlink.cpp#L393>`__, `Plane <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/GCS_Mavlink.cpp#L547>`__, `Rover <https://github.com/ArduPilot/ardupilot/blob/master/Rover/GCS_Mavlink.cpp#L457>`__, `Sub <https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/GCS_Mavlink.cpp#L318>`__ and `AntennaTracker <https://github.com/ArduPilot/ardupilot/blob/master/AntennaTracker/GCS_Mavlink.cpp#L232>`__).  Below is the list of messages by group for Copter-4.0:

    - SRx_ADSB

      - `ADSB_VEHICLE <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`__

    - SRx_EXT_STAT

      - `SYS_STATUS <https://mavlink.io/en/messages/common.html#SYS_STATUS>`__
      - `POWER_STATUS <https://mavlink.io/en/messages/common.html#POWER_STATUS>`__
      - `MEMINFO <https://mavlink.io/en/messages/ardupilotmega.html#MEMINFO>`__
      - `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`__
      - `GPS_RAW_INT <https://mavlink.io/en/messages/common.html#GPS_RAW_INT>`__
      - `GPS_RTK <https://mavlink.io/en/messages/common.html#GPS_RTK>`__
      - `GPS2_RAW <https://mavlink.io/en/messages/common.html#GPS2_RAW>`__
      - `GPS2_RTK <https://mavlink.io/en/messages/common.html#GPS2_RTK>`__
      - `NAV_CONTROLLER_OUTPUT <https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT>`__
      - `FENCE_STATUS <https://mavlink.io/en/messages/common.html#FENCE_STATUS>`__
      - `POSITION_TARGET_GLOBAL_INT <https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT>`__

    - SRx_EXTRA1

      - `ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`__
      - `SIMSTATE <https://mavlink.io/en/messages/common.html#SIMSTATE>`__
      - `AHRS2 <https://mavlink.io/en/messages/common.html#AHRS2>`__
      - `AHRS3 <https://mavlink.io/en/messages/common.html#AHRS3>`__
      - `PID_TUNING <https://mavlink.io/en/messages/common.html#PID_TUNING>`__

    - SRx_EXTRA2

      - `VFR_HUD <https://mavlink.io/en/messages/common.html#VFR_HUD>`__

    - SRx_EXTRA3

      - `AHRS <https://mavlink.io/en/messages/common.html#AHRS>`__
      - `HWSTATUS <https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS>`__
      - `SYSTEM_TIME <https://mavlink.io/en/messages/common.html#SYSTEM_TIME>`__
      - `RANGEFINDER <https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER>`__
      - `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`__
      - `TERRAIN_REQUEST <https://mavlink.io/en/messages/common.html#TERRAIN>`__
      - `BATTERY2 <https://mavlink.io/en/messages/ardupilotmega.html#BATTERY2>`__ (deprecated, use BATTERY_STATUS)
      - `BATTERY_STATUS <https://mavlink.io/en/messages/common.html#BATTERY_STATUS>`__
      - `MOUNT_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_STATUS>`__
      - `OPTICAL_FLOW <https://mavlink.io/en/messages/common.html#OPTICAL_FLOW>`__
      - `GIMBAL_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#GIMBAL_REPORT>`__
      - `MAG_CAL_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_REPORT>`__
      - `MAG_CAL_PROGRESS <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS>`__
      - `EKF_STATUS_REPORT <https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT>`__
      - `VIBRATION <https://mavlink.io/en/messages/ardupilotmega.html#VIBRATION>`__
      - `RPM <https://mavlink.io/en/messages/ardupilotmega.html#RPM>`__
      - `ESC_TELEMETRY_1_TO_4 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4>`__
      - `ESC_TELEMETRY_5_TO_8 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_5_TO_8>`__
      - `ESC_TELEMETRY_9_TO_12 <https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_9_TO_12>`__

    - SRx_PARAMS - should not be changed

    - SRx_POSITION

      - `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`__
      - `LOCAL_POSITION_NED <https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED>`__

    - SRx_RAW_CTRL (Not Used)

    - SRx_RAW_SENS

      - `RAW_IMU <https://mavlink.io/en/messages/common.html#RAW_IMU>`__
      - `SCALED_IMU2 <https://mavlink.io/en/messages/common.html#SCALED_IMU2>`__
      - `SCALED_IMU3 <https://mavlink.io/en/messages/common.html#SCALED_IMU3>`__
      - `SCALED_PRESSURE <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE>`__
      - `SCALED_PRESSURE2 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE2>`__
      - `SCALED_PRESSURE3 <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE3>`__
      - `SENSOR_OFFSETS <https://mavlink.io/en/messages/ardupilotmega.html#SENSOR_OFFSETS>`__

    - SRx_RC_CHAN

      - `SERVO_OUTPUT_RAW <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>`__
      - `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`__
      - `RC_CHANNELS_RAW  <https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW>`__ (only sent on mavlink1 links)

Using REQUEST_DATA_STREAM
-------------------------

Most ground stations including the Mission Planner use this method.  See :ref:`Setting the Datarate <planner:mission-planner-telemetry-logs-setting-the-datarate>` in the Mission Planner wiki.

Send a `REQUEST_DATA_STREAM <https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM>`__ message with the following fields

- target_system : the MAVLink system id of the vehicle (normally "1")
- target_components : normally "0"
- req_stream_id : 0 to 12 corresponding to the group of messages (see `MAV_DATA_STREAM <https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM>`__).  See the "Using SRx Parameters" section above to determine exactly which messages are in each group
- req_message_rate : the rate (in hz) of the message
- start_stop : "1" to start sending, "0" to stop

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- module load message

+-------------------------------------------+------------------------------------------------------+
| Example MAVProxy/SITL Command             | Description                                          |
+===========================================+======================================================+
| ``message REQUEST_DATA_STREAM 1 0 1 5 1`` | Request sysid:1, compid:0 send sensor data at 5hz    |
+-------------------------------------------+------------------------------------------------------+
| ``message REQUEST_DATA_STREAM 1 0 1 0 0`` | Request sysid:1, compid:0 stop sending sensor data   |
+-------------------------------------------+------------------------------------------------------+
| ``message REQUEST_DATA_STREAM 1 0 6 5 1`` | Request sysid:1, compid:0 send position data at 5hz  |
+-------------------------------------------+------------------------------------------------------+
| ``message REQUEST_DATA_STREAM 1 0 6 0 0`` | Request sysid:1, compid:0 stop sending position data |
+-------------------------------------------+------------------------------------------------------+

Using SET_MESSAGE_INTERVAL
--------------------------

This method provides the most precise control and reduces bandwidth requirements (because unnecessary messages are not sent) but requires knowing exactly which messages you require

Send a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the following fields

- target_system : the MAVLink system id of the vehicle (normally "1")
- target_components : normally "0"
- command: 511 (for `MAV_CMD_SET_MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL>`__)
- confirmation: 0
- param1: desired MAVLink message's id (i.e. 33 for `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`__)
- param2: time interval between messages in microseconds (i.e. 100000 for 10hz, 1000000 for 1hz)
- param3 to param7: 0 (not used)

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- module load message

+---------------------------------------------------------+---------------------------------------------+
| Example MAVProxy/SITL Command                           | Description                                 |
+=========================================================+=============================================+
| ``message COMMAND_LONG 1 0 511 0 33 100000 0 0 0 0 0``  | Request GLOBAL_POSITION_INT at 10hz         |
+---------------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 1 0 511 0 33 1000000 0 0 0 0 0`` | Request GLOBAL_POSITION_INT at 1hz          |
+---------------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 1 0 511 0 33 0 0 0 0 0 0``       | Request GLOBAL_POSITION_INT at default rate |
+---------------------------------------------------------+---------------------------------------------+
| ``message COMMAND_LONG 1 0 511 0 33 -1 0 0 0 0 0``      | Request GLOBAL_POSITION_INT not be sent     |
+---------------------------------------------------------+---------------------------------------------+

.. warning::

   If the telemetry link is shared (i.e. multiple GCSs or a GCS and a companion computer) there can be conflicting requests.  The most common example is the Mission Planner using the REQUEST_DATA_STREAM method while a companion computer uses SET_MESSAGE_INTERVAL method.  Mission Planner at least allows turning off the REQUEST_DATA_STREAM requests by setting the rates to "-1" (see :ref:`Setting the Datarate <planner:mission-planner-telemetry-logs-setting-the-datarate>`).  MAVProxy users can ``set streamrate -1``.

Using REQUEST_MESSAGE
---------------------

A GCS can poll for a single instance of a message from the autopilot.

Send a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the following fields

- target_system : the MAVLink system id of the vehicle (normally "1")
- target_components : normally "0"
- command: 512 (for `MAV_CMD_REQUEST_MESSAGE <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE>`__)
- confirmation: 0
- param1: desired MAVLink message's id (i.e. 33 for `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`__)
- param2: depends on message requested; see that message's definition for details.
- param3 to param7: 0 (not used)

**Examples**

Here are some example commands that can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter the following

- module load message

+---------------------------------------------------+------------------------------------------+
| Example MAVProxy/SITL Command                     | Description                              |
+===================================================+==========================================+
| ``message COMMAND_LONG 1 0 512 0 33 0 0 0 0 0 0`` | Request GLOBAL_POSITION_INT be sent once |
+---------------------------------------------------+------------------------------------------+

Specifying Message Rates in a File
----------------------------------

At boot ArduPilot will populate the initial message intervals from files found in either ROMFS or in the filesystem.

On ChibiOS-based boards (with more than 1MB of flash) the SD card will be searched for specially-named files in the root directory.

Each mavlink channel is configured in a separate configuration file.  The first serial port configured as mavlink is channel 0, the second serial port channel 1 etc.

An example filename is ``message-intervals-chan0.txt``

The format is simple but strict.  There are two columns, separated by a single space and both containing numbers.  The first number is a mavlink message ID.  The second is the message interval, in milliseconds.  Each line must be terminated by either carriage-return *or* a line-feed.

::

   30 50
   28 100
   29 200

This sample file content will stream `ATTITUDE <https://mavlink.io/en/messages/common.html#ATTITUDE>`__ (ID=30) at 20Hz and `SCALED_PRESSURE <https://mavlink.io/en/messages/common.html#SCALED_PRESSURE>`__ (ID=29) at 5Hz.  Message ID 28 is RAW_PRESSURE which ArduPilot does not send - this line will be ignored.

Configuration files can be included in ROMFS (i.e. compiled into the image) by specifying their path in the relevant board's hwdef file:

::

   ROMFS message-intervals-chan0.txt libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/message-intervals-chan0.txt

The second parameter is a path relative to the ArduPilot checkout's root directory.

Example use cases of this include locking telemetry rates on boards that can't run scripting, or before scripting can be run.

Using a Lua Script
------------------

- Download the `message_interval.lua <https://raw.githubusercontent.com/ArduPilot/ardupilot/refs/heads/master/libraries/AP_Scripting/examples/message_interval.lua>`__ Lua script
- Open the script in an editor (like Notepad)
- Ensure each mavlink message you wish to enable or disable is listed in the "mavlink message ids" (around line 13) and "intervals" array (around line 36)
- Upload the script to the autopilot by following :ref:`these instructions <copter:common-lua-scripts>`
- Restart the autopilot and confirm no errors appear on the ground stations's Messages tab
- Follow the instructions below to ensure the messages are appearing at the desired rate.  If the rates are variable, this may be because of a conflict between the Lua script and a GCS, please see the "Rates Reverting to some Fixed value" section below

Checking The Message Rates
--------------------------

Some ground stations including Mission Planner and QGC include a "MAVLink Inspector" which is useful when checking the update rate of specific messages.

If using Mission Planner:

- Press Ctrl-F
- Push the "MAVLink Inspector" button
- Expand the vehicle and component IDs to see individual messages and their update rate

.. image:: ../images/mavlink-mp-mavlink-inspector.png
    :target: ../_images/mavlink-mp-mavlink-inspector.png
    :width: 450px

If using MAVProxy:

 - module load messagerate
 - messagerate status

Rates Reverting to some Fixed value
-----------------------------------

If you find the message rates are reverting to some fixed rate (often 4hz) after you set them, it is probably a Ground Control station adjusting the rates.

Most Ground Control Stations have a control to disable this behaviour.  For example:

- If using MAVProxy type ``set streamrate -1`` into the console
- If using Misison Planner, open the Config, Planner screen and set the "Telemetry Rates" dropdowns to -1
- If using QGroundControl, open Application Settings, Telemetry and move the "Controlled By vehicle" slider to the right

If adjusting the GCS behaviour is impossible, set the appropriate ``SERIALn_OPTIONS`` parameter's bit 12 ("Ignore Streamrate") to ignore attempts by the GCS to set message rates via streamrate commands.  For example if Serial1/Telem1's rates are being inconveniently adjusted by the GCS, set the :ref:`SERIAL1_OPTIONS <SERIAL1_OPTIONS>` parameter value
