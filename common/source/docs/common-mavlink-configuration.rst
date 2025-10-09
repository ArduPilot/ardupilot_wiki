.. _mavlink_configuration:

==============================
MAVLink Advanced Configuration
==============================


.. note :: Most users never have to change these parameters from their default values.

If a serial port protocol is set to MAVLink (``SERIALx_PROTOCOL`` == 1 or 2), then advance configuration options can be set. The first serial port to have the protocol uses the ``MAV1_`` parameters (often the USB port on ``SERIAL0``), the second port will be ``MAV2_``. etc.

STREAM RATES
============
The following stream rate parameters can be set for each instance of a MAVLink using serial port to determine how fast data is sent over the port(SERIAL1 will be used in the following as an example)


* :ref:`MAV1_ADSB<MAV1_ADSB>`: ADSB stream rate
* :ref:`MAV1_EXT_STAT<MAV1_EXT_STAT>`: MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT
* :ref:`MAV1_EXTRA1<MAV1_EXTRA1>`: MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING
* :ref:`MAV1_EXTRA2<MAV1_EXTRA2>`: MAVLink Stream rate of VFR_HUD
* :ref:`MAV1_EXTRA3<MAV1_EXTRA3>`: MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, BATTERY2, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS
* :ref:`MAV1_PARAMS<MAV1_PARAMS>`: MAVLink Stream rate of PARAM_VALUE
* :ref:`MAV1_POSITION<MAV1_POSITION>`: MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED
* :ref:`MAV1_RAW_CTRL<MAV1_RAW_CTRL>`: MAVLink Raw Control stream rate of SERVO_OUT
* :ref:`MAV1_RAW_SENS<MAV1_RAW_SENS>`: MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED
* :ref:`MAV1_RC_CHAN<MAV1_RC_CHAN>`: MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS

OPTIONS
=======
Each established MAVLink channel has several behaviors set by its options parameter:

================ =====================
MAVx_OPTIONS     Function
================ =====================
bit 1 set        Do not forward MAVLink messages to or from this channel to other channels
bit 2 set        Ignore stream rate change requests from other MAVLink stations (GCS, etc.)
================ =====================

In addition, there are several global MAVLink related parameters

* :ref:`MAV_GCS_SYSID<MAV_GCS_SYSID>`: This controls whether packets from other than the expected GCS system ID will be accepted
* :ref:`MAV_OPTIONS<MAV_OPTIONS>`: Alters various behaviour of the MAVLink interface. See table below. If bit 0 is set, the the channel will accetp only MAVLink messages for the SYSID_GCS.
* :ref:`MAV_SYSID<MAV_SYSID>`: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
* :ref:`MAV_TELEM_DELAY<MAV_TELEM_DELAY>`: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up

