.. _hwdef-parameter-defaults:

========================================
hwdef Defines that Change Param Defaults
========================================

Many parameters have their default value set by a ``define`` in the board's ``hwdef.dat`` file. This lets a board ship with the correct battery monitor pins, serial port protocols, heater target, etc. already configured, without requiring a separate ``defaults.parm`` file entry.

The tables below list the hwdef defines that change a parameter's default, the parameter they affect, and the "Normal Default" value the parameter takes if the board's hwdef does not set the define. Where the normal default depends on the vehicle or board features, the conditions are noted.

.. note:: Serial port baud rate defines (``DEFAULT_SERIALn_BAUD``) use the same units as the parameter (e.g. 57 for 57600 baud) except ``DEFAULT_SERIAL0_BAUD``, which is given in bits per second (e.g. 115200).

.. note:: Defines that only enable or disable a feature (``*_ENABLED``) are not listed here. They determine whether a parameter exists at all, rather than its default value.

For more information on creating hwdef files, see :ref:`porting`.

Serial Ports
============

.. list-table::
   :header-rows: 1
   :widths: 30 25 20 25

   * - Define
     - Parameter
     - Normal Default
     - Notes
   * - ``DEFAULT_SERIAL1_PROTOCOL``
     - :ref:`SERIAL1_PROTOCOL<copter:SERIAL1_PROTOCOL>`
     - 2 (MAVLink2)
     -
   * - ``DEFAULT_SERIAL2_PROTOCOL``
     - :ref:`SERIAL2_PROTOCOL<copter:SERIAL2_PROTOCOL>`
     - 2 (MAVLink2)
     -
   * - ``DEFAULT_SERIAL3_PROTOCOL``
     - :ref:`SERIAL3_PROTOCOL<copter:SERIAL3_PROTOCOL>`
     - 5 (GPS)
     -
   * - ``DEFAULT_SERIAL4_PROTOCOL``
     - :ref:`SERIAL4_PROTOCOL<copter:SERIAL4_PROTOCOL>`
     - 5 (GPS)
     -
   * - ``DEFAULT_SERIAL5_PROTOCOL`` to ``DEFAULT_SERIAL9_PROTOCOL``
     - :ref:`SERIAL5_PROTOCOL<copter:SERIAL5_PROTOCOL>` to :ref:`SERIAL9_PROTOCOL<copter:SERIAL9_PROTOCOL>`
     - -1 (None)
     -
   * - ``DEFAULT_SERIAL0_BAUD``
     - :ref:`SERIAL0_BAUD<copter:SERIAL0_BAUD>`
     - 115 (115200)
     - define is in bits per second
   * - ``DEFAULT_SERIAL1_BAUD``, ``DEFAULT_SERIAL2_BAUD``
     - :ref:`SERIAL1_BAUD<copter:SERIAL1_BAUD>`, :ref:`SERIAL2_BAUD<copter:SERIAL2_BAUD>`
     - 57 (57600)
     -
   * - ``DEFAULT_SERIAL3_BAUD``, ``DEFAULT_SERIAL4_BAUD``
     - :ref:`SERIAL3_BAUD<copter:SERIAL3_BAUD>`, :ref:`SERIAL4_BAUD<copter:SERIAL4_BAUD>`
     - 230 (230400)
     -
   * - ``DEFAULT_SERIAL5_BAUD`` to ``DEFAULT_SERIAL9_BAUD``
     - :ref:`SERIAL5_BAUD<copter:SERIAL5_BAUD>` to :ref:`SERIAL9_BAUD<copter:SERIAL9_BAUD>`
     - 57 (57600)
     -
   * - ``DEFAULT_SERIAL1_OPTIONS`` to ``DEFAULT_SERIAL6_OPTIONS``, ``DEFAULT_SERIAL9_OPTIONS``
     - :ref:`SERIAL1_OPTIONS<copter:SERIAL1_OPTIONS>` to :ref:`SERIAL6_OPTIONS<copter:SERIAL6_OPTIONS>`, :ref:`SERIAL9_OPTIONS<copter:SERIAL9_OPTIONS>`
     - 0
     - e.g. to set inversion or RX/TX swap on a port; SERIAL7_OPTIONS and SERIAL8_OPTIONS have hard-coded defaults

Battery Monitor
===============

.. list-table::
   :header-rows: 1
   :widths: 30 25 20 25

   * - Define
     - Parameter
     - Normal Default
     - Notes
   * - ``HAL_BATT_MONITOR_DEFAULT``
     - :ref:`BATT_MONITOR<copter:BATT_MONITOR>`
     - 0 (Disabled)
     -
   * - ``HAL_BATT_VOLT_PIN``
     - :ref:`BATT_VOLT_PIN<copter:BATT_VOLT_PIN__AP_BattMonitor_Analog>`
     - 4 (ChibiOS), -1 (others)
     - requires ``HAL_BATT_VOLT_SCALE`` to also be defined
   * - ``HAL_BATT_CURR_PIN``
     - :ref:`BATT_CURR_PIN<copter:BATT_CURR_PIN__AP_BattMonitor_Analog>`
     - 3 (ChibiOS), -1 (others)
     - requires ``HAL_BATT_CURR_SCALE`` to also be defined
   * - ``HAL_BATT_VOLT_SCALE``
     - :ref:`BATT_VOLT_MULT<copter:BATT_VOLT_MULT__AP_BattMonitor_Analog>`
     - 10.1
     -
   * - ``HAL_BATT_CURR_SCALE``
     - :ref:`BATT_AMP_PERVLT<copter:BATT_AMP_PERVLT__AP_BattMonitor_Analog>`
     - 17.0
     -
   * - ``AP_BATT_CURR_AMP_OFFSET_DEFAULT``
     - :ref:`BATT_AMP_OFFSET<copter:BATT_AMP_OFFSET__AP_BattMonitor_Analog>`
     - 0
     -
   * - ``HAL_BATT2_MONITOR_DEFAULT``
     - :ref:`BATT2_MONITOR<copter:BATT2_MONITOR>`
     - 0 (Disabled)
     -
   * - ``HAL_BATT2_VOLT_PIN``
     - :ref:`BATT2_VOLT_PIN<copter:BATT2_VOLT_PIN__AP_BattMonitor_Analog>`
     - same as BATT_VOLT_PIN
     - only applies if the second monitor is analog
   * - ``HAL_BATT2_CURR_PIN``
     - :ref:`BATT2_CURR_PIN<copter:BATT2_CURR_PIN__AP_BattMonitor_Analog>`
     - same as BATT_CURR_PIN
     - only applies if the second monitor is analog
   * - ``HAL_BATT2_VOLT_SCALE``
     - :ref:`BATT2_VOLT_MULT<copter:BATT2_VOLT_MULT__AP_BattMonitor_Analog>`
     - 10.1
     - only applies if the second monitor is analog
   * - ``HAL_BATT2_CURR_SCALE``
     - :ref:`BATT2_AMP_PERVLT<copter:BATT2_AMP_PERVLT__AP_BattMonitor_Analog>`
     - 17.0
     - only applies if the second monitor is analog
   * - ``HAL_BATTMON_INA2XX_BUS``
     - :ref:`BATT_I2C_BUS<copter:BATT_I2C_BUS__AP_BattMonitor_INA2xx>`
     - 0
     -
   * - ``HAL_BATTMON_INA2XX_ADDR``
     - :ref:`BATT_I2C_ADDR<copter:BATT_I2C_ADDR__AP_BattMonitor_INA2xx>`
     - 0 (probe)
     -
   * - ``DEFAULT_BATTMON_INA2XX_SHUNT``
     - :ref:`BATT_SHUNT<copter:BATT_SHUNT__AP_BattMonitor_INA2xx>`
     - 0.0005
     -

Board, IMU and Heater
=====================

.. list-table::
   :header-rows: 1
   :widths: 30 25 20 25

   * - Define
     - Parameter
     - Normal Default
     - Notes
   * - ``HAL_DEFAULT_INS_FAST_SAMPLE``
     - :ref:`INS_FAST_SAMPLE<copter:INS_FAST_SAMPLE>`
     - 1 (first IMU)
     -
   * - ``HAL_EKF_IMU_MASK_DEFAULT``
     - :ref:`EK3_IMU_MASK<copter:EK3_IMU_MASK>`, :ref:`EK2_IMU_MASK<copter:EK2_IMU_MASK>`
     - 3 (first two IMUs)
     -
   * - ``HAL_IMU_TEMP_DEFAULT``
     - :ref:`BRD_HEAT_TARG<copter:BRD_HEAT_TARG>`
     - -1 (heater disabled)
     -
   * - ``HAL_IMU_TEMP_MARGIN_LOW_DEFAULT``
     - :ref:`BRD_HEAT_LOWMGN<copter:BRD_HEAT_LOWMGN>`
     - 0 (disabled)
     -
   * - ``HAL_IMUHEAT_P_DEFAULT``
     - :ref:`BRD_HEAT_P<copter:BRD_HEAT_P>`
     - 200
     -
   * - ``HAL_IMUHEAT_I_DEFAULT``
     - :ref:`BRD_HEAT_I<copter:BRD_HEAT_I>`
     - 0.3
     -
   * - ``BOARD_TYPE_DEFAULT``
     - :ref:`BRD_TYPE<copter:BRD_TYPE>`
     - 0 (Auto)
     -
   * - ``BOARD_SAFETY_ENABLE_DEFAULT``
     - :ref:`BRD_SAFETY_DEFLT<copter:BRD_SAFETY_DEFLT>`
     - 1 if the board has a safety switch input or an IOMCU, otherwise 0
     -
   * - ``HAL_HAVE_SAFETY_SWITCH``
     - :ref:`BRD_SAFETYOPTION<rover:BRD_SAFETYOPTION>`
     - 3
     - Rover and AntennaTracker only: if set, changes the default to 7 so the safety button is also active while armed
   * - ``BOARD_CONFIG_BOARD_VOLTAGE_MIN``
     - :ref:`BRD_VBUS_MIN<copter:BRD_VBUS_MIN>`
     - 4.3
     -
   * - ``HAL_TCAL_BARO_EXP_DEFAULT``
     - :ref:`TCAL_BARO_EXP<copter:TCAL_BARO_EXP>`
     - 0
     -

Sensors
=======

.. list-table::
   :header-rows: 1
   :widths: 30 25 20 25

   * - Define
     - Parameter
     - Normal Default
     - Notes
   * - ``HAL_COMPASS_AUTO_ROT_DEFAULT``
     - :ref:`COMPASS_AUTO_ROT<copter:COMPASS_AUTO_ROT>`
     - 2 (CheckAndFix)
     -
   * - ``AP_COMPASS_ENABLE_DEFAULT``
     - :ref:`COMPASS_ENABLE<copter:COMPASS_ENABLE>`
     - 1
     -
   * - ``HAL_AIRSPEED_TYPE_DEFAULT``
     - :ref:`ARSPD_TYPE<plane:ARSPD_TYPE>`
     - 1 (MS4525) on Plane if that backend is enabled, otherwise 0 (None)
     - also changes the ARSPD_PIN default to 1, except on AP_Periph where that parameter is not present
   * - ``HAL_AIRSPEED_BUS_DEFAULT``
     - :ref:`ARSPD_BUS<plane:ARSPD_BUS>`
     - 1
     -
   * - ``HAL_DEFAULT_AIRSPEED_PIN``
     - :ref:`ARSPD_PIN<plane:ARSPD_PIN>`
     - 15
     - only used on Plane when HAL_AIRSPEED_TYPE_DEFAULT is not defined
   * - ``HAL_GPS1_TYPE_DEFAULT``
     - :ref:`GPS1_TYPE<copter:GPS1_TYPE>`
     - 1 (Auto)
     - also used by AP_Periph
   * - ``HAL_GPS_COM_PORT_DEFAULT``
     - :ref:`GPS1_COM_PORT<copter:GPS1_COM_PORT>`
     - 1
     -
   * - ``OPTICAL_FLOW_TYPE_DEFAULT``
     - :ref:`FLOW_TYPE<copter:FLOW_TYPE>`
     - 0 (None)
     -
   * - ``BOARD_RSSI_ANA_PIN``
     - :ref:`RSSI_ANA_PIN<copter:RSSI_ANA_PIN>`
     - -1
     -
   * - ``AP_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT``
     - :ref:`TEMP1_BUS<copter:TEMP1_BUS>`
     - 0
     -
   * - ``AP_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT``
     - :ref:`TEMP1_ADDR<copter:TEMP1_ADDR>`
     - 0
     -

Notify, OSD and Relays
======================

.. list-table::
   :header-rows: 1
   :widths: 30 25 20 25

   * - Define
     - Parameter
     - Normal Default
     - Notes
   * - ``HAL_BUZZER_PIN``
     - :ref:`NTF_BUZZ_PIN<copter:NTF_BUZZ_PIN>`
     - -1 (none)
     -
   * - ``DEFAULT_BUZZ_ON_LVL``
     - :ref:`NTF_BUZZ_ON_LVL<copter:NTF_BUZZ_ON_LVL>`
     - 1 (high)
     -
   * - ``DEFAULT_NTF_LED_TYPES``
     - :ref:`NTF_LED_TYPES<copter:NTF_LED_TYPES>`
     - Board LEDs and I2C LEDs (plus ProfiLED_IOMCU on boards with an IOMCU)
     - If defining a serial led add its type and include this define
   * - ``HAL_OSD_TYPE_DEFAULT``
     - :ref:`OSD_TYPE<copter:OSD_TYPE>`
     - 0 (None)
     -
   * - ``RELAY1_PIN_DEFAULT`` to ``RELAY6_PIN_DEFAULT``
     - :ref:`RELAY1_PIN<copter:RELAY1_PIN>` etc.
     - -1 (none)
     - commonly included when VTX power switch or dual Camera inputs are defined
   * - ``RELAY1_DEFAULT`` to ``RELAY6_DEFAULT``
     - :ref:`RELAY1_DEFAULT<copter:RELAY1_DEFAULT>` etc.
     - 0 (Off)
     - define of -1 leaves the parameter's normal default

Vehicle, CAN, Logging and Networking
====================================

.. list-table::
   :header-rows: 1
   :widths: 30 25 20 25

   * - Define
     - Parameter
     - Normal Default
     - Notes
   * - ``HAL_FRAME_TYPE_DEFAULT``
     - :ref:`FRAME_TYPE<copter:FRAME_TYPE>`
     - 1 (X)
     - Copter only
   * - ``MAV_SYSID_DEFAULT``
     - :ref:`MAV_SYSID<copter:MAV_SYSID>`
     - 1 (2 on AntennaTracker)
     -
   * - ``HAL_CAN_DRIVER_DEFAULT``
     - :ref:`CAN_P1_DRIVER<copter:CAN_P1_DRIVER>` etc.
     - 0 (Disabled)
     - commonly used to enable the CAN ports, especially if CAN power monitors or GPSes are included.
   * - ``DRONECAN_NODE_POOL_SIZE``
     - :ref:`CAN_D1_UC_POOL<copter:CAN_D1_UC_POOL>`
     - 8192 (16384 on CAN FD capable boards)
     -
   * - ``HAL_LOGGER_MIN_MB_FREE``
     - :ref:`LOG_FILE_MB_FREE<copter:LOG_FILE_MB_FREE>`
     - 500 (2 or 10 on boards logging to flash via LittleFS)
     -
   * - ``HAL_LOGGING_BACKENDS_DEFAULT``
     - :ref:`LOG_BACKEND_TYPE<copter:LOG_BACKEND_TYPE>`
     - 1 (File), 4 (Block), 2 (MAVLink), or 0 (None), depending on the compiled backends
     - File is preferred on SITL; otherwise Block, File, and MAVLink are selected in that order
   * - ``AP_NETWORKING_DEFAULT_STATIC_IP_ADDR``
     - :ref:`NET_IPADDR0<copter:NET_IPADDR0>` to NET_IPADDR3
     - 192.168.144.14
     - define is a string, e.g. "192.168.144.14"
   * - ``AP_NETWORKING_DEFAULT_STATIC_GW_ADDR``
     - :ref:`NET_GWADDR0<copter:NET_GWADDR0>` to NET_GWADDR3
     - 192.168.144.1
     - define is a string
   * - ``AP_NETWORKING_DEFAULT_NETMASK``
     - :ref:`NET_NETMASK<copter:NET_NETMASK>`
     - 24
     -
   * - ``AP_NETWORKING_DEFAULT_MAC_ADDR``
     - :ref:`NET_MACADDR0<copter:NET_MACADDR0>` to NET_MACADDR5
     - C2:AF:51:03:CF:46
     - define is a string

AP_Periph
=========

These defines are only used by :ref:`AP_Periph <ap-peripheral-landing-page>` firmware.

.. list-table::
   :header-rows: 1
   :widths: 30 25 20 25

   * - Define
     - Parameter
     - Normal Default
     - Notes
   * - ``HAL_CAN_DEFAULT_NODE_ID``
     - :ref:`CAN_NODE<CAN_NODE>`
     - 0 (dynamic node allocation)
     -
   * - ``HAL_PERIPH_GPS_PORT_DEFAULT``
     - :ref:`GPS_PORT<GPS_PORT>`
     - 3
     -
   * - ``AP_PERIPH_MSP_PORT_DEFAULT``
     - :ref:`MSP_PORT<MSP_PORT>`
     - 1
     -
   * - ``AP_PERIPH_RANGEFINDER_PORT_DEFAULT``
     - :ref:`RNGFND_PORT<RNGFND_PORT>` and :ref:`PRX_PORT<PRX_PORT>`
     - 3
     - sets the first rangefinder and proximity sensor serial ports when those features are enabled
   * - ``HAL_PERIPH_ADSB_PORT_DEFAULT``
     - :ref:`ADSB_PORT<ADSB_PORT>`
     - 1
     -
   * - ``HAL_PERIPH_ADSB_BAUD_DEFAULT``
     - :ref:`ADSB_BAUDRATE<ADSB_BAUDRATE>`
     - 57600
     -
   * - ``AP_PERIPH_EFI_PORT_DEFAULT``
     - :ref:`EFI_PORT<EFI_PORT>`
     - 3
     -
   * - ``HAL_PERIPH_EFI_BAUDRATE_DEFAULT``
     - :ref:`EFI_BAUDRATE<EFI_BAUDRATE>`
     - 115200
     -
   * - ``AP_PERIPH_RC1_PORT_DEFAULT``
     - :ref:`RC1_PORT<RC1_PORT>`
     - -1 (disabled)
     -
   * - ``HAL_PWM_HARDPOINT_ID_DEFAULT``
     - :ref:`HARDPOINT_ID<HARDPOINT_ID>`
     - none
     - must be defined if ``AP_PERIPH_PWM_HARDPOINT_ENABLED`` is set
   * - ``AP_PERIPH_BARO_ENABLE_DEFAULT``
     - :ref:`BARO_ENABLE<BARO_ENABLE>`
     - 1
     -
   * - ``AP_PERIPH_PROBE_CONTINUOUS``
     - :ref:`OPTIONS<OPTIONS>`
     - 0
     - value of 1 sets bit 0, "Enable continuous sensor probe"
   * - ``AP_DAC_DEFAULT_TYPE``, ``AP_DAC_DEFAULT_BUS``, ``AP_DAC_DEFAULT_ADDR``
     - :ref:`DAC1_TYPE<DAC1_TYPE>`, :ref:`DAC1_BUS<DAC1_BUS>`, :ref:`DAC1_ADDR<DAC1_ADDR>`
     - 0, 0, 0
     -
   * - ``AP_PERIPH_BATTERY_BALANCE_NUMCELLS_DEFAULT``
     - :ref:`BAL_NUM_CELLS<BAL_NUM_CELLS>`
     - 0
     -
   * - ``AP_PERIPH_BATTERY_BALANCE_ID_DEFAULT``
     - :ref:`BAL_ID<BAL_ID>`
     - 0
     -
   * - ``AP_PERIPH_BATTERY_BALANCE_RATE_DEFAULT``
     - :ref:`BAL_RATE<BAL_RATE>`
     - 1
     -
   * - ``AP_PERIPH_BATTERY_BALANCE_CELL1_PIN_DEFAULT``
     - :ref:`BAL_CELL1_PIN<BAL_CELL1_PIN>`
     - 1
     -
   * - ``AP_PERIPH_NET_PPP_PORT_DEFAULT``
     - :ref:`NET_PPP_PORT<NET_PPP_PORT>`
     - -1 (disabled)
     -
   * - ``AP_PERIPH_NET_PPP_BAUD_DEFAULT``
     - :ref:`NET_PPP_BAUD<NET_PPP_BAUD>`
     - 12500000
     -
