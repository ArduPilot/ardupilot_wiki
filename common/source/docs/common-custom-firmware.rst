.. _common-custom-firmware:

=================================
ArduPilot Custom Firmware Builder
=================================

As the ArduPilot software evolves through community contributions and development efforts, it’s expanding rapidly with many new features being added. However, not all users need every feature. Depending on their specific requirements and applications, users might prefer to exclude certain features from the software.

To address this need, we have developed the **Custom Firmware Builder**, a web application hosted at `custom.ardupilot.org <https://custom.ardupilot.org/>`__. This process helps in reducing the firmware size, making it easier to fit into flight control units (FCUs) with limited flash memory. It also benefits our testers, allowing them to test specific features without needing to upgrade their flight controllers due to space constraints.

The source code of the custom firmware builder can be found `here. <https://github.com/ArduPilot/CustomBuild/>`__

Submitting a build request
==========================

To submit a build request, follow these steps:

#. Click the **'Add a build'** button located at the top right corner of the homepage.

    .. image:: ../../../images/custom-firmware-build-server-addbuild.png
        :target: ../_images/custom-firmware-build-server-addbuild.png
        :width: 450px

#. Choose the vehicle, version, and board,in that order, for which you want to build the firmware.

    .. image:: ../../../images/custom-firmware-build-server.png
        :target: ../_images//custom-firmware-build-server.png
        :width: 450px

#. Select the features you wish to include in the firmware

    - The default features for the board/vehicle will automatically be selected
    - Some features depend on other features so when a feature is selected, all its dependencies will be automatically selected but if the feature is later deselected, the dependencies are not automatically deselected
    - It is possible to select a set of features that will not fit on a board which will cause the build will fail.

#. Click the **'Generate'** button. You will be redirected to the homepage, and a build log for the request you just submitted will appear.
#. Once the build is complete the build artifact will be downloaded automatically, You can also download it manually by clicking the **'Download'** button next to your build request in the build list table. The button will be enabled only after a successful build.
#. Extract the downloaded build artifact. The extracted files will include the .apj firmware file along with other files
#. Use your GCS to install the .apj file onto your autopilot.  If using Mission Planner, open the Install Firmware and click then "Load custom firmware" link

    .. image:: ../../../images/mission-planner-load-custom-firmware.png
        :target: ../_images/mission-planner-load-custom-firmware.png
        :width: 450px

Versions
========

Latest
------
This is the current development branch of ArduPilot. It includes new features, enhancements, clean-ups, etc. but has not been through a release cycle of beta testing. It has been alpha tested and thoroughly tested in ArduPilot's Continuous Integration auto-test suite. Many experienced users select this to get the latest features but it has the highest risk, in terms of potential bugs.

Beta
----
This is a release of the development branch used for widespread testing and feedback. While the risk of major issues is low, it is expected that minor behavioral deviations may pop up. These are normally addressed in later beta releases of a major new release before declaring the code as a stable release.

Stable
------
These are fully released versions of the code. There are usually several minor releases for each major release number which correct any anomalies found during the beta testing phases. These releases, have the lowest risk. It is recommended that the highest numbered major/minor release be used in a given release sequence, unless an older version is needed.

Guide to Size Impacts
=====================

Often, the custom build server is used to create firmware with features that have been removed in the standard firmware to conserve flash space, but desired by the user. If a build fails to complete due to lack of flash, the user may remove features/sensors that are  not needed in order to make room for those additional items wanted. In order to facilitate those choices, the table below shows what impact a feature's removal has on flash size. Note that a removed feature in this list automatically removed all features dependent upon it.

.. note:: the exact numbers of bytes will change slightly over time due to code changes, but this table should be accurate enough to make decisions on removal candidates to gain flash space.

=============================================== ====== ====== ======
FEATURE                                         PLANE  COPTER ROVER
=============================================== ====== ====== ======
AP_SCRIPTING_ENABLED                            175800 182160 171688  
HAL_NAVEKF3_AVAILABLE                           171392 171400 170784  
HAL_QUADPLANE_ENABLED                           139776 na     na      
HAL_ENABLE_DRONECAN_DRIVERS                     97480  96120  95936   
HAL_LOGGING_ENABLED                             88792  95704  86400   
HAL_MOUNT_ENABLED                               67268  67300  67116   
AP_RANGEFINDER_ENABLED                          46568  48056  46272   
HAL_GYROFFT_ENABLED                             42396  42308  41772   
OSD_ENABLED                                     40992  40568  40472   
AP_RCPROTOCOL_ENABLED                           35888  35888  35904   
AP_FILESYSTEM_ROMFS_ENABLED                     32920  32912  32912   
HAL_MSP_ENABLED                                 31848  31808  31816   
AP_BOOTLOADER_FLASHING_ENABLED                  31648  31648  31640   
AP_OPTICALFLOW_ENABLED                          29232  33816  29216   
HAL_EXTERNAL_AHRS_ENABLED                       29120  29112  29008   
HAL_ADSB_ENABLED                                25760  25800  19112   
AP_AIRSPEED_ENABLED                             25440  22984  23468   
AP_CAMERA_ENABLED                               25192  25192  25192   
AP_FRSKY_TELEM_ENABLED                          24504  24288  24376   
AP_FRSKY_SPORT_TELEM_ENABLED                    22088  21952  22040   
AP_FRSKY_SPORT_PASSTHROUGH_ENABLED              18376  18240  18328   
HAL_WITH_MSP_DISPLAYPORT                        18328  18248  18240   
AP_FENCE_ENABLED                                17488  50360  44748   
HAL_SOLO_GIMBAL_ENABLED                         17432  17192  17424   
HAL_EFI_ENABLED                                 14724  14816  14784   
COMPASS_CAL_ENABLED                             14624  14592  14600   
EK3_FEATURE_OPTFLOW_FUSION                      12824  12816  12832   
AP_GPS_UBLOX_ENABLED                            12504  12368  12440   
AP_RCPROTOCOL_CRSF_ENABLED                      11648  11640  11648   
AP_BARO_PROBE_EXTERNAL_I2C_BUSES                11296  11296  11296   
OSD_PARAM_ENABLED                               11116  11596  8152    
HAL_VISUALODOM_ENABLED                          10616  10520  10648   
AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED           10328  10344  10224   
HAL_CRSF_TELEM_ENABLED                          9680   9680   9680    
AP_RPM_ENABLED                                  9616   5504   5648    
HAL_MOUNT_XACTI_ENABLED                         8984   8968   8976    
AP_VIDEOTX_ENABLED                              8956   9036   9060    
HAL_PICCOLO_CAN_ENABLE                          8920   8920   8920    
AP_BEACON_ENABLED                               8840   14424  14440   
AP_RCPROTOCOL_SRXL2_ENABLED                     8592   8584   8584    
HAL_GENERATOR_ENABLED                           8424   8544   8416    
HAL_ADSB_SAGETECH_MXS_ENABLED                   8376   8376   8592    
HAL_INS_TEMPERATURE_CAL_ENABLE                  7952   8040   8000    
AC_PRECLAND_ENABLED                             7832   10144  8968    
HAL_SOARING_ENABLED                             7656   na     na      
HAL_PROXIMITY_ENABLED                           7272   26824  25472   
HAL_MOUNT_TOPOTEK_ENABLED                       7092   7092   7092    
AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED         6368   9824   5128    
AP_GPS_NMEA_ENABLED                             6176   6176   6184    
HAL_MOUNT_SIYI_ENABLED                          6000   5992   5992    
HAL_LANDING_DEEPSTALL_ENABLED                   5712   120    120     
AP_CAMERA_RUNCAM_ENABLED                        5616   5600   5592    
AP_OPTICALFLOW_PIXART_ENABLED                   5536   5536   5536    
AP_SERVO_TELEM_ENABLED                          5520   4864   5344    
GPS_MOVING_BASELINE                             5096   5040   5088    
AP_NOTIFY_TONEALARM_ENABLED                     4992   4984   4984    
EK3_FEATURE_DRAG_FUSION                         4984   4984   4984    
AP_GPS_SBF_ENABLED                              4904   4904   4904    
AP_EXTERNAL_AHRS_VECTORNAV_ENABLED              4824   4816   4832    
HAL_MOUNT_VIEWPRO_ENABLED                       4812   4812   4804    
AP_GPS_SBP_ENABLED                              4792   4864   4880    
AP_MAVLINK_FTP_ENABLED                          4656   4648   4648    
AP_CRASHDUMP_ENABLED                            4652   4636   4644    
AP_ICENGINE_ENABLED                             4600   144    144     
HAL_SPEKTRUM_TELEM_ENABLED                      4576   4576   4576    
AP_RELAY_ENABLED                                4464   4376   4528    
AP_SERIALMANAGER_REGISTER_ENABLED               4376   4448   4456    
AP_PROXIMITY_DRONECAN_ENABLED                   4248   2024   2024    
HAL_DISPLAY_ENABLED                             4176   4168   4168    
AP_BATTERY_SMBUS_ENABLED                        4172   4172   4172    
AP_RANGEFINDER_VL53L1X_ENABLED                  4160   4152   4152    
AP_QUICKTUNE_ENABLED                            4136   na     na      
AP_TUNING_ENABLED                               4072   na     na      
AP_ADVANCEDFAILSAFE_ENABLED                     4032   472    472     
AP_CAN_SLCAN_ENABLED                            4016   4096   4032    
AP_RCPROTOCOL_GHST_ENABLED                      3680   3672   3672    
HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL              3632   3496   3592    
AP_CAMERA_INFO_FROM_SCRIPT_ENABLED              3480   3480   3480    
HAL_ADSB_UCP_ENABLED                            3424   3416   3416    
AP_FETTEC_ONEWIRE_ENABLED                       3312   3296   3248    
EK3_FEATURE_EXTERNAL_NAV                        3272   3272   3272    
AP_FILESYSTEM_PARAM_ENABLED                     3208   3208   3208    
AP_RANGEFINDER_VL53L0X_ENABLED                  3200   3200   3200    
AP_SMARTAUDIO_ENABLED                           3160   3144   3144    
AP_GPS_BLENDED_ENABLED                          3096   3096   3096    
HAL_MSP_OPTICALFLOW_ENABLED                     2944   2952   2952    
AP_COMPASS_DRONECAN_ENABLED                     2872   2872   2872    
HAL_MSP_RANGEFINDER_ENABLED                     2856   2864   2864    
AP_DRONECAN_SERIAL_ENABLED                      2800   2800   2800    
AP_DRONECAN_HOBBYWING_ESC_SUPPORT               2792   2792   2792    
HAL_RALLY_ENABLED                               2768   2848   3016    
AP_COMPASS_BMM350_ENABLED                       2656   2656   2656    
AP_FOLLOW_ENABLED                               2608   4448   3928    
AP_FILESYSTEM_SYS_ENABLED                       2576   2576   2568    
MODE_AUTOLAND_ENABLED                           2488   na     na      
HAL_ADSB_SAGETECH_ENABLED                       2472   2464   2464    
AP_AIRSPEED_DRONECAN_ENABLED                    2456   2456   2456    
AP_COMPASS_HMC5843_ENABLED                      2456   2448   2448    
AP_GENERATOR_IE_2400_ENABLED                    2392   2392   2392    
AP_DRONECAN_SEND_GPS                            2360   2352   2352    
AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED           2336   2336   2336    
AP_GPS_SBP2_ENABLED                             2320   2376   2440    
AP_RANGEFINDER_LWI2C_ENABLED                    2320   2320   2320    
AP_GHST_TELEM_ENABLED                           2304   2304   2304    
AP_BATTERY_INA2XX_ENABLED                       2288   2280   2280    
HAL_MSP_SENSORS_ENABLED                         2288   2288   2288    
AP_AIRSPEED_MSP_ENABLED                         2272   2272   2272    
AP_BARO_MSP_ENABLED                             2272   2280   2280    
AP_COMPASS_MSP_ENABLED                          2272   2280   2280    
HAL_MSP_GPS_ENABLED                             2272   2280   2280    
AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED           2264   na     na      
AP_AIRSPEED_SDP3X_ENABLED                       2256   2248   2256    
AP_FILESYSTEM_MISSION_ENABLED                   2200   2200   2200    
AP_GPS_GSOF_ENABLED                             2192   2184   2184    
AP_FILTER_ENABLED                               2168   2128   2136    
AP_GENERATOR_RICHENPOWER_ENABLED                2168   2248   2168    
HAL_HIGH_LATENCY2_ENABLED                       2168   2024   1920    
AP_COMPASS_LIS3MDL_ENABLED                      2128   2128   2128    
AP_COMPASS_MMC3416_ENABLED                      2120   2112   2112    
AP_BARO_DRONECAN_ENABLED                        2080   2080   2080    
AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED          2064   2072   2056    
AP_GPS_NMEA_UNICORE_ENABLED                     2008   2000   2000    
AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED        1976   1976   1976    
HAL_BUTTON_ENABLED                              1960   1984   2072    
AP_EFI_SERIAL_MS_ENABLED                        1944   1936   1936    
AP_ROBOTISSERVO_ENABLED                         1936   1872   1960    
HAL_PARACHUTE_ENABLED                           1912   1824   616     
HAL_MOUNT_SERVO_ENABLED                         1880   1688   1688    
AP_LANDINGGEAR_ENABLED                          1824   1360   na      
AP_EFI_DRONECAN_ENABLED                         1752   1752   1752    
AP_VOLZ_ENABLED                                 1752   2048   1840    
AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED           1744   1736   1736    
AP_SERVORELAYEVENTS_ENABLED                     1696   1712   1712    
AP_EFI_CURRAWONG_ECU_ENABLED                    1680   1672   1672    
AP_RSSI_ENABLED                                 1672   1728   1808    
AP_COMPASS_IST8310_ENABLED                      1656   1656   1656    
AP_EFI_SERIAL_HIRTH_ENABLED                     1656   1688   1648    
AP_AIRSPEED_MS4525_ENABLED                      1648   1640   1640    
AP_BARO_BMP085_ENABLED                          1648   1648   1648    
AP_RANGEFINDER_WASP_ENABLED                     1648   1640   1640    
AP_GRIPPER_ENABLED                              1644   2068   1756    
HAL_NMEA_OUTPUT_ENABLED                         1632   1688   1688    
AP_SDCARD_STORAGE_ENABLED                       1624   1480   1488    
AP_GPS_ERB_ENABLED                              1616   1480   1552    
AP_AIRSPEED_MS5525_ENABLED                      1600   1600   1584    
AP_BARO_DPS280_ENABLED                          1600   1592   1592    
AP_BARO_SPL06_ENABLED                           1600   1592   1592    
AP_COMPASS_BMM150_ENABLED                       1600   1592   1592    
AP_TRAMP_ENABLED                                1584   1600   1568    
AP_COMPASS_RM3100_ENABLED                       1568   1560   1560    
AP_TEMPERATURE_SENSOR_ENABLED                   1560   1560   1552    
HAL_MOUNT_GREMSY_ENABLED                        1560   1560   1560    
AP_AIRSPEED_AUAV_ENABLED                        1544   1544   1528    
AP_MOTORS_FRAME_QUAD_ENABLED                    1544   1376   na      
AP_CAMERA_MAVLINKCAMV2_ENABLED                  1512   1512   1512    
AP_GPS_NOVA_ENABLED                             1512   1512   1512    
AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED      1472   1472   1472    
HAL_MOUNT_ALEXMOS_ENABLED                       1472   1464   1464    
AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED      1464   1456   1456    
AP_COMPASS_IST8308_ENABLED                      1424   1416   1416    
AP_BARO_FBM320_ENABLED                          1392   1392   1392    
AP_KDECAN_ENABLED                               1392   1248   1264    
AP_SCRIPTING_SERIALDEVICE_ENABLED               1376   1352   1336    
AP_RANGEFINDER_DRONECAN_ENABLED                 1360   1360   1360    
AP_DRONECAN_HIMARK_SERVO_SUPPORT                1352   1344   1344    
AP_BARO_BMP388_ENABLED                          1320   1320   1320    
AP_BARO_BMP280_ENABLED                          1312   1304   1304    
AP_COMPASS_QMC5883L_ENABLED                     1312   1304   1304    
AP_MOTORS_FRAME_OCTA_ENABLED                    1312   1312   na      
AP_RPM_PIN_ENABLED                              1248   1552   1552    
AP_CUSTOMROTATIONS_ENABLED                      1216   1232   1216    
AP_OPTICALFLOW_HEREFLOW_ENABLED                 1168   1160   1160    
HAL_MOUNT_STORM32SERIAL_ENABLED                 1156   1148   1148    
AP_FRSKY_D_TELEM_ENABLED                        1144   1136   1136    
HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED           1120   1120   1120    
AC_PRECLAND_IRLOCK_ENABLED                      1112   1112   1112    
AP_GPS_SIRF_ENABLED                             1104   968    1048    
AP_BARO_LPS2XH_ENABLED                          1040   1040   1040    
AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED           1024   1024   1024    
AP_RANGEFINDER_JRE_SERIAL_ENABLED               1016   1008   1008    
HAL_ADSB_UAVIONIX_MAVLINK_ENABLED               992    984    984     
AP_RPM_DRONECAN_ENABLED                         984    984    984     
AP_AIRSPEED_DLVR_ENABLED                        976    976    976     
AP_RANGEFINDER_TRI2C_ENABLED                    968    968    968     
HAL_PLUSCODE_ENABLE                             960    944    944     
AP_EXTENDED_ESC_TELEM_ENABLED                   944    936    928     
AP_RANGEFINDER_BLPING_ENABLED                   936    936    936     
AP_RANGEFINDER_NMEA_ENABLED                     936    928    880     
AP_MOTORS_FRAME_OCTAQUAD_ENABLED                896    896    na      
AP_CAMERA_SOLOGIMBAL_ENABLED                    888    888    888     
AP_RCPROTOCOL_SRXL_ENABLED                      888    888    888     
AP_CAMERA_MOUNT_ENABLED                         880    872    872     
HAL_MOUNT_CADDX_ENABLED                         880    872    872     
AP_AIRSPEED_ASP5033_ENABLED                     856    856    856     
HAL_OSD_SIDEBAR_ENABLE                          848    848    848     
AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED         832    832    832     
AP_EFI_NWPWU_ENABLED                            824    816    816     
AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED   824    824    824     
AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED            824    824    824     
HAL_MOUNT_STORM32MAVLINK_ENABLED                824    824    824     
AP_MAVLINK_MSG_DEVICE_OP_ENABLED                816    816    816     
AP_OPTICALFLOW_PX4FLOW_ENABLED                  816    808    808     
AP_RANGEFINDER_PWM_ENABLED                      808    800    800     
AP_BARO_EXTERNALAHRS_ENABLED                    792    792    800     
AP_RANGEFINDER_LEDDARVU8_ENABLED                776    776    776     
AP_NOTIFY_NCP5623_ENABLED                       768    760    760     
AP_OPTICALFLOW_CXOF_ENABLED                     768    760    760     
AP_RANGEFINDER_RDS02UF_ENABLED                  768    760    760     
AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED           760    760    760     
AP_CAMERA_SEND_FOV_STATUS_ENABLED               752    752    752     
AP_BARO_BMP581_ENABLED                          736    736    736     
AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED       728    720    752     
AP_OPTICALFLOW_UPFLOW_ENABLED                   728    728    728     
HAL_BARO_WIND_COMP_ENABLED                      712    680    728     
AP_EFI_SERIAL_LUTAN_ENABLED                     704    704    704     
AP_GPS_MAV_ENABLED                              704    704    704     
AP_MOTORS_FRAME_Y6_ENABLED                      704    704    na      
AP_RCPROTOCOL_ST24_ENABLED                      704    704    704     
AP_BATTERY_FUELFLOW_ENABLED                     696    696    696     
AP_RANGEFINDER_MAXSONARI2CXL_ENABLED            688    680    680     
AP_BATTERY_FUELLEVEL_ANALOG_ENABLED             680    672    672     
AP_BATTERY_SUM_ENABLED                          680    672    672     
AP_MOTORS_FRAME_HEXA_ENABLED                    672    672    na      
AP_RANGEFINDER_LEDDARONE_ENABLED                672    672    672     
AP_RANGEFINDER_USD1_SERIAL_ENABLED              640    640    640     
AP_RCPROTOCOL_SUMD_ENABLED                      616    608    608     
HAL_TORQEEDO_ENABLED                            608    608    7992    
AP_CAMERA_SERVO_ENABLED                         600    600    600     
AP_RCPROTOCOL_SBUS_ENABLED                      592    584    592     
AP_COMPASS_EXTERNALAHRS_ENABLED                 584    584    600     
AP_SBUSOUTPUT_ENABLED                           560    472    544     
AP_OPTICALFLOW_MAV_ENABLED                      552    544    544     
AP_CAN_LOGGING_ENABLED                          544    416    488     
COMPASS_LEARN_ENABLED                           512    504    504     
AP_RANGEFINDER_HC_SR04_ENABLED                  504    504    504     
AP_BATTERY_WATT_MAX_ENABLED                     496    na     na      
AP_OAPATHPLANNER_ENABLED                        488    19016  16768   
AP_RANGEFINDER_ANALOG_ENABLED                   488    480    480     
AP_RANGEFINDER_MAVLINK_ENABLED                  488    488    488     
AP_GENERATOR_IE_650_800_ENABLED                 472    464    464     
AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED          456    448    448     
AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED        448    448    448     
AP_RANGEFINDER_NOOPLOOP_ENABLED                 440    432    432     
AP_RCPROTOCOL_IBUS_ENABLED                      440    432    432     
AP_AVOIDANCE_ENABLED                            432    15680  13232   
AP_BATTERY_FUELLEVEL_PWM_ENABLED                392    392    392     
AP_BATTERY_SYNTHETIC_CURRENT_ENABLED            392    392    392     
AP_RANGEFINDER_LANBAO_ENABLED                   392    384    384     
AP_RANGEFINDER_GYUS42V2_ENABLED                 384    384    384     
AP_MAVLINK_MSG_RELAY_STATUS_ENABLED             376    376    376     
AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED             376    368    368     
AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED             360    na     na      
AP_CAMERA_MAVLINK_ENABLED                       352    344    344     
AP_RANGEFINDER_LUA_ENABLED                      352    352    352     
AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED          352    344    344     
AP_MOTORS_FRAME_DODECAHEXA_ENABLED              344    344    na      
AC_PRECLAND_COMPANION_ENABLED                   320    312    312     
AP_EFI_MAV_ENABLED                              312    312    312     
AP_CAMERA_RELAY_ENABLED                         304    296    296     
AP_MOTORS_FRAME_DECA_ENABLED                    296    296    na      
HAL_SPRAYER_ENABLED                             280    1224   1016    
AP_FILESYSTEM_FORMAT_ENABLED                    272    272    272     
AP_RANGEFINDER_BENEWAKE_CAN_ENABLED             272    264    264     
AP_NOTIFY_NEOPIXEL_ENABLED                      264    264    264     
AP_INERTIALSENSOR_KILL_IMU_ENABLED              256    240    240     
AP_MAVLINK_SERVO_RELAY_ENABLED                  256    248    248     
AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED            256    256    256     
AP_AIRSPEED_ANALOG_ENABLED                      248    248    248     
AP_NOTIFY_PROFILED_ENABLED                      224    216    216     
AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED          216    208    208     
AP_RANGEFINDER_USD1_CAN_ENABLED                 216    208    208     
AP_RCPROTOCOL_PPMSUM_ENABLED                    216    208    208     
AP_CAMERA_SEND_THERMAL_RANGE_ENABLED            176    176    176     
AP_RPM_HARMONICNOTCH_ENABLED                    176    176    176     
AP_RANGEFINDER_BENEWAKE_TF02_ENABLED            144    144    144     
AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED 136    136    136     
AP_RANGEFINDER_BENEWAKE_TF03_ENABLED            136    136    136     
AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED          136    136    136     
AP_RPM_GENERATOR_ENABLED                        136    128    128     
AP_RPM_EFI_ENABLED                              128    128    128     
AP_RPM_ESC_TELEM_ENABLED                        128    120    120     
AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED     96     88     88      
AP_PROXIMITY_RANGEFINDER_ENABLED                80     792    792     
AP_WINCH_ENABLED                                64     5060   64      
AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED            40     2096   32      
AP_AIS_ENABLED                                  24     1048   7404    
AP_PROXIMITY_CYGBOT_ENABLED                     na     936    936     
AP_PROXIMITY_MR72_ENABLED                       na     na     na      
AP_SERIALMANAGER_IMUOUT_ENABLED                 na     na     na      
AP_TEMPCALIBRATION_ENABLED                      na     1328   na      
AP_WINCH_DAIWA_ENABLED                          na     2452   na      
AP_WINCH_PWM_ENABLED                            na     560    na      
HAL_HOTT_TELEM_ENABLED                          na     na     na      
HAL_MOUNT_XFROBOT_ENABLED                       na     na     na      
MODE_BRAKE_ENABLED                              na     688    na      
MODE_FLIP_ENABLED                               na     1296   na      
MODE_FLOWHOLD_ENABLED                           na     4616   na      
MODE_FOLLOW_ENABLED                             na     2296   1488    
MODE_GUIDED_NOGPS_ENABLED                       na     240    na      
MODE_SPORT_ENABLED                              na     na     na      
MODE_SYSTEMID_ENABLED                           na     4208   na      
MODE_TURTLE_ENABLED                             na     1896   na      
MODE_ZIGZAG_ENABLED                             na     4784   na      
AP_COPTER_AHRS_AUTO_TRIM_ENABLED                na     472    na      
AP_DRONECAN_VOLZ_FEEDBACK_ENABLED               na     na     na      
AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED    na     2968   na      
AP_PROXIMITY_LIGHTWARE_SF40C_ENABLED            na     1816   1816    
AP_PROXIMITY_LIGHTWARE_SF45B_ENABLED            na     1760   1760    
AP_PROXIMITY_RPLIDARA2_ENABLED                  na     1504   1504    
AP_PROXIMITY_TERARANGERTOWEREVO_ENABLED         na     992    992     
AP_PROXIMITY_TERARANGERTOWER_ENABLED            na     656    656     
AP_PROXIMITY_MAV_ENABLED                        na     1800   1800    
AP_AIRSPEED_NMEA_ENABLED                        na     na     888     
=============================================== ====== ====== ====== 

Build Server Versions
=====================
Before deploying new features to the main instance of the application, we test them on a dedicated testing instance. This testing environment is accessible at `custom-beta.ardupilot.org <https://custom-beta.ardupilot.org>`__. Feel free to use the beta features and provide your valuable feedback to help us improve the application.


[copywiki destination="copter,plane,rover,planner,blimp,sub"]
