.. _input-shaping:

====================================
Setting the input shaping parameters
====================================

Copter has a set of parameters that define the way the aircraft feels to fly. This allows the aircraft to be set up with a very aggressive tune but still feel like a very docile and friendly aircraft to fly.

The most important of these parameters is:

- :ref:`ACRO_YAW_P <ACRO_YAW_P>` : desired maximum yaw rate / 45 degrees/s
- :ref:`ANGLE_MAX <ANGLE_MAX>` :  maximum lean angle
- :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>` : Pitch rate acceleration
- :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>` : Roll rate acceleration
- :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>` : Yaw rate acceleration
- :ref:`ATC_ANG_LIM_TC <ATC_ANG_LIM_TC>` : Aircraft smoothing time

Autotune will set the :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>`, :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>` and :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>` parameters to their maximum based on measurements done during the Autotune tests. These values should not be increased beyond what Autotune suggests without careful testing. In most cases pilots will want to reduce these values significantly.

For aircraft designed to carry large directly mounted payloads, the maximum values of :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>`, :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>` and :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>` should be reduced based on the minimum and maximum takeoff weight (TOW):

- :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>`  x (min_TOW / max_TOW)
- :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>`  x (min_TOW / max_TOW)
- :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>`  x (min_TOW / max_TOW)

:ref:`ACRO_YAW_P <ACRO_YAW_P>` should be set to be approximately 0.5 x :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>` / 4500 to ensure that the aircraft can achieve full yaw rate in approximately half a second.

:ref:`ATC_ANG_LIM_TC <ATC_ANG_LIM_TC>` may be increased to provide a very smooth feeling on the sticks at the expense of a slower reaction time.

Aerobatic aircraft should keep the :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>`, :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>` and :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>` provided by autotune and reduce :ref:`ATC_ANG_LIM_TC <ATC_ANG_LIM_TC>` to achieve the stick feel desired by the pilot. For pilots wanting to fly ACRO the following input shaping parameters can be used to tune the feel of ACRO:

- :ref:`ACRO_BAL_PITCH <ACRO_BAL_PITCH>`
- :ref:`ACRO_BAL_ROLL <ACRO_BAL_ROLL>`
- :ref:`ACRO_RP_EXPO <ACRO_RP_EXPO>`
- :ref:`ACRO_RP_P <ACRO_RP_P>`
- :ref:`ACRO_THR_MID <ACRO_THR_MID>`
- :ref:`ACRO_TRAINER <ACRO_TRAINER>`
- :ref:`ACRO_Y_EXPO <ACRO_Y_EXPO>`
- :ref:`ACRO_YAW_P <ACRO_YAW_P>`

The full list of input shaping parameters are:

- :ref:`ACRO_BAL_PITCH <ACRO_BAL_PITCH>`
- :ref:`ACRO_BAL_ROLL <ACRO_BAL_ROLL>`
- :ref:`ACRO_RP_EXPO <ACRO_RP_EXPO>`
- :ref:`ACRO_RP_P <ACRO_RP_P>`
- :ref:`ACRO_THR_MID <ACRO_THR_MID>`
- :ref:`ACRO_TRAINER <ACRO_TRAINER>`
- :ref:`ACRO_Y_EXPO <ACRO_Y_EXPO>`
- :ref:`ACRO_YAW_P <ACRO_YAW_P>`
- :ref:`ANGLE_MAX <ANGLE_MAX>`
- :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>`
- :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>`
- :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>`
- :ref:`ATC_ANG_LIM_TC <ATC_ANG_LIM_TC>`
- :ref:`ATC_RATE_P_MAX <ATC_RATE_P_MAX>`
- :ref:`ATC_RATE_R_MAX <ATC_RATE_R_MAX>`
- :ref:`ATC_RATE_Y_MAX <ATC_RATE_Y_MAX>`
- :ref:`ATC_SLEW_YAW <ATC_SLEW_YAW>`
- :ref:`PILOT_ACCEL_Z <PILOT_ACCEL_Z>`
- :ref:`PILOT_SPEED_DN <PILOT_SPEED_DN>`
- :ref:`PILOT_SPEED_UP <PILOT_SPEED_UP>`
- :ref:`PILOT_THR_BHV <PILOT_THR_BHV>`
- :ref:`PILOT_THR_FILT <PILOT_THR_FILT>`
- :ref:`PILOT_TKOFF_ALT <PILOT_TKOFF_ALT>`
- :ref:`LOIT_ACC_MAX <LOIT_ACC_MAX>`
- :ref:`LOIT_ANG_MAX <LOIT_ANG_MAX>`
- :ref:`LOIT_BRK_ACCEL <LOIT_BRK_ACCEL>`
- :ref:`LOIT_BRK_DELAY <LOIT_BRK_DELAY>`
- :ref:`LOIT_BRK_JERK <LOIT_BRK_JERK>`
- :ref:`LOIT_SPEED <LOIT_SPEED>`