.. _traditional-helicopter-parameter-list:

=================================================
Traditional Helicopter Parameter List at a Glance
=================================================


This list is provided as a quick means to explore all the unique Single and Dual Rotor Helicopter parameters at a glance. To see the parameter description, click on it.

Vehicle Type/Configuration
==========================

These are basic setup parameters that need to be set in all cases.

- :ref:`FRAME_CLASS<FRAME_CLASS>`  Note: only Single Heli and Dual Heli are valid with TradHeli firmware
- :ref:`H_FLYBAR_MODE<H_FLYBAR_MODE>`
- :ref:`H_OPTIONS<H_OPTIONS>`

General Swashplate/Collective Setup
===================================

These set the cyclic max, allow bench testing of blade angle set up, and set features of the swashplate/collective.

- :ref:`H_COL_ANG_MIN<H_COL_ANG_MIN>`
- :ref:`H_COL_ANG_MAX<H_COL_ANG_MAX>`
- :ref:`H_COL_ZERO_THRST<H_COL_ZERO_THRST>`
- :ref:`H_COL_LAND_MIN<H_COL_LAND_MIN>`
- :ref:`H_CYC_MAX<H_CYC_MAX>`
- :ref:`H_SV_MAN<H_SV_MAN>`
- :ref:`H_SV_TEST<H_SV_TEST>`
- :ref:`H_HOVER_LEARN<H_HOVER_LEARN>`
- :ref:`H_COL_HOVER<H_COL_HOVER>`

Single Swashplate/Collective Setup
==================================

These set up the configuration of the swashplate and collective travel.

- :ref:`H_SW_TYPE<H_SW_TYPE__AP_MotorsHeli_Single>`
- :ref:`H_SW_H3_ENABLE<H_SW_H3_ENABLE__AP_MotorsHeli_Single>` Never manually set this parameter!
- ``H_COLYAW<H_COLYAW>`` This parameter will be eliminated in versions after 4.3
- :ref:`H_COL_MAX<H_COL_MAX>`
- ``H_COL_MID``  This parameter was used in versions prior to Copter 4.3.0
- :ref:`H_COL_MIN<H_COL_MIN>`
- :ref:`H_SW_COL_DIR<H_SW_COL_DIR__AP_MotorsHeli_Single>`
- :ref:`H_SW_LIN_SVO<H_SW_LIN_SVO__AP_MotorsHeli_Single>`

Dual Rotor Helicopter
=====================

If Dual Heli frame type is selected, these additional parameters for the second rotor and rotor interactions are enabled and visible.

- :ref:`H_DUAL_MODE<H_DUAL_MODE>`
- :ref:`H_DCP_SCALER<H_DCP_SCALER>`
- :ref:`H_DCP_YAW<H_DCP_YAW>`
- :ref:`H_YAW_SCALER<H_YAW_SCALER>`
- :ref:`H_COL2_MIN<H_COL2_MIN>`
- :ref:`H_COL2_MAX<H_COL2_MAX>`
- ``H_COL2_MID``  This parameter was used in versions prior to Copter 4.3.0
- :ref:`H_SW2_TYPE<H_SW2_TYPE>`
- :ref:`H_SW2_COL_DIR<H_SW2_COL_DIR>`
- :ref:`H_SW2_LIN_SVO<H_SW2_LIN_SVO>`
- :ref:`H_SW2_H3_ENABLE<H_SW2_H3_ENABLE>`
- :ref:`H_SW2_H3_SV1_POS<H_SW2_H3_SV1_POS>`
- :ref:`H_SW2_H3_SV2_POS<H_SW2_H3_SV2_POS>`
- :ref:`H_SW2_H3_SV3_POS<H_SW2_H3_SV3_POS>`
- :ref:`H_SW2_H3_PHANG<H_SW2_H3_PHANG>`
- :ref:`H_DCP_TRIM<H_DCP_TRIM>`
- :ref:`H_YAW_REV_EXPO<H_YAW_REV_EXPO>`

Custom Swashplate Configuration
===============================

ArduPilot allows custom swashplate servo placement for three servo swashplates. These parameters are active if :ref:`H_SW_TYPE<H_SW_TYPE__AP_MotorsHeli_Single>` = 0 (single heli):

- :ref:`H_SW_H3_SV1_POS<H_SW_H3_SV1_POS__AP_MotorsHeli_Single>`
- :ref:`H_SW_H3_SV2_POS<H_SW_H3_SV2_POS__AP_MotorsHeli_Single>`
- :ref:`H_SW_H3_SV3_POS<H_SW_H3_SV3_POS__AP_MotorsHeli_Single>`
- :ref:`H_SW_H3_PHANG<H_SW_H3_PHANG__AP_MotorsHeli_Single>`

 and if :ref:`H_SW2_TYPE<H_SW2_TYPE>` = 0 (dual helis only):

- :ref:`H_SW2_H3_SV1_POS<H_SW_H3_SV1_POS__AP_MotorsHeli_Dual>`
- :ref:`H_SW2_H3_SV2_POS<H_SW_H3_SV2_POS__AP_MotorsHeli_Dual>`
- :ref:`H_SW2_H3_SV3_POS<H_SW_H3_SV3_POS__AP_MotorsHeli_Dual>`
- :ref:`H_SW2_H3_PHANG<H_SW_H3_PHANG__AP_MotorsHeli_Dual>`

Rotor Speed Control Setup
=========================

The speed of the rotor, or rotors in the case of Dual Heli, is controlled by ArduPilot using the following RSC modes: RSC channel passthrough (not recommended), external governor, internal throttle curve, or internal governor. Parameters are provided for rotor spool time, throttle slew rate, and critical rotor speed.  The internal throttle curve and governor are mainly for use with ICE or turbine engines but can be used for electric-powered helis without ESC governing.

- :ref:`H_RSC_MODE<H_RSC_MODE>`
- :ref:`H_RSC_IDLE<H_RSC_IDLE>`
- :ref:`H_RSC_RAMP_TIME<H_RSC_RAMP_TIME>`
- :ref:`H_RSC_RUNUP_TIME<H_RSC_RUNUP_TIME>`
- :ref:`H_RSC_SLEWRATE<H_RSC_SLEWRATE>`
- :ref:`H_RSC_CRITICAL<H_RSC_CRITICAL>`
- :ref:`H_RSC_CLDWN_TIME<H_RSC_CLDWN_TIME>`
- ``H_RSC_AROT_PCT`` Currently only available in SITL

External Governor
=======================

Parameter for an external rotor speed governor like an ESC governor, if used.

- :ref:`H_RSC_SETPOINT<H_RSC_SETPOINT>`

Internal Throttle Curve
=======================

Parameters for internal throttle curve, if used.

- :ref:`H_RSC_THRCRV_0<H_RSC_THRCRV_0>`
- :ref:`H_RSC_THRCRV_25<H_RSC_THRCRV_25>`
- :ref:`H_RSC_THRCRV_50<H_RSC_THRCRV_50>`
- :ref:`H_RSC_THRCRV_75<H_RSC_THRCRV_75>`
- :ref:`H_RSC_THRCRV_100<H_RSC_THRCRV_100>`

ArduPilot Provided Rotor Speed Governor
=======================================

Parameters for internal rotor speed governor, if used. The transmitter's throttle curve will need to be set up to use this mode. See the Setup section of :ref:`traditional-helicopter-internal-rsc-governor`.

- ``H_RSC_GOV_DISGAG``  This parameter was used in versions prior to Copter 4.3.0
- :ref:`H_RSC_GOV_DROOP<H_RSC_GOV_DROOP>`
- :ref:`H_RSC_GOV_RANGE<H_RSC_GOV_RANGE>`
- ``H_RSC_GOV_SETPNT``  This parameter was used in versions prior to Copter 4.3.0
- ``H_RSC_GOV_TCGAIN``  This parameter was used in versions prior to Copter 4.3.0
- :ref:`H_RSC_GOV_COMP<H_RSC_GOV_COMP>`
- :ref:`H_RSC_GOV_FF<H_RSC_GOV_FF>`
- :ref:`H_RSC_GOV_RPM<H_RSC_GOV_RPM>`
- :ref:`H_RSC_GOV_TORQUE<H_RSC_GOV_TORQUE>`

Tail Setup
==========

The type of tail rotor/control used is determined by these parameters.

- :ref:`H_TAIL_SPEED<H_TAIL_SPEED>`
- :ref:`H_TAIL_TYPE<H_TAIL_TYPE>`


External Tail Gyro
==================

Parameters are provided for external tail gyro, if used.

- :ref:`H_GYR_GAIN<H_GYR_GAIN>`
- :ref:`H_GYR_GAIN_ACRO<H_GYR_GAIN_ACRO>`

