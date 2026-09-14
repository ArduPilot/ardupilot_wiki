:orphan:

.. _common-param-name-changes:

[copywiki destination="copter,plane,rover,sub,blimp,antennatracker"]

=======================
Parameter Name Changes
=======================

Parameters get renamed, rescaled, or reorganized fairly often, mostly to
standardize units (converting centidegrees/cm values to degrees/meters) and
to give related parameters a consistent naming scheme. This page lists old
and new parameter names across recent ArduPilot releases so that users
upgrading can find their previous settings.

.. note:: Automatic parameter conversion runs on first boot with the newer
   firmware, so old values normally carry over to the new parameter names on
   the vehicle itself. This page is mainly useful for updating scripts,
   ground station profiles, log analysis tools, finding parameters in an old firmware
   revisions on GCSs, and other external references that still use the old names.

ArduPilot 4.6 → 4.7
====================

These renames shipped in the 4.7.0 release

GCS/MAVLink streamrate and system ID parameters
------------------------------------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - SR0_RAW_SENS ... SR9_RAW_SENS
     - MAV1_RAW_SENS ... MAV10_RAW_SENS
   * - SR0_EXT_STAT ... SR9_EXT_STAT
     - MAV1_EXT_STAT ... MAV10_EXT_STAT
   * - SR0_RC_CHAN ... SR9_RC_CHAN
     - MAV1_RC_CHAN ... MAV10_RC_CHAN
   * - SR0_RAW_CTRL ... SR9_RAW_CTRL
     - MAV1_RAW_CTRL ... MAV10_RAW_CTRL
   * - SR0_POSITION ... SR9_POSITION
     - MAV1_POSITION ... MAV10_POSITION
   * - SR0_EXTRA1 ... SR9_EXTRA1
     - MAV1_EXTRA1 ... MAV10_EXTRA1
   * - SR0_EXTRA2 ... SR9_EXTRA2
     - MAV1_EXTRA2 ... MAV10_EXTRA2
   * - SR0_EXTRA3 ... SR9_EXTRA3
     - MAV1_EXTRA3 ... MAV10_EXTRA3
   * - SR0_PARAMS ... SR9_PARAMS
     - MAV1_PARAMS ... MAV10_PARAMS
   * - SR0_ADSB ... SR9_ADSB
     - MAV1_ADSB ... MAV10_ADSB
   * - SYSID_MYGCS
     - MAV_GCS_SYSID
   * - TELEM_DELAY
     - MAV_TELEM_DELAY
   * - SYSID_ENFORCE
     - Replaced by MAV_OPTIONS bit 0 ("Accept MAVLink only from SYSID_GCS")
   * - SYSID_THISMAV
     - MAV_SYSID

Serial port option bits moved to MAVn_OPTIONS
------------------------------------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - SERIALn_OPTIONS bit 10 (don't forward MAVLink)
     - MAVn_OPTIONS bit 1
   * - SERIALn_OPTIONS bit 12 (ignore stream rate override)
     - MAVn_OPTIONS bit 2

Attitude control
------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - ATC_ACCEL_R_MAX
     - ATC_ACC_R_MAX
   * - ATC_ACCEL_P_MAX
     - ATC_ACC_P_MAX
   * - ATC_ACCEL_Y_MAX
     - ATC_ACC_Y_MAX
   * - ATC_SLEW_YAW (centidegrees/s)
     - ATC_RATE_WPY_MAX (degrees/s)
   * - Q_A_ACCEL_R_MAX
     - Q_A_ACC_R_MAX
   * - Q_A_ACCEL_P_MAX
     - Q_A_ACC_P_MAX
   * - Q_A_ACCEL_Y_MAX
     - Q_A_ACC_Y_MAX
   * - Q_A_SLEW_YAW (centidegrees/s)
     - Q_A_RATE_WPY_MAX (degrees/s)
   * - ANGLE_MAX (centidegrees)
     - ATC_ANGLE_MAX (Copter/Sub) or Q_A_ANGLE_MAX (QuadPlane), in degrees

Position controller (NEU → NED axis naming)
----------------------------------------------

AC_PosControl's vertical ("Z"/"D") and horizontal ("XY"/"NE") axis groups
were renamed to use consistent NED-style naming. This affects Copter/Sub
(prefix ``PSC_``) and QuadPlane (prefix ``Q_P_`` instead of ``PSC_``).

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - PSC_POSZ_P
     - PSC_D_POS_P
   * - PSC_VELZ_P / _I / _IMAX / _D / _FF / _FLTE / _FLTD
     - PSC_D_VEL_P / _I / _IMAX / _D / _FF / _FLTE / _FLTD
   * - PSC_ACCZ_P / _I / _IMAX / _D / _FF / _FLTT / _FLTE / _FLTD / _SMAX / _PDMX / _D_FF / _NTF / _NEF
     - PSC_D_ACC_P / _I / _IMAX / _D / _FF / _FLTT / _FLTE / _FLTD / _SMAX / _PDMX / _D_FF / _NTF / _NEF
   * - PSC_POSXY_P
     - PSC_NE_POS_P
   * - PSC_VELXY_P / _I / _D / _IMAX / _FLTE / _FLTD / _FF
     - PSC_NE_VEL_P / _I / _D / _IMAX / _FLTE / _FLTD / _FF
   * - PSC_JERK_XY
     - PSC_JERK_NE
   * - PSC_JERK_Z
     - PSC_JERK_D

.. note:: For QuadPlane, replace the ``PSC_`` prefix above with ``Q_P_``
   (e.g. ``Q_P_NE_POS_P``).

Avoidance
------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - AVOID_ANGLE_MAX (centidegrees)
     - AVOID_ANG_MAX (degrees)

Circle mode
--------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - CIRCLE_RADIUS (cm)
     - CIRCLE_RADIUS_M (m)

Loiter mode
--------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - LOIT_SPEED (cm/s)
     - LOIT_SPEED_MS (m/s)
   * - LOIT_ACC_MAX (cm/s/s)
     - LOIT_ACC_MAX_M (m/s/s)
   * - LOIT_BRK_ACCEL (cm/s/s)
     - LOIT_BRK_ACC_M (m/s/s)
   * - LOIT_BRK_JERK (cm/s/s/s)
     - LOIT_BRK_JRK_M (m/s/s/s)

LOIT_ANG_MAX, LOIT_BRK_DELAY and LOIT_OPTIONS were not renamed.

.. note:: For QuadPlane, the same four renames apply with the ``Q_LOIT_``
   prefix (e.g. ``Q_LOIT_SPEED`` → ``Q_LOIT_SPEED_MS``).

Waypoint navigation (``WPNAV_`` → ``WP_``)
--------------------------------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - WPNAV_SPEED
     - WP_SPD
   * - WPNAV_RADIUS
     - WP_RADIUS_M
   * - WPNAV_SPEED_UP
     - WP_SPD_UP
   * - WPNAV_SPEED_DN
     - WP_SPD_DN
   * - WPNAV_ACCEL
     - WP_ACC
   * - WPNAV_ACCEL_Z
     - WP_ACC_Z
   * - WPNAV_ACCEL_C
     - WP_ACC_CNR
   * - WPNAV_JERK
     - WP_JERK
   * - WPNAV_RFND_USE
     - WP_RFND_USE
   * - WPNAV_TER_MARGIN
     - WP_TER_MARGIN

.. note:: For QuadPlane, the same renames apply to the ``Q_WP_`` parameters
   (e.g. ``Q_WP_SPEED`` → ``Q_WP_SPD``, ``Q_WP_RADIUS`` → ``Q_WP_RADIUS_M``).

Copter Land mode
-------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - LAND_SPEED
     - LAND_SPD_MS
   * - LAND_SPEED_HIGH
     - LAND_SPD_HIGH_MS
   * - LAND_ALT_LOW
     - LAND_ALT_LOW_M

Copter RTL mode
------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - RTL_ALT
     - RTL_ALT_M
   * - RTL_SPEED
     - RTL_SPEED_MS
   * - RTL_ALT_FINAL
     - RTL_ALT_FINAL_M
   * - RTL_CLIMB_MIN
     - RTL_CLIMB_MIN_M

Copter PosHold mode
-----------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - PHLD_BRAKE_RATE
     - PHLD_BRK_RATE
   * - PHLD_BRAKE_ANGLE (centidegrees)
     - PHLD_BRK_ANGLE (degrees)

Pilot input parameters
--------------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - PILOT_TKOFF_ALT
     - PILOT_TKO_ALT_M
   * - PILOT_SPEED_UP
     - PILOT_SPD_UP
   * - PILOT_SPEED_DN
     - PILOT_SPD_DN
   * - PILOT_ACCEL_Z
     - PILOT_ACC_Z

EKF3
-------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - EK3_MAX_FLOW
     - EK3_FLOW_MAX

Rangefinders
---------------

Rangefinder distances moved from centimetres to metres.

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - RNGFNDx_MIN_CM (cm)
     - RNGFNDx_MIN (m)
   * - RNGFNDx_MAX_CM (cm)
     - RNGFNDx_MAX (m)
   * - RNGFNDx_GNDCLEAR (cm)
     - RNGFNDx_GNDCLR (m)

Sub EKF origin backup
------------------------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - ORIGIN_LAT
     - AHRS_ORIGIN_LAT
   * - ORIGIN_LON
     - AHRS_ORIGIN_LON
   * - ORIGIN_ALT
     - AHRS_ORIGIN_ALT

Plane altitude (glide) slope
-------------------------------

"Glide slope" was renamed to "altitude slope", since it also covers climbs.

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - GLIDE_SLOPE_MIN
     - ALT_SLOPE_MIN
   * - GLIDE_SLOPE_THR
     - ALT_SLOPE_MAXHGT

RunCam
---------

RunCam is now a camera backend, so its parameters moved under camera
instance 2 and the enable is CAM2_TYPE = 4 (RunCam).

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - CAM_RC_TYPE
     - CAM2_RC_TYPE
   * - CAM_RC_FEATURES
     - CAM2_RC_FEATURES
   * - CAM_RC_BT_DELAY
     - CAM2_RC_BT_DELAY
   * - CAM_RC_BTN_DELAY
     - CAM2_RC_BTN_DELY
   * - CAM_RC_MDE_DELAY
     - CAM2_RC_MDE_DELY
   * - CAM_RC_CONTROL
     - CAM2_RC_CONTROL

.. note:: CAM2_RC_TYPE no longer has a "Disabled" meaning; whether RunCam
   runs is now decided by CAM2_TYPE.

Arming
---------

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - ARMING_CHECK
     - ARMING_SKIPCHK

.. warning:: This is not a pure rename. ``ARMING_CHECK`` was a bitmask of
   checks to *run*, while ``ARMING_SKIPCHK`` is a bitmask of checks to
   *skip*. The bit meanings are inverted, so a converted value should be
   double checked rather than assumed correct.

ArduPilot 4.7 → current master (pre-4.8)
===========================================

.. warning:: Master is under active, unreleased development for the
   upcoming 4.8 release. The renames below have merged as of the last time
   this page was updated, but **more parameter renames are likely to land
   before 4.8 ships**. Treat this section as a work-in-progress snapshot,
   not a final list — check the source or the eventual 4.8.0 release notes
   before relying on it for a production upgrade.

TradHeli swashplate phase angle
-----------------------------------

Phase adjustment support was added to all swashplate types, dropping the
"H3" naming from the phase angle parameter.

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - H_SW_H3_PHANG
     - H_SW_PHANG
   * - H_SW2_H3_PHANG
     - H_SW2_PHANG

SITL board mounting trim
-----------------------------

SITL's accelerometer-only trim parameter was replaced with a general rigid
board mounting offset applied to accelerometers, gyros, and compasses.

.. list-table::
   :header-rows: 1

   * - Old Name
     - New Name
   * - SIM_ACC_TRIM
     - SIM_BRD_TRIM
