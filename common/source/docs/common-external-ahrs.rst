.. _common-external-ahrs:

=====================
External AHRS Systems
=====================

Rather than using ArduPilot's internal Attitude Heading Reference System (AHRS) for attitude, heading and position, it is possible to use several external systems.

Supported Systems
=================

Currently, ArduPilot supports these systems:

- `Parker Lord 3DM® Series <https://www.microstrain.com/inertial-sensors/all-sensors>`_
- `VectorNav VN-300 AHRS <https://www.vectornav.com/products>`__
- `VectorNav VN-100AHRS <https://www.vectornav.com/products>`__

Setup
=====
VectorNav300 or Parker Lord
---------------------------

    - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)

    - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV) or 2 (Parker Lord)

This will replace ArduPilot’s internally generated INS/AHRS subsystems with the external system

.. note::
    When setting up the VN-300, you may need to configure unique update rates for different data. You can achieve this by running the following two commands:
    - ``$VNWRG,75,3,8,34,072E,0106,0612*0C``
    - ``$VNWRG,76,3,80,4E,0002,0010,20B8,0018*63``

VN-300 data is expected at two different update rates, 50Hz and 5Hz.
**50Hz**
* Group 3 (GPS)
    * UncompMag
    * UncompAccel
    * UncompGyro
    * Pres
    * Mag 
    * Accel
    * AngularRate 
* Group 5 (Attitude)
    * YawPitchRoll
    * Quaternion
    * YprU
* Group 6 (INS)
    * PosLLa
    * VelNed
    * PosU 
    * VelU 
**5Hz**
* Group 2 (Time)
    * TimeGps
* Group 3 (IMU)
    * Temp 
* Group 4 (GPS1)
    * NumSats
    * Fix
    * PosLLa
    * VelNed
    * DOP
* Group 7 (GPS2)
    * NumSats
    * DOP


VectorNav100
------------

    - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 3 (ArduPilot's EKF3)

    - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV)

    - :ref:`EAHRS_OPTIONS<EAHRS_OPTIONS>` bit 0 set to 1 ("1" value) to disable its compensation of the sensor biases, letting EKF3 do that (since there is no internal GPS to provide the best estimates)

- for all of the above, set the ``SERIALx_PROTOCOL`` to “36” (AHRS) and ``SERIALx_BAUD`` to “115” (unless you have changed the external unit’s baud rate from its default value) for the port which is connected to the external AHRS unit.

This will replace ArduPilot's internally generated INS/AHRS subsystems with the external system.

In addition, instead of replacing ArduPilot's INS/AHRS systems, it is possible to use an external AHRS system's sensors (IMU,GPS, Compass, and Barometer) as additional sensors for use with ArduPilot's INS/AHRS systems. Simply do not change the :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` from "3" (EKF3), but setup the others parameters as discussed above.
