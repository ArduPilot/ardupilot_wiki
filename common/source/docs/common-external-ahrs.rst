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

Setup
=====

- :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)
- :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV) or 2 (Parker Lord)
- set the ``SERIALx_PROTOCOL`` to "36" (AHRS) and ``SERIALx_BAUD`` to "115" (unless you have changed the external unit's baud rate from its default value) for the port which is connected to the external AHRS unit. 

This will replace ArduPilot's internally generated INS/AHRS subsystems with the external system.

In addition, instead of replacing ArduPilot's INS/AHRS systems, it is possible to use an external AHRS system's sensors (IMU,GPS, Compass, and Barometer) as additional sensors for use with ArduPilot's INS/AHRS systems. Simply do not change the :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` from "3" (EKF3), but setup the others parameters as discussed above.
