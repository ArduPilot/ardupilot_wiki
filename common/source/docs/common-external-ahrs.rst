.. _common-external-ahrs:

=====================
External AHRS Systems
=====================

Rather than using ArduPilot's internal Attitude Heading Reference System (AHRS) for attitude, heading and position, it is possible to use several external systems.

Supported Systems
=================

Currently, ArduPilot supports these systems:

- `MicroStrain 3DM® Series <https://www.microstrain.com/inertial-sensors/all-sensors>`_
- `VectorNav <https://www.vectornav.com/products>`__
- `Inertialabs INS-P <https://inertiallabs.com/wp-content/uploads/2023/09/INS-B-P-D-DL_Datasheet_rev-6.27_August_2023.pdf>`__

Setup
=====

VectorNav
~~~~~~~~~
Refer to :ref:`common-external-ahrs-vectornav` for setup.

MicroStrain
-----------

    - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)

    - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV), 2 (MicroStrain5), 5(InertialLabs), or 7(MicroStrain7)

This will replace ArduPilot’s internally generated INS/AHRS subsystems with the external system.
The MicroStrain system must be configured via `Sensor Connect <https://www.microstrain.com/software/sensorconnect>`__ before use.
See the below section on data rates.


MicroStrain7
------------

The 3DM-GQ7 supports up to 230k baud on the 3.3V TTL serial interface.
The RS-232 interface supports higher baud rates, but most autopilots do not come with RS-232 interfaces.

To use the 3DM-GQ7 with 230k baud in ArduPilot, set the following data rates in SensorConnect.

* IMU

  * Enable the Time Field at 50Hz

  * Scaled Ambient Pressure: 50Hz

  * Magnetometer Vector: 50Hz

  * Accelerometer Vector: 50Hz

  * Gyroscope Vector: 50Hz

  * Attitude (Quaternion): 50Hz

* GNSS Receiver 1 and 2

  * Enable the Time Field at 2Hz

  * UTC Time: 2Hz

  * GPS Fix Information: 2Hz

  * Position (LLH): 2Hz

  * DOP Data: 2Hz

  * Velocity (NED): 2Hz

* Estimation Filter

  * Position (LLH): 50Hz

  * Filter Status: 50Hz

  * Velocity (NED): 50Hz

  * Attitude (Quaternion): 50Hz

  * Position Uncertainty (LLH): 50Hz

  * Velocity Uncertainty (NED): 50Hz

This rates work well for platforms that run the EKF at 50Hz, such as Plane, Sub, Blimp.
MicroStrain hardware and firmware for higher baud rates than 230k is in development.
With higher baud rates, MicroStrain7 should be suitable for Copter and QuadPlane.

.. toctree::
    :hidden:

    common-external-ahrs-vectornav

