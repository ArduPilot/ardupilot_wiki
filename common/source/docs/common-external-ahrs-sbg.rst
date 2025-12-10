.. _common-external-ahrs-sbg:

=========================
SBG Systems External AHRS
=========================

`SBG Systems <https://www.sbg-systems.com>` designs, manufactures, and support an extensive range of state-of-the-art inertial sensors such as Inertial Measurement Units (IMU), Attitude and Heading Reference Systems (AHRS), Inertial Navigation Systems with embedded GNSS (INS/GNSS), and so on.

ArduPilot can use these sensors as an :ref:`external AHRS <common-external-ahrs>`, bypassing/replacing the EKF3 estimator, or as a source of raw sensor data provided to the estimator.

SBG Systems products provide a range of benefits to ArduPilot users and can be integrated for:

- Higher accuracy heading, pitch, and roll estimates
- More robust and reliable GNSS positioning
- Improved positioning and attitude performance in GNSS-contested environments
- Performance under challenging dynamic conditions (e.g. catapult launches, VTOL operations, high-g or high angular rate operations)

The SBG driver is streamlined to provide a simple plug-and-play architecture, removing engineering obstacles and allowing the acceleration of the design, development, and launch of platforms to keep pace with the rapid rate of innovation.

The driver supports `all SBG Systems products <https://www.sbg-systems.com/products>`.
In particular the following systems are recommended:

- **Pulse:** Recommended for fixed-wing systems without hovering, where static heading is not necessary.
- **Ellipse:** Recommended for multicopter systems where hovering and low dynamics requires the use of static heading.

SBG Systems solutions are available directly from `MySBG <https://my.sbg-systems.com>` (FR) or through their Global Sales Representatives. For more information on their solutions or for international orders, please contact contact@sbg-systems.com.

Hardware Setup
==============

Wiring
------
Connect any unused flight controller serial interface, such as a spare `GPS` or `TELEM` port, to the SBG Systems product MAIN port.

Mounting
--------
The SBG Systems product sensor can be mounted in any orientation, in any position on the vehicle, without regard to center of gravity.

All SBG Systems product sensors default to a coordinate system of x-forward, y-right, and z-down, making the default mounting as connector-back, base down.

This can be changed to any rigid rotation using the sbgECom Reference Frame Rotation register.

If using a GNSS-enabled product, the GNSS antenna must be mounted rigidly with respect to the inertial sensor and with an unobstructed sky view. If using a dual-GNSS-enabled product (Ellipse-D), the secondary antenna must be mounted rigidly with respect to the primary antenna and the inertial sensor with an unobstructed sky view.

For more mounting and configuration requirements and recommendations, see the relevant `SBG SUPPORT CENTER <https://support.sbg-systems.com/sc>`.

Sensor Configuration
====================
To configure any SBG Systems product, please refer to `SBG SUPPORT CENTER <https://support.sbg-systems.com/sc>`.

ArduPilot Configuration
=======================
There are two possible ways for sbgECom data to be used by ArduPilot: as an external sensor set to ArduPilot's EKFs or as an external AHRS. Both ways utilize ArduPilot's External AHRS driver.

To establish communication with the SBG Systems unit, set the following for the relevant serial port:

  - ``SERIALx_PROTOCOL`` = 36 (AHRS)
  - ``SERIALx_BAUD`` = matching SBG Systems sensor baudrate

.. tip::
  The External AHRS-specific parameters may not be visible before the ``Serialx_Protocol`` parameter is configured. As such, either a Refresh Params or a reset of ArduPilot may be necessary to see the parameters.

Use as an External Sensor Set
-----------------------------
If set up as an external sensor, SBG Systems' raw sensor data (IMU, GNSS, Compass, and GNSS, if available) can be used by ArduPilot's internal EKFs, as configured. After the serial parameters have been configured, configure:

- :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 3 (ArduPilot’s EKF3)
- :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 8 (SBG Systems)
- :ref:`GPS1_TYPE<GPS1_TYPE>` = 21 (External AHRS) (If using a GNSS-enabled unit)
- :ref:`GPS2_TYPE<GPS2_TYPE>` = 21 (External AHRS) (If using a Dual GNSS-enabled unit)

If desired,
  - :ref:`EAHRS_SENSORS<EAHRS_SENSORS>` may be used to specify which sensor data should be used by ArduPilot's filters.
  - :ref:`EAHRS_OPTIONS<EAHRS_OPTIONS>` bit 1 may be set to "1" value to use SBG EKF Navigation logs as GNSS input instead of SBG GNSS logs.


Use as an External AHRS
-----------------------
Configuring ArduPilot to use the SBG Systems sensor as an External AHRS will use the SBG Systems solution as canonical rather than one of the possible internal ArduPilot filters.
This will allow ArduPilot to use the SBG Systems sensor's INS data that combines IMU and GNSS data in an advanced Kalman filtering estimation to provide position, velocity, and attitude estimates of higher accuracies and with better dynamic performance.

.. note::
  SBG Systems uses the term AHRS to refer to an attitude-only solution, without absolute position measurement input. SBG Systems uses the term INS to refer to a solution which accepts a position (often GNSS) measurement input and outputs a full PVTA. Because ArduPilot's External AHRS driver requires the data source to provide an absolute PVT, use as an External AHRS is restricted to a SBG Systems INS-enabled product (Ellipse & High Perfomance INS).

After the serial parameters have been configured, configure:
  - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)
  - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 8 (SBG Systems)

Published Data
==============

The SBG driver is configured to handle to following incomming messages:

- `UTC Time <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_UTC_TIME>`: UTC time reference.
- `IMU Data <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_IMU_DATA>`: accelerations and rotation rates.
- `IMU Short <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_IMU_SHORT>`: accelerations and rotation rates.
- `Magnetic Data <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_MAG>`: calibrated magnetometer data.
- `GNSS Position <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_GPSX_POS>`: position information from the GNSS receiver.
- `GNSS Velocity <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_GPSX_VEL>`: velocity and course information from the GNSS receiver.
- `EKF Euler <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_EKF_EULER>`: INS orientation using Euler angles.
- `EKF Quaternion <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_EKF_QUAT>`: INS orientation using quaternions.
- `EKF Navigation <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_EKF_NAV>`: INS velocity and position.
- `Air Data <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_AIR_DATA>`: barometric altitude and true airspeed.

Moreover, the SBG driver is also configured to send the following messages from the autopilot to the INS to improve Extended Kalman Filter from external sensors:

- `Air Data <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_AIR_DATA>`: barometric altitude and true airspeed.
- `Magnetic Data <https://developer.sbg-systems.com/sbgECom/5.3/binary_messages.html#SBG_ECOM_LOG_MAG>`: calibrated magnetometer data.
