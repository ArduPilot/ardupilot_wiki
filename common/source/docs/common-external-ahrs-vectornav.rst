.. _common-external-ahrs-vectornav:

=======================
VectorNav External AHRS
=======================

VectorNav Technologies designs and develops high-performance, low-SWaP `IMU/AHRS <https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-ahrs>`__, `GNSS/INS <https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-gpsins>`__, and `Dual GNSS/INS <https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-gnsscompass>`__ solutions that enable safe and reliable autonomy at scale.

VectorNav products provide a range of benefits to ArduPilot users and can be integrated for:

- Higher accuracy heading, pitch, and roll estimates
- More robust and reliable GNSS positioning
- Improved positioning and attitude performance in GNSS-contested environments
- Performance under challenging dynamic conditions (e.g. catapult launches, VTOL operations, high-g or high angular rate operations)

The VectorNav ArduPilot Driver is streamlined to provide a simple plug-and-play architecture, removing engineering obstacles and allowing the acceleration of the design, development, and launch of platforms to keep pace with the rapid rate of innovation.

ArduPilot can use these sensors as an :ref:`external AHRS <common-external-ahrs>`, bypassing/replacing the EKF3 estimator, or as a source of raw sensor data provided to the estimator.

The driver supports `all VectorNav sensors <https://www.vectornav.com/store/products>`__. In particular the following systems are recommended:

- `VN-200 <https://www.vectornav.com/products/detail/vn-200>`__ or `VN-210 <https://www.vectornav.com/products/detail/vn-210>`__ `GNSS/INS <https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-gpsins>`__: Recommended for fixed-wing systems without hovering, where static heading is not necessary.
- `VN-300 <https://www.vectornav.com/products/detail/vn-300>`__ or `VN-310 <https://www.vectornav.com/products/detail/vn-310>`__ `Dual GNSS/INS <https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-gnsscompass>`__: Recommended for multicopter systems where hovering and low dynamics requires the use of static heading.

For performance specifications and basic product information, refer to the respective model's `Product Brief <https://www.vectornav.com/resources/product-information/product-briefs>`__. Industrial-series units can be purchased on the `VectorNav website <https://www.vectornav.com/store/products>`__; for tactical-series purchases, please contact sales@vectornav.com. For help integrating VectorNav products with ArduPilot, please contact support@vectornav.com.

Hardware Setup
==============

Wiring
------
Connect any unused flight controller serial interface, such as a spare **GPS** or **TELEM** port, to either available VectorNav **UART** port.

.. warning::
  VectorNav industrial-series (VN-X00) rugged units (non-SMD) use RS-232 voltage levels on **UART1** and TTL voltage levels on **UART2**. As such, if using one of those sensors it may be necessary to use VectorNav **UART2**.

For any model-specific hardware requirements, see the relevant model's `Datasheet <https://www.vectornav.com/resources/technical-documentation/datasheets>`__.

Mounting
--------
The VectorNav sensor can be mounted in any orientation, in any position on the vehicle, without regard to center of gravity. All VectorNav sensors default to a coordinate system of x-forward, y-right, and z-down, making the default mounting as connector-back, base-down. This can be changed to any rigid rotation using the VectorNav Reference Frame Rotation register.

If using a GNSS-enabled product, the GNSS antenna must be mounted rigidly with respect to the inertial sensor and with an unobstructed sky view. If using a dual-GNSS-enabled product (VN-3X0), the secondary antenna must be mounted rigidly with respect to the primary antenna and the inertial sensor with an unobstructed sky view.

For more mounting requirements and recommendations, see the relevant model's `Quick Start Guide <https://www.vectornav.com/resources/technical-documentation/quick-start-guides>`__ and `User Manual <https://www.vectornav.com/resources/technical-documentation/user-manuals>`__.

Sensor Configuration
====================

Upon initialization, ArduPilot configures the VectorNav unit as follows:

- Configures necessary binary outputs
- Disables ASCII outputs on both serial ports

All other necessary configuration parameters must be separately loaded to the VectorNav unit manually. 

- ``Baudrate``: Necessary if increasing the :ref:`EAHRS_RATE<EAHRS_RATE>` above a certain rate. The default baudrate of VectorNav sensors is 115200 bps, but is recommended to increase the baudrate on the sensor to 921600 bps and set :ref:`EAHRS_RATE<EAHRS_RATE>` to 800Hz, if possible.
- ``Reference Frame Rotation``: Necessary if not mounted as connector-back, base-down configuration.
- ``GNSS Antenna A Offset``: Necessary if using a GNSS-enabled product and the GNSS antenna is mounted more than 10 centimeters from the VectorNav unit.
- ``GNSS Antenna Baseline``: Necessary if using a dual-GNSS-enabled product. 

After setting these parameters, the settings must be set to persist over a power cycle using a **Write Settings** command.

For VectorNav register and command definitions, see the relevant model's `Interface Control Document <https://www.vectornav.com/resources/technical-documentation/interface-control-documents>`__.

Configured Binary Outputs
-------------------------

Upon initialization, ArduPilot configures the VectorNav to output the following binary outputs:

Binary Output 1 (IMU, all models)

- Configured according to EAHRS_RATE (default 50Hz)
- ``$VNWRG,75,3,16,01,0721*D415``
- Common group (Group 1)

  - TimeStartup
  - AngularRate
  - Accel
  - Imu
  - MagPres
 

Binary Output 2 (EKF, if using VN-1X0)

- Configured to 50Hz
- ``$VNWRG,76,3,16,11,0001,0106*B36B``
- Common group (Group 1)

  - TimeStartup

- Attitude group (Group 4)

  - Ypr
  - Quaternion
  - YprU


Binary Output 2 (EKF, if using VN-2X0 or VN-3X0)

- Configured to 50Hz
- ``$VNWRG,76,3,16,31,0001,0106,0613*097A``
- Common group (Group 1)

  - TimeStartup

- Attitude group (Group 4)

  - Ypr
  - Quaternion
  - YprU

- Ins group (Group 5)

  - InsStatus
  - PosLla
  - VelNed
  - PosU
  - VelU


Binary Output 3 (GNSS, if using VN-2X0 or VN-3X0)

- Configured to 5Hz
- ``$VNWRG,77,1,160,49,0003,26B8,0018*4FD9``
- Common group (Group 1)

  - TimeStartup
  - TimeGps

- Gnss1 group (Group 3)

  - NumSats
  - GnssFix
  - GnssPosLla
  - GnssVelNed
  - PosU1
  - VelU1
  - GnssDop

- Gnss2 group (Group 6)

  - NumSats
  - GnssFix

.. note::
  The ``RateDivisor`` field sent to the sensor varies for Binary Output 1 according to :ref:`EAHRS_RATE<EAHRS_RATE>`, and is halved for the ``VN-300``. 


ArduPilot Configuration
=======================
There are two possible ways for VectorNav data to be used by ArduPilot: as an external sensor set to ArduPilot's EKFs or as an external AHRS. Both ways utilize ArduPilot's External AHRS driver.

To establish communication with the VectorNav unit, set the following for the relevant serial port:

  - ``SERIALx_PROTOCOL`` = 36 (AHRS)
  - ``SERIALx_BAUD`` = matching VectorNav sensor baudrate

.. tip::
  The External AHRS-specific parameters may not be visible before the ``Serialx_Protocol`` parameter is configured. As such, either a Refresh Params or a reset of ArduPilot may be necessary to see the parameters.

Use as an External Sensor Set
-----------------------------
If set up as an external sensor, VectorNav's raw sensor data (IMU, GNSS, Compass, Barometer, and GNSS, if available) can be used by ArduPilot's internal EKFs, as configured. After the serial parameters have been configured, configure:

- :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 3 (ArduPilot’s EKF3)
- :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNav)
- :ref:`EAHRS_OPTIONS<EAHRS_OPTIONS>` bit 0 set to "1" value to disable ArduPilot's use of the bias-compensated IMU data, letting ArduPilot's filters do that (optional)
- :ref:`GPS1_TYPE<GPS1_TYPE>` = 21 (External AHRS) (If using a GNSS-enabled unit)
- :ref:`GPS2_TYPE<GPS2_TYPE>` = 21 (External AHRS) (If using a Dual GNSS-enabled unit)

If desired, :ref:`EAHRS_SENSORS<EAHRS_SENSORS>` may be used to specify which sensor data should be used by ArduPilot's filters.

Because ArduPilot's internal EKF will only update at the input IMU rate, it is recommended to raise the VectorNav IMU output rate beyond the default 50Hz. To do so, set :ref:`EAHRS_RATE<EAHRS_RATE>` to the desired IMU rate (800Hz maximum, 400Hz maximum for VN-300). Because the IMU output rate is configured on initialization, an ArduPilot reset is required after changing :ref:`EAHRS_RATE<EAHRS_RATE>`.

Use as an External AHRS
-----------------------
Configuring ArduPilot to use the VectorNav sensor as an External AHRS will use the VectorNav PVTA (position, velocity, time, and attitude) solution as canonical rather than one of the possible internal ArduPilot filters.
This will allow ArduPilot to use the VectorNav sensor's INS data that combines IMU and GNSS data in an advanced Kalman filtering estimation to provide position, velocity, and attitude estimates of higher accuracies and with better dynamic performance.

.. note::
  VectorNav uses the term AHRS to refer to an attitude-only solution, without absolute position measurement input. VectorNav uses the term INS to refer to a solution which accepts a position (often GNSS) measurement input and outputs a full PVTA. Because ArduPilot's External AHRS driver requires the data source (VectorNav) to provide an absolute PVT, use as an External AHRS is restricted to a VectorNav INS-enabled product (VN-2X0 or VN-3X0).

After the serial parameters have been configured, configure:
  - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)
  - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV)

.. tip::
  ArduPilot's internal navigation filters run even when configured to use a VectorNav as the canonical navigation source (unless internal filters are disabled). As such, it is recommended to additionally configure the VectorNav as an external sensor set. This allows ease of switching canonical PVTA between VectorNav's and ArduPilot's navigation filters, if necessary.
  To do this, configure the necessary paramters in Use as an External Sensor Set, but leave `AHRS_EKF_TYPE<AHRS_EKF_TYPE>` as External AHRS.

Published Data
==============

ArduPilot is configured to save VectorNav sensor data to a DataFlash Log as up to three messages: EAHI, EAHA, and EAHK.

The EAHI (External AHRS IMU) message contains IMU data outputs:

- Time (microseconds)
- Temperature (deg C)
- Pressure (Pa)
- Magnetometer (Gauss)
- Accelerometer (m/s^2)
- Gyroscope (rad/s)

The EAHA (External AHRS Attitude) message contains the following data outputs:

- Time (microseconds)
- Quaternion
- Yaw, pitch, roll (deg)
- Yaw, pitch, roll uncertainty (deg)

The EAHK (External AHRS INS/EKF) message contains INS data outputs:

- Time (microseconds)
- InsStatus
- Position LLA
- Velocity NED
- Position Uncertainty
- Velocity Uncertainty
