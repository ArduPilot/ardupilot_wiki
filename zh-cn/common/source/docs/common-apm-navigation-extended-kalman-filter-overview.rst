.. _common-apm-navigation-extended-kalman-filter-overview:

======================================
ArduPilot Extended Kalman Filter (EKF)
======================================

Copter and Plane can use an Extended Kalman Filter (EKF) algorithm to
estimate vehicle position, velocity and angular orientation based on
rate gyroscopes, accelerometer, compass, GPS, airspeed and barometric
pressure measurements.

The advantage of the EKF over the simpler complementary filter
algorithms, is that by fusing all available measurements it is better
able to reject measurements with significant errors. This makes the
vehicle less susceptible to faults that affect a single sensor. EKF also
enables measurements from optional sensors such as optical flow and
laser range finders to be used to assist navigation.

Commonly modified parameters:
=============================

:ref:`AHRS_EKF_USE <dev:extended-kalman-filter_ahrs_ekf_use>`
: set to "1" to enable the EKF, "0" to disable and rely on DCM for
attitude control, inertial nav (Copter-3.2.1) or ahrs dead reckoning
(Plane) for position control.  In Copter-3.3, this feature is forced to
"1" and cannot be changed.

:ref:`EKF_ALT_NOISE <dev:extended-kalman-filter_ekf_alt_noise>`:
Default is "1.0".  Lower number reduces reliance on accelerometers,
increases reliance on barometer.

:ref:`EKF_GPS_TYPE <dev:extended-kalman-filter_ekf_gps_type>`:
Controls how GPS is used.

-  0 : use 3D velocity & 2D position from GPS
-  1 : use 2D velocity & 2D position (GPS velocity does not contribute
   to altitude estimate)
-  2: use 2D position
-  3 : no GPS (will use :ref:`optical flow <common-px4flow-overview>` only
   if available)

A more detailed overview of EKF benefits, theory and *tuning parameters*
is available on the developer wiki topic :ref:`Extended Kalman Filter Navigation Overview and Tuning <dev:extended-kalman-filter>`.
