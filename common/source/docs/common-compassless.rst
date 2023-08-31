.. _common-compassless:

======================
Compass-less Operation
======================

Operation without a compass has been possible for Plane for a long time. However, recent additions of alternative yaw sources has opened the possibility of compass-less operation of multicopters, quadplanes and rovers.  The available mechanism are:

- Align yaw to GPS after takeoff (Plane only)
- GSF (Gaussian Sum Filter) estimates the heading by comparing a single GPS's outputs to the IMU movements
- :ref:`GPS-for-Yaw <common-gps-for-yaw>` uses the position of two UBlox F9 GPSs to calculate yaw
- :ref:`Intel RealSense T265 <common-vio-tracking-camera>` can provide yaw relative to its startup yaw

Align Yaw to GPS (Plane only)
=============================

On Plane, a steady GPS heading is used after takeoff to align the /attitude/velocity/position/heading estimation filters (EKFs) for yaw. This usually occurs a few seconds after takeoff on Planes without compass. However, this approach cannot be used for Quadplane's VTOL navigation, since it needs to be accurate very shortly after takeoffs with no GPS velocity or heading inputs.

.. note:: in 4.3 firmware, GSF is used instead, see below.

GSF
===

ArduPilot 4.1 (and higher) supports a Gaussian Sum Filter (GSF) which takes IMU and GPS inputs to compute a probable heading, if a yaw source (like a compass) is unavailable, or unhealthy.

.. warning:: Currently, it does not function well for QuadPlane VTOL tailsitters, since their nose is pointed vertically in VTOL flight. Compasses are still recommended for Tailsitters. Also, for this to work well at all, vibration should be low and the GPS velocity accurate, see below.

.. note:: GSF depends on good velocity reports from the GPS. Do not attempt with older generation GPSes, and use at least UBlox M8 or equivalent performance GPSes. Also, be sure GPS view of the sky is unobstructed throughout the flight. Otherwise, poor heading estimation will result. On Plane, if the GPS accuracy is impaired by interference from other nearby vehicle devices (like cameras, etc.), GSF can result in poorer performance than relying solely on Align Yaw to GPS heading, above. In those cases, it may be better to disable GSF use by setting the :ref:`EK3_GSF_USE_MASK<EK3_GSF_USE_MASK>` = 0.

Setup
-----

Simply, set :ref:`COMPASS_ENABLE<COMPASS_ENABLE>` = 0. Fallback to the GSF will be automatic if normal default values are being used for EKF parameters.

For configure EKF3 to always use GSF set :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 8 (GSF).

Operation
---------

Currently, only Plane will arm if the :ref:`ARMING_CHECK<ARMING_CHECK>` for compass (or ALL) is enabled, when no compasses are enabled. Copter and QuadPlane will give an pre-arming failure. Three possible methods to arm Copter and QuadPlane are:

#. Disable :ref:`ARMING_CHECK<ARMING_CHECK>` for Compass (keep the others!).
#. Force arming from ground station.
#. Before arming, but after GPS lock has been obtained and EKF origin has been set and is "using GPS", pick up the vehicle and walk around in a circle a few meters in diameter. This should allow the GSF to acquire yaw alignment and the message "EKF yaw alignment complete" would be sent to the ground station.

.. warning:: If either of the first two methods are used, then the first movements of the vehicle in VTOL position controlled modes may be initially in the wrong direction, but should quickly stop once yaw alignment of the GSF has been obtained. Either fly the vehicle a bit in a non-position controller mode (like STABILIZE or QHOVER), or give the vehicle sufficient clearance for the unknown initial movement direction.

Afterwards, flight and navigation should be acceptable.

.. note:: on Plane, without compass, after arming the vehicle will fall back to DCM ("DCM Active"  message over MAVLinK) from EKF3 since it has no yaw sensor yet. This will be flagged in MP HUD as "Unhealthy AHRS" or on RC telemetry systems (like CRSF) with a "AHRS Bad" message. Ignore this, once the plane starts moving EKF3 will obtain yaw information and align, and operation will switch back to it ("EKF3 active").

If any problems are experienced, please post logs on the https://discuss.ardupilot.org forum.

Video
-----

..  youtube:: 3xW9hj-lxNU
