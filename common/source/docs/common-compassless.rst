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

On Plane, a steady GPS heading is used after takeoff to align the /attitude/velocity/position/heading estimation filters (EKFs) for yaw. This usually occurs a few seconds after takeoff on Planes without compass. However, this approach cannot be used for VTOL navigation, since it needs to be accurate very shortly after takeoffs with no GPS velocity or heading inputs.

GSF
===

ArduPilot 4.1 (and higher) supports a Gaussian Sum Filter (GSF) which takes IMU and GPS inputs to compute a probable heading, if a yaw source (like a compass) is unavailable, or unhealthy.

.. note:: In testing so far, the accuracy is quite acceptable, but this is still considered experimental and more experience and test flights by the community are required.

.. note:: GSF depends on good velocity reports from the GPS. Do not attempt with older generation GPS, use at least UBlox M8 or equivalent performance GPS. Also, be sure GPS view of the sky is unobstructed throughout the flight. Otherwise, poor heading estimation will result.

Setup
-----

Simply, set :ref:`COMPASS_ENABLE<COMPASS_ENABLE>` = 0. Fallback to the GSF will be automatic if normal default values are being used for EKF parameters.

For configure EKF3 to always use GSF set :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 8 (GSF).

Operation
---------

Currently, only Plane will arm if the :ref:`ARMING_CHECK<ARMING_CHECK>` for compass (or ALL) is enabled, when no compasses are enabled. Copter and QuadPlane will give an pre-arming failure. Three possible methods to arm are:

#. Disable :ref:`ARMING_CHECK<ARMING_CHECK>` for Compass (keep the others!).
#. Force arming from ground station.
#. Before arming, but after GPS lock has been obtained and EKF origin has been set and is "using GPS", pick up the vehicle and walk around in a circle a few meters in diameter. This should allow the GSF to acquire yaw alignment and the message "EKF yaw alignment complete" would be sent to the ground station.

.. warning:: If either of the first two methods are used, then the first movements of the vehicle in VTOL position controlled modes may be initially in the wrong direction, but should quickly stop once yaw alignment of the GSF has been obtained. Either fly the vehicle a bit in a non-position controller mode (like STABILIZE or QHOVER), or give the vehicle sufficient clearance for the unknown initial movement direction.

Afterwards, flight and navigation should be acceptable.

If any problems are experienced, please post logs on the https://discuss.ardupilot.org forum.

Video
-----

..  youtube:: 3xW9hj-lxNU
