.. _common-non-gps-to-gps:

=========================
GPS / Non-GPS Transitions
=========================

ArduPilot 4.1 (and higher) support in-flight transitions between GPS and Non-GPS environments.

Setup
-----

To enable transitions the EKF3 must be used.  Instructions for enabling EKF3 can be found on the :ref:`Extended Kalman Filter page <common-apm-navigation-extended-kalman-filter-overview>` but in short it requires setting the following parameters:

- :ref:`EK3_ENABLE <EK3_ENABLE>` = 1
- :ref:`EK2_ENABLE <EK2_ENABLE>` = 0
- :ref:`AHRS_EKF_TYPE <AHRS_EKF_TYPE>` = 3

Up to three "sets" of sensor sources are supported.  Normally the primary set should be configured for the GPS environment.  Below is the default setup:

- :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` = 3 (Primary horizontal position from GPS)
- :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` = 3 (Primary horizontal velocity from GPS)
- :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` = 1 (Primary vertical position from barometer)
- :ref:`EK3_SRC1_VELZ <EK3_SRC1_VELZ>` = 3 (Primary vertical velocity from GPS)
- :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 1 (Primary yaw/heading from compass)

The secondary sensor source set should be configured for the non-GPS environment.  Please refer to the setup instructions for the :ref:`non-GPS sensor <common-non-gps-navigation-landing-page>` you plan to use.  Below is the setup if using the :ref:`Intel Realsense T265 <common-vio-tracking-camera>`:

- :ref:`EK3_SRC2_POSXY <EK3_SRC2_POSXY>` = 6 (Secondary horizontal position from External Nav)
- :ref:`EK3_SRC2_VELXY <EK3_SRC2_VELXY>` = 6 (Secondary horizontal velocity from External Nav)
- :ref:`EK3_SRC2_POSZ <EK3_SRC2_POSZ>` = 1 (Secondary vertical position from barometer)
- :ref:`EK3_SRC2_VELZ <EK3_SRC2_VELZ>` = 6 (Secondary vertical velocity from External Nav)
- :ref:`EK3_SRC2_YAW <EK3_SRC2_YAW>` = 6 (Secondary yaw/heading from External Nav)

The fusing of all velocities should be disabled by unchecking the :ref:`EK3_SRC_OPTIONS <EK3_SRC_OPTIONS>` parameter's "FuseAllVelocities" bit:

- :ref:`EK3_SRC_OPTIONS <EK3_SRC_OPTIONS>` = 0

The pilot can manually switch between the source sets using a 3-position :ref:`auxiliary switch <common-auxiliary-functions>` configured for "EKF Pos Source" (90).  When the switch is pulled low, the primary source set will be used.  Middle position is for the secondary source set and High is for the tiertiary source set.  If the transmitter's channel 8 switch is to be used set:

- :ref:`RC8_OPTION <RC8_OPTION>` = 90 (EKF Pos Source)

:ref:`Lua scripts <common-lua-scripts>` can be used to automatically switch between sources based on information from the sensors (i.e. GPS speed accuracy, rangefinder distance, etc) or "innovations" from the EKF ("innovations" are a measure of how close the sensor's latest sensor data matches the EKF's estimate).  As of Nov 2020, there is only one supported script, `ahrs-source.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/ahrs-source.lua>`__ which allows automatically switching between GPS, T265 and optical flow.

Testing
-------

First perform a bench test, using the auxiliary switch to manually switch between sources.

- Connect with a ground station and confirm that after switching sources, text messages appear indicating that the EKF has changed sources.  If using Mission Planner these messages will appear in on the Data screen's Messages tab
- After changing sources wait 10 seconds to confirm the EKF remains healthy. If using Mission Planner the EKF label on the HUD should remain white
- Check the vehicle's horizontal position, altitude and heading using the ground station.  If using GPS with the :ref:`Intel Realsense T265 <common-vio-tracking-camera>` you should notice a jump when switching from the T265 to GPS but not when switching from GPS to T265.  This is because the "ExternalNav" position is always updated to match the GPS when the GPS is the primary source set

Next perform a flight test in a safe environment, manually switching between sources.  Be prepared to re-take control in a manual mode such as Stabilize.

If you plan to automatically switch sources, first walk the vehicle between the GPS and Non-GPS environments to confirm it is switching source sets.  Finally perform a flight test in which the vehicle is flown (or driven) at low speeds between the two environments.  As with previous tests be ready to re-take control in a manual mode such as Stabilize.

Future Improvements
-------------------

A list of planned improvements to this feature are here on the `Issues List <https://github.com/ArduPilot/ardupilot/issues/15859>`__
