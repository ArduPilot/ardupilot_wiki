.. _airspeed:

========================
Using an Airspeed Sensor
========================

Plane supports the use of an airspeed sensor, which can help in windy
conditions, slow flight and autonomous landings. It is not recommended
for most new users, however, as it does require additional tuning and
adds one more layer of control to set up.

The following sections explain how to wire sensors to the autopilot and set them up. After you install an airspeed sensor don't forget to
:ref:`calibrate it <calibrating-an-airspeed-sensor>`!

.. image:: ../images/BR-0004-03-2T1.jpg
    :target: ../_images/BR-0004-03-2T1.jpg


ARSPD_USE
=========

:ref:`ARSPD_USE<ARSPD_USE>` enables airspeed use for automatic throttle modes instead of :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` as the target throttle setting (altered by :ref:`tecs-total-energy-control-system-for-speed-height-tuning-guide` as needed during altitude control) . The autopilot continues to display and log airspeed if set to 0, but only airspeed sensor readings are used for control if set to 1. It will only use airspeed sensor readings for control when throttle is idle, if set to 2 (useful for gliders with airspeed sensors behind propellers).

If an airspeed sensor is used, the throttle stick will set the target airspeed in CRUISE and FBWB, while maintaining altitude target. In AUTO and GUIDED, it will use the :ref:`AIRSPEED_CRUISE<AIRSPEED_CRUISE>` value unless :ref:`THROTTLE_NUDGE<THROTTLE_NUDGE>` is enabled and throttle stick is used to alter it, or a MAVLink command to change speed is sent to the vehicle.

Airspeed Sensor Type
====================

Airspeed sensors can be either analog or digital. The analog sensors connect to an A/D converter input pin on the autopilot, while digital sensors connect to the autopilot's external I2C bus using the SDA and SCL external digital I/O pins or via DroneCAN. The type is set by the :ref:`ARSPD_TYPE<ARSPD_TYPE>` parameter. Analog sensors are type 2, DroneCAN sensors as type 8, and supported I2C bus digital sensors by other numbers.

If there is no sensor, be sure to set the :ref:`ARSPD_TYPE<ARSPD_TYPE>` to 0. ArduPilot calculates an sensor-less airspeed estimate that is used if no sensor is present or fails. :ref:`ARSPD_TYPE<ARSPD_TYPE>` must be set to zero in order to display this value if no sensor is present.

.. warning:: Many airspeed sensors are sensitive to light. Unless you are certain that the particular sensor used is not light sensitive, in order to avoid measurement errors, the sensor should be shielded from light.

Autopilot Airspeed Connection
=============================

A list of digital and DroneCAN airspeed sensors are listed :ref:`below<arspd-sensor-list>`.


I2C
---
Connect the airspeed sensor to autopilots's I2C port (or I2C splitter
module). The :ref:`ARSPD_BUS<ARSPD_BUS>` parameter must be set for the bus used to connect the sensor. Normally this defaults to "1" , and corresponds to the I2C bus normally designated for connecting the sensor. But if it is attached to another I2C bus (eg. compass, or on some autopilots its been mistakenly assigned) it will need to be changed to "0". If the sensor fails to initialize (a GCS message will be sent if this is the case), then try changing the bus number and rebooting.

.. image:: ../images/airspeed_full_assembly_800px.jpg
    :target: ../_images/airspeed_full_assembly_800px.jpg

To enable the digital airspeed sensor, connect the autopilot to Mission
Planner (or APM Planner for OS X), and select the Optional Hardware/Airspeed
tab under the CONFIG menu. Using the drop-down box for Type, select your sensor's type. The Pin dropdown is not used and can be ignored. Check the "Use Airpeed" box to use it in control, or leave it unchecked during in-flight calibration discussed below to check its operation before use.

DroneCAN
--------

Attach the sensor to the AutoPilot's DroneCAN port and select DroneCAN in the above mentioned Type dropdown box and check the "Use Airspeed" box as appropriate. 

Analog Airspeed sensor
----------------------

Analog sensors are now largely discontinued. Digital sensors are much more accurate and consistent over temperature.
ArduPilot still supports these analog sensors, but they are not preferred. See :ref:`Analog Airspeed Sensors<analog-airspeed-sensors>`.


Installing the Pitot Tubes
==========================

When you place the airspeed sensor in your aircraft, use the pitot tube
set in the kit (the kit comes with a single tube to measure both static
and total pressure). In the case of the *EasyStar*, you'll need to push
it through the foam in the cockpit so it points straight into the
airstream (drill or cut a small hole in the foam first).
Make sure the holes in the side of the tube are not covered.
They should be at least 1 centimeter out past the nose. First connect
the two tubes coming out the back to the airspeed sensor. The tube
coming straight out the back should go into the top port and the tube
exiting at an angle should connect to the bottom port on the airspeed
sensor.

.. image:: ../images/pitotinstalled1.jpg
    :target: ../_images/pitotinstalled1.jpg

If you are using Plane in an aircraft with the propeller in the nose,
the pitot tube must be mounted out on one wing, at least a foot from the
fuselage to be outside the prop flow.

See :ref:`common-pitot-considerations` for more infomation on pitot tubes and placement considerations.

Checking operation
==================

You can check the airspeed reading with Mission Planner or another
ground station. Just blow on the pitot tube or press your finger over it and observe the response. In
still air oscillation between zero and small values (2-3) is normal. The
airspeed varies with the square root of the pressure, so for
differential pressures near zero it varies quite a bit with very small
pressure changes, while at flying speeds it takes much greater pressure
changes to produce a similar change in speed. If you see mostly 0, 1, 2,
with an occasional bounce to 3 or 4, consider it normal. You will not
see that sort of variability at flying speeds.

Calibration
===========

The airspeed sensor reading is automatically zeroed by the autopilot during
initialization, so it is good practice during windy conditions to place
a loose fitting cover over the pitot tube that shields the front hole
and the four small side holes from the wind. This cover should be fitted
prior to power on and removed before flight. If you forget to do this,
you can always place the cover and repeat the airspeed auto-zero using
the Mission Planner's PREFLIGHT CALIBRATE => Do Action.

.. note:: the ``DLVR`` type airspeed sensors do not require calibration at initialization and allow :ref:`ARSPD_SKIP_CAL<ARSPD_SKIP_CAL>` to be set to "1", avoiding the need to cover the pitot during initialization.

The airspeed reading scale factor is adjusted using the :ref:`ARSPD_RATIO<ARSPD_RATIO>`
parameter. Plane has an automatic calibration function that will adjust
the value of :ref:`ARSPD_RATIO<ARSPD_RATIO>` automatically provided the plane is flown with
frequent direction changes. A normal model flying field circuit pattern
or loiter will achieve the required direction changes, cross-country
flying will not. To enable automatic airspeed sensor calibration, set
the value of :ref:`ARSPD_AUTOCAL<ARSPD_AUTOCAL>` to 1. See :ref:`calibrating-an-airspeed-sensor` for more details.

Miscalibration Safeguards
=========================

In order to help prevent Airspeed sensor use when its been miscalibrated either during ground static calibration during the power up sequence, or by accidental parameter changes to offset or ratio, three parameters are available. If the ground speed is consistently lower than the reported airspeed for a few seconds by :ref:`ARSPD_WIND_MAX<ARSPD_WIND_MAX>`, i.e. the apparent wind speed is greater than that amount, the sensor can be disabled to avoid erroneous reporting. It can be allowed to re-enable if the apparent wind falls back below that value. These actions are controlled by :ref:`ARSPD_OPTIONS<ARSPD_OPTIONS>`.

You can also send a warning to the Ground Control Station if the apparent wind exceeds :ref:`ARSPD_WIND_WARN<ARSPD_WIND_WARN>`. This can be used instead of, or together with the above

Failure
=======

A failing airspeed sensor can lead to the aircraft stalling or over-speeding, this is something that is hard for ArduPilot to detect. Likewise, accidentally miscalibrating the offset during ground initialization can occur if the pitot tube is not covered to prevent wind upsetting the calibration, and can result in wildly inaccurate readings. The parameters below can be used to help detect these conditions and warn of, and/or disable, a failed sensor.

:ref:`ARSPD_WIND_MAX<ARSPD_WIND_MAX>` can be used to set the maximum expected wind speed the vehicle should ever see. This is then be used
in combination with the GPS ground speed to detect a airspeed sensor error. :ref:`ARSPD_WIND_WARN<ARSPD_WIND_WARN>` can be set to a lower speed to give 
some warning to the operator before the airspeed sensor is disabled. :ref:`ARSPD_OPTIONS<ARSPD_OPTIONS>` can be set to allow sensors to be disabled
based on this wind speed metric, a second option bit allows then to be re-enabled if the speed error is resolved.

:ref:`AHRS_WIND_MAX<AHRS_WIND_MAX>` sets the maximum allowable airspeed and ground speed difference that will ever be used for navigation.

By default, these functions are disabled.

:ref:`EKF3 affinity and lane switching <common-ek3-affinity-lane-switching>` is another option for dealing with airspeed sensor failure.

.. _arspd-sensor-list:

Airspeed sensors available from ArduPilot Partners:
===================================================

I2C
---

- 4525DO 
    - `CUAV <https://store.cuav.net/shop/airspeed-sensor/>`_
    - `Holybro <https://shop.holybro.com/digital-air-speed-sensor_p1029.html>`_
    - `Matek 4525DO <http://www.mateksys.com/?portfolio=aspd-4525>`_
    - `mRobotics <https://store.mrobotics.io/mRo-I2C-Airspeed-Sensor-JST-GH-p/m10030a.htm>`_

- ASP5033
    - `Qiotek ASP5033 <https://www.qio-tek.com/index.php/product/qiotek-asp5033-airspeed-sensor-and-pitot-tube>`_

- DLVR
    - `Matek DLVR <http://www.mateksys.com/?portfolio=aspd-dlvr>`_


DroneCAN
--------

- DLVR
    - `Foxtech AEROFOX Airspeed/Compass <https://www.foxtechfpv.com/foxtech-aerofox-can-airspeed-compass.html>`__
    - `Matek DroneCAN DLVR <http://www.mateksys.com/?portfolio=aspd-dlvr>`_

- ASP5033 
    - `Qiotek DroneCAN 5033 <https://www.qio-tek.com/index.php/product/qiotek-asp5033-dronecan-airspeed-and-compass-module>`_

- 6897
    - `Foxtech AEROFOX Airspeed/Compass <https://www.foxtechfpv.com/foxtech-aerofox-can-airspeed-compass.html>`__

Other Topics
============
.. toctree::
    :maxdepth: 1

    Calibrating an Airspeed Sensor <calibrating-an-airspeed-sensor>
    Pitot Tube Considerations <common-pitot-considerations>
    Mocking an Airspeed Sensor for Bench Testing <mocking-an-airspeed-sensor-for-bench-testing>
    Analog Airspeed Sensors <analog-airspeed-sensors>
