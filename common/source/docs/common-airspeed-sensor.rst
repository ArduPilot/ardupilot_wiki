.. _common-airspeed-sensor:

[copywiki destination="copter,rover"]

Airspeed Sensor
===============

Copter, Rover, and Blimp can attach and enable an Airspeed Sensor. However, unlike Plane which uses the readings for navigation and control, these vehicles only log and report the readings for use by the displays in Ground Control Stations, OSDs, and telemetry. The Plane airspeed sensor :ref:`documentation<plane:airspeed>` can followed for selecting and mounting an airspeed sensor. However, be aware that it will not provide all the features discussed in Plane except for measurement and reporting.

The one exception to the above is that Sailboats can use an airspeed sensor as part of a :ref:`Wind Vane<rover:wind-vane>` (:ref:`WNDVN_TYPE<WNDVN_TYPE>` = 4) and use its readings to calculate wind speed as a part of navigation.

- Set :ref:`ARSPD_ENABLE<ARSPD_ENABLE>` = 1 to allow use of airspeed sensor and to show other airspeed parameters

The following airspeed parameters have no effect and should always be set to "0" value:

- :ref:`ARSPD_USE<ARSPD_USE>`
- :ref:`ARSPD2_USE<ARSPD2_USE>`
- :ref:`ARSPD_OPTIONS<ARSPD_OPTIONS>`
- :ref:`ARSPD_AUTOCAL<ARSPD_AUTOCAL>`
- :ref:`ARSPD2_AUTOCAL<ARSPD2_AUTOCAL>`
- :ref:`ARSPD_WIND_MAX<ARSPD_WIND_MAX>`
- :ref:`ARSPD_WIND_WARN<ARSPD_WIND_WARN>`

Otherwise, the following parameters should be set if an airspeed sensor(s) are used:

- :ref:`ARSPD_TYPE<ARSPD_TYPE>`
- :ref:`ARSPD2_TYPE<ARSPD2_TYPE>`  if two sensors are used
- :ref:`ARSPD_PIN<ARSPD_PIN>` if an analog sensor is used
- :ref:`ARSPD2_PIN<ARSPD2_PIN>` if a second analog sensor is used
- :ref:`ARSPD_PRIMARY<ARSPD_PRIMARY>` if two sensors are used
- :ref:`ARSPD_BUS<ARSPD_BUS>` if an I2C sensor is used
- :ref:`ARSPD_RATIO<ARSPD_RATIO>` for the sensor is used, but usually will be automatically set by type selection
- :ref:`ARSPD2_RATIO<ARSPD2_RATIO>` if a second sensor is used, but usually will be automatically set by type selection