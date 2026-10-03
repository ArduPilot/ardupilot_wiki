.. _common-twtof240ui-lidar:

================
TWTOF240UI Lidar
================

The TWTOF240UI is a small time-of-flight lidar. The module states a range of 0.10 m to 2.4 m on a 90% reflectivity target, a resolution of 1 cm, and a 25 degree field of view. In direct sun the outdoor range is much shorter than that indoor rating.

ArduPilot's driver uses the I2C interface only. The module also has a UART interface (9600 baud), which this driver does not use.

.. warning::

   The module supply is 3.0 V to 3.6 V, and the logic level is 3.3 V. Do not power it from a 5 V rail.

Connecting to the Autopilot
===========================

The connector is a 6-pin 1.25 mm plug. For I2C, connect these pins to an I2C port on the autopilot:

- 1 VDD
- 2 GND
- 5 SDA
- 6 SCL

Pins 3 (TX) and 4 (RX) are the UART lines. Leave them unconnected for this driver.

The module's I2C address is 0x36. These pages do not give a command to change it.

Set the following parameters:

- :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 49 (TWTOF240UI). Reboot after setting this.
- :ref:`RNGFND1_ADDR <RNGFND1_ADDR>` = 0 uses 54 decimal (0x36). This does not change the address stored in the module.
- :ref:`RNGFND1_MIN <RNGFND1_MIN>` = 0.10
- :ref:`RNGFND1_MAX <RNGFND1_MAX>` = 2.4
- :ref:`RNGFND1_GNDCLR <RNGFND1_GNDCLR>` to the distance in metres from the sensor to the ground when the vehicle is landed. This depends on how the sensor is mounted.

.. note::

   A reading above 3 m is the module's failure code, not a distance. The driver ignores it.

.. note::

   Autopilots with 1 MB of flash leave this driver out of the standard build. Include it by setting ``AP_RANGEFINDER_TWTOF240UI_ENABLED`` in the build. See :ref:`common-custom-firmware`.

Testing the sensor
==================

Distances read by the sensor can be seen on Mission Planner's Flight Data screen's Status tab. Look for "rangefinder1".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
