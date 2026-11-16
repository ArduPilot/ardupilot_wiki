.. _common-rangefinder-dts6012m:

========================
DTS6012M Lidar (Serial)
========================

.. note::
    Support for this sensor is available in firmware versions 4.7 and later, on autopilots with more than 1MB of flash.

The DTS6012M is a very small (1.35g) laser rangefinder with a range of up to 20m. ArduPilot supports it over UART. See the `spec sheet <https://www.polarisic.com/uploads/soft/20241212/1734000227.pdf>`__ for details and pinout.

Connecting to the Autopilot
===========================

Pin 5 of the sensor must be pulled low to put it into UART mode.

Connect the sensor's TX and RX to any spare autopilot Serial/UART port. The example parameters below assume SERIAL4 is used.

Set the following parameters:

-  :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` = 9 (Rangefinder)
-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 47 (DTS6012M-Serial). Reboot after setting this.
-  :ref:`RNGFND1_MIN <RNGFND1_MIN>` = 0.2
-  :ref:`RNGFND1_MAX <RNGFND1_MAX>` = 20
-  :ref:`RNGFND1_ORIENT <RNGFND1_ORIENT>` = 25 (Down) if the sensor faces downward

The driver sets the port's baud rate to 921600 itself, so :ref:`SERIAL4_BAUD<SERIAL4_BAUD>` does not need to be changed. The driver also reports a signal quality, derived from the strength of the return.

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "rangefinder1".
