.. _common-gps-navisys-gm-m04b:

=======================================
Navisys GM-M04B L1/L5 GNSS with Compass
=======================================

.. image:: ../../../images/navisys/navisys_gm-m04b.png
    :target: ../_images/navisys_gm-m04b.png
    :width: 300px

The Navisys GM-M04B is a compact L1/L5 dual-band GNSS smart antenna module based on the Airoha AG3335 chipset, with an integrated Bosch BMM350 compass. It outputs standard NMEA sentences over UART and is used in ArduPilot with the NMEA GPS driver.

Specifications
==============

-  **Sensors**

   - Airoha AG3335 GNSS (12nm)

     - 135 channels, concurrent L1/L5 dual-band
     - GPS, GLONASS, Galileo, BeiDou, QZSS, NavIC, SBAS
     - Horizontal accuracy 1.5 m CEP (L1+L5)
     - Tracking sensitivity -165 dBm
     - Update rate up to 10 Hz (default 5 Hz)
     - Jamming detection and 12-tone active interference canceller on L1/L5

   - Bosch BMM350 3-axis compass (±2000 μT, I2C)

-  **Interface**

   - 6-pin, 1.0 mm pitch connector (UART TX, ON/OFF control, I2C)
   - UART TTL output, NMEA 0183 (V3.01 / V4.10), 230400 bps by default

-  **Power**

   - 3.3 ~ 5 V, 56 mA (average tracking)
   - Built-in backup battery

-  **Other**

   - Built-in L1/L5 patch antenna
   - Fix status LED (blinking: fixed, on: no fix)
   - Fully EMI shielded
   - Operating temperature: -40 ~ 85 °C (battery: -20 ~ 60 °C)
   - Dimensions: 30 x 30 x 11.8 mm
   - Weight: 19.2 g

Pinout
======

.. image:: ../../../images/navisys/navisys_gm-m04b_pinout.png
    :target: ../_images/navisys_gm-m04b_pinout.png
    :width: 335px

===  =======  ==============================  ======
Pin  Name     Function                        Type
===  =======  ==============================  ======
1    GND      Ground                          Power
2    VCC      Power supply (3.3 ~ 5 V)        Power
3    TXD      UART transmit (TTL)             Output
4    ON/OFF   High/floating: on, low: off     Input
5    SCL      I2C clock (compass)             Input
6    SDA      I2C data (compass)              I/O
===  =======  ==============================  ======

An optional 30 cm cable (6-pin JST-GH to the module connector) is available for direct connection to the 6-pin GPS2 port of Pixhawk-standard autopilots.

.. note::
   Pin 4 is a power ON/OFF control input, not a UART receive line. The module does not accept commands or RTCM corrections from the autopilot. If pin 4 is connected to the autopilot's UART TX line, any data sent on that line pulls the pin low, which may switch the module off. The recommended wiring is to leave pin 4 unconnected.

   If pin 4 must stay connected (for example when using the standard cable), the autopilot can still transmit on this port in two cases:

   * During GPS detection, ArduPilot sends configuration data. Set :ref:`GPS_AUTO_CONFIG <GPS_AUTO_CONFIG>` = 0 to stop this.
   * RTCM correction data received from the ground station is sent to every GPS by default. If another (RTK) GPS is used, set :ref:`GPS_INJECT_TO <GPS_INJECT_TO>` to that GPS instance so that corrections are not sent to the GM-M04B.

ArduPilot Configuration
=======================

The GM-M04B has been tested on the GPS1 (SERIAL3) and GPS2 (SERIAL4) ports of a Pixhawk 6C. Any free serial port with an I2C bus can be used, as long as the parameters are changed to match.

ArduPilot assigns GPS instances to the serial ports with ``SERIALx_PROTOCOL`` = 5 (GPS) in order, starting from the lowest-numbered port, whether or not a GPS is connected to that port. On most Pixhawk-standard autopilots, both SERIAL3 (GPS1 port) and SERIAL4 (GPS2 port) default to 5, so the first GPS instance is always on SERIAL3.

GPS
---

When connected to the GPS2 port (SERIAL4) as the only GPS, using the optional cable:

* :ref:`SERIAL3_PROTOCOL <SERIAL3_PROTOCOL>` = -1 (None), or any protocol other than 5, so that SERIAL4 becomes the first GPS port
* :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 5 (GPS)
* :ref:`GPS1_TYPE <GPS1_TYPE>` = 5 (NMEA)
* :ref:`GPS_AUTO_CONFIG <GPS_AUTO_CONFIG>` = 0 (Disabled), recommended if pin 4 is connected (see note above). This parameter applies to all GPS instances.

When connected to the GPS1 port (SERIAL3), keep :ref:`SERIAL3_PROTOCOL <SERIAL3_PROTOCOL>` = 5 and set :ref:`GPS1_TYPE <GPS1_TYPE>` = 5.

If another GPS is connected to a lower-numbered port that also has ``SERIALx_PROTOCOL`` = 5 (for example a primary GPS on the GPS1 port), the GM-M04B becomes the second GPS instance: keep ``SERIAL3_PROTOCOL`` = 5 and set :ref:`GPS2_TYPE <GPS2_TYPE>` = 5 instead.

.. note::
   The NMEA driver is not included in the automatic detection of ``GPSx_TYPE`` = 1 (AUTO), so the type must be set to 5. The baud rate is detected automatically. The module is used with its factory settings (5 Hz, GGA and RMC output).

Compass
-------

The BMM350 compass is detected automatically on the external I2C bus (ArduPilot 4.6 and later). After connecting, perform a :ref:`compass calibration <common-compass-calibration-in-mission-planner>` with :ref:`COMPASS_AUTO_ROT <COMPASS_AUTO_ROT>` enabled so that the orientation of the compass is set automatically. In Navisys testing with the antenna facing up, the detected orientation was :ref:`COMPASS_ORIENT <COMPASS_ORIENT>` = 10 (Roll180Yaw90).

More Information
================

* `Navisys Official Website <https://www.navisys.com.tw/>`_
