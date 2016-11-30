.. _common-lightware-sf10-lidar:

=============================
LightWare SF10 and SF11 Lidar
=============================

The `Lightware SF10 and SF11 <http://www.lightware.co.za/shop/en/>`__ series of laser rangefinders are particularly lightweight, and provide fast and accurate distance measurements.
Although slightly more expensive than other rangefinders, members of the dev team have had good success with them.
The series includes a number of models:
`SF10/A <http://www.lightware.co.za/shop/en/drone-altimeters/33-sf10a.html>`__ (25m),
`SF10/B <http://www.lightware.co.za/shop/en/drone-altimeters/32-sf10b.html>`__ (50m),
`SF10/C <http://www.lightware.co.za/shop/en/drone-altimeters/34-sf10c.html>`__ (100m) and 
`SF11/C <http://www.lightware.co.za/shop/en/drone-altimeters/51-sf11c-120-m.html>`__ (120m)

\ |SF10-B|

Connecting to the Pixhawk
=========================

The diagram below shows the sensor output pins and a conveniently colour-coded cable (normally included or `you can purchase here <http://www.lightware.co.za/shop/en/accessories/37-main-cable-type-1-35-cm.html>`__) which is used to connect to the flight controller. :ref:`Serial <sf10-serial-connection>`, :ref:`I2C <sf10-i2c-connection>` and :ref:`Analog <sf10-analog-connection>` connections are possible but we recommended using :ref:`Serial <sf10-serial-connection>` if possible (`see issue here <https://github.com/ArduPilot/ardupilot/issues/4803>`__).

.. tip::

   The serial connection is recommended when using longer
   cables or when using Copter (`see issue here <https://github.com/ArduPilot/ardupilot/issues/4803>`__)

.. figure:: ../../../images/RangeFinder_SF10_Output_Pins.png
   :target: http://www.lightware.co.za/shop/en/drone-altimeters/32-sf10b.html

   SF10 Rangefinder: Output Pins

.. figure:: ../../../images/SF10_Rangefinder_main-cable-type-1-35-cm.jpg
   :target: http://www.lightware.co.za/shop/en/accessories/37-main-cable-type-1-35-cm.html

   Main cable for SF10 Rangefinder

.. _sf10-serial-connection:

Serial connection
-----------------

For a serial connection you can use any spare UART. Connect the RX line
of the UART to the TX line of the Lidar, and the TX line of the UART to
the RX line of the Lidar. Also connect the GND and 5V lines. You do not
need flow control pins.

The diagram below shows how to connect to SERIAL4.

.. figure:: ../../../images/Pixhawk_Rangefinder_SF10_Serial.jpg
   :target: ../_images/Pixhawk_Rangefinder_SF10_Serial.jpg

   Pixhawk and SF10Rangefinder: Serial Connection (SERIAL4)

You then need to setup the serial port and rangefinder parameters. If
you have used the SERIAL4/5 port on the Pixhawk then you would set the
following parameters (this is done in the *Mission Planner*
**Config/Tuning \| Full Parameter List** page):

-  :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 9 (Lidar)
-  :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` = 19 (19200 baud)
-  :ref:`RNGFND_TYPE <RNGFND_TYPE>` = 8 (LightWareSerial)
-  :ref:`RNGFND_SCALING <RNGFND_SCALING>` = 1
-  :ref:`RNGFND_MIN_CM <RNGFND_MIN_CM>` = 5
-  :ref:`RNGFND_MAX_CM <RNGFND_MAX_CM>` = **2500** (for SF10A), **5000** (for SF10B), **10000** (for SF10C) or **12000** (for SF11C).  *This is the distance in centimeters that the rangefinder can reliably read. The value depends on the model of the lidar.*
-  :ref:`RNGFND_GNDCLEAR <RNGFND_GNDCLEAR>` = 10 *or more accurately the distance in centimetres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

If you instead were using the Telem2 port on the Pixhawk then you would set :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 9, and :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 19200

.. _sf10-i2c-connection:

I2C connection
--------------

.. warning::

   I2C support is present in Plane 3.4 (and higher) and Rover 2.50 (and higher) but should not be used for Copter (`see issue here <https://github.com/ArduPilot/ardupilot/issues/4803>`__).

Connect the SDA line of the Lidar to the SDA line of the I2C port on the Pixhawk, and the SCL line of the Lidar to the SCL line of the I2C port. Also connect the GND and 5V lines.

.. figure:: ../../../images/Pixhawk_Rangefinder_SF10_I2C.jpg
   :target: ../_images/Pixhawk_Rangefinder_SF10_I2C.jpg

   Pixhawk and SF10 Rangefinder: I2CConnection

You then need to configure the rangefinder parameters as shown below
(this is done in the *Mission Planner* **Config/Tuning \| Full Parameter
List** page):

-  :ref:`RNGFND_TYPE <RNGFND_TYPE>` = 7 (LightWareI2C)
-  :ref:`RNGFND_ADDR <RNGFND_ADDR>` = 102 (I2C Address of lidar in decimal).  *Please note that this setting is in decimal and not hexadecimal as shown in the lidar settings screen. The default address is 0x66 which is 102 in decimal.*
-  :ref:`RNGFND_SCALING <RNGFND_SCALING>` = 1
-  :ref:`RNGFND_MIN_CM <RNGFND_MIN_CM>` = 5
-  :ref:`RNGFND_MAX_CM <RNGFND_MAX_CM>` = **2500** (for SF10A), **5000** (for SF10B), **10000** (for SF10C) or **12000** (for SF11C).  *This is the distance in centimeters that the rangefinder can reliably read. The value depends on the model of the lidar.*
-  :ref:`RNGFND_GNDCLEAR <RNGFND_GNDCLEAR>` = 10 *or more accurately the distance in centimetres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

.. warning::

    The default I2C address was 0x55 on older LightWare rangefinders.
    This was changed to prevent conflict with another device on ArduPilot.
    Please check your rangefinder system settings to determine what your I2C address is.

.. _sf10-analog-connection:

Analog connection
-----------------

The SF10's Analog Out pin (5) should be connected to the Pixhawk's 3.3V
ADC (analog to digital converter).  The Pixhawk will provide the
regulated 5V power supply needed by the sensor using the 5V and GND pins
of the ADC connector.

.. figure:: ../../../images/Pixhawk_Rangefinder_SF10_Analog.jpg
   :target: ../_images/Pixhawk_Rangefinder_SF10_Analog.jpg

   Pixhawk and SF10 Rangefinder:Analog Connection

You then need to setup the ADC and rangefinder parameters as shown below
(this is done in the *Mission Planner* **Config/Tuning \| Full Parameter
List** page):

-  :ref:`RNGFND_TYPE <RNGFND_TYPE>` = 1 (Analog)
-  :ref:`RNGFND_PIN <RNGFND_PIN>` = 14 (2nd pin of 3.3V ADC connector)
-  :ref:`RNGFND_SCALING <RNGFND_SCALING>` = **9.76** (for SF10A), **19.531** (for SF10B), **39.06** (for SF10C), **46.87** (for SF11C)
-  :ref:`RNGFND_MIN_CM <RNGFND_MIN_CM>` = 5
-  :ref:`RNGFND_MAX_CM <RNGFND_MAX_CM>` = **2000** (for SF10A), **4500** (for SF10B), **9500** (for SF10C) or **11500** (for SF11C).  *This is the distance in centimeters that the rangefinder can reliably read. The value depends on the model of the lidar.  Note the range is 5m less than using Serial or I2C protocols so that out-of-range can be reliably detected*
-  :ref:`RNGFND_GNDCLEAR <RNGFND_GNDCLEAR>` = 10 *or more accurately the distance in centimetres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

The :ref:`RNGFND_SCALING <RNGFND_SCALING>` value depends on the voltage on the rangefinders output pin at the maximum range. By default the SF10/B will output 2.56V at 50m, so the scaling factor is 50m / 2.56v ≈ 19.53 (the analog
distance range for each of the rangefinder variants can be found in the `SF10 Manual <http://www.lightware.co.za/shop/en/shop/en/index.php?controller=attachment&id_attachment=9>`__).
The manual explains how you can confirm and change the maximum output range/voltage.

.. tip::

   We highly recommend that you tune the ``RNGFND_SCALING`` value by
   comparing the output against a known distance.

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "sonarrange".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg

.. |SF10-B| image:: ../../../images/SF10-B.jpg
    :target: ../_images/SF10-B.jpg
