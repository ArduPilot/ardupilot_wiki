.. _common-vl53l0x-lidar:

==========================
ST VL53L1X / VL53L1X Lidar
==========================

The `ST VL53L1X <http://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html>`__ (4m range) and `VL53L1X <http://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html>`__ (2m range) lidar is a very small, affordable but relatively short range time-of-flight lidar.

.. image:: ../../../images/vl53l0x.jpg

*images courtesy of Pololu.com and Sparkfun.com*

.. note::

   Support for this lidar was first made available in available in Copter-3.6, Rover-3.3, and Plane-3.9

Where to Buy
------------

- `Sparkfun <https://www.sparkfun.com/products/14667>`__ (VL53L1X, 4m)
- `Pololu <https://www.pololu.com/product/2490>`__ (VL53L0X, 2m)
- `Adafruit <https://www.adafruit.com/product/3317>`__ (VL53L0X, 2m)

Connecting to the Flight Controller
-----------------------------------

Connect the VCC, GND, SDA ans SCL lines of the lidar to the I2C port on the flight controller as shown below.

.. image:: ../../../images/vl53l0x-pixhawk.jpg

Please set the rangefinder parameters as shown below (this can be done using the *Mission Planner* **Config/Tuning \| Full Parameter List** page):

-  :ref:`RNGFND_TYPE <RNGFND_TYPE>` = 16 (VL53L0X)
-  :ref:`RNGFND_ADDR <RNGFND_ADDR>` = 41 (I2C Address of lidar in decimal).  *The sensor's default I2C address is 0x29 hexademical which is 41 in decimal.*
-  :ref:`RNGFND_SCALING <RNGFND_SCALING>` = 1
-  :ref:`RNGFND_MIN_CM <RNGFND_MIN_CM>` = 5
-  :ref:`RNGFND_MAX_CM <RNGFND_MAX_CM>` = **320** (for 4m sensor) or **120** (for 2m).  *This is the distance in centimeters that the rangefinder can reliably read.*
-  :ref:`RNGFND_GNDCLEAR <RNGFND_GNDCLEAR>` = 10 *or more accurately the distance in centimetres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "sonarrange".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
