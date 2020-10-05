.. _common-vl53l0x-lidar:

==========================
ST VL53L0X / VL53L1X Lidar
==========================

The `VL53L0X <https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html>`__ lidar is a very small, affordable but relatively short range (2m range) time-of-flight lidar

.. image:: ../../../images/vl53l0x.jpg

*images courtesy of Pololu.com*

.. note::

   Support for this lidar was first made available in available in Copter-3.6, Rover-3.3, and Plane-3.9.
   Support the 4m `ST VL53L1X <https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html>`__ will be released with Copter-4.0.


.. note::
   
   This sensor type is suitable for indoor use, however its range and precision is significantly reduced in bright sunlight conditions and is not recommended for outdoor use.

Where to Buy
------------

- `Pololu <https://www.pololu.com/product/2490>`__ (VL53L0X, 2m)
- `Adafruit <https://www.adafruit.com/product/3317>`__ (VL53L0X, 2m)

Connecting to the Autopilot
-----------------------------------

Connect the VCC, GND, SDA and SCL lines of the lidar to the I2C port on the autopilot as shown below for the first rangefinder.

.. image:: ../../../images/vl53l0x-pixhawk.jpg

Please set the rangefinder parameters as shown below (this can be done using the *Mission Planner* **Config/Tuning \| Full Parameter List** page):

-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 16 (VL53L0X)
-  :ref:`RNGFND1_ADDR <RNGFND1_ADDR>` = 41 (I2C Address of lidar in decimal).  *The sensor's default I2C address is 0x29 hexademical which is 41 in decimal.*
-  :ref:`RNGFND1_SCALING <RNGFND1_SCALING>` = 1
-  :ref:`RNGFND1_MIN_CM <RNGFND1_MIN_CM>` = 5
-  :ref:`RNGFND1_MAX_CM <RNGFND1_MAX_CM>` = **120** for the VL53L0X, **360** for the VL53L1X.  *This is the distance in cm that the rangefinder can reliably read.*
-  :ref:`RNGFND1_GNDCLEAR <RNGFND1_GNDCLEAR>` = 10 *or more accurately the distance in cm from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "sonarrange".(The value of this image is not a real value.)

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
