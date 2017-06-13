.. _common-lightware-lw20-lidar:

=====================
LightWare SF20 / LW20
=====================

The `Lightware SF20 <http://lightware.co.za/shop2017/home/53-sf20-100-m.html>`__ and `LW20 <http://lightware.co.za/shop2017/drone-altimeters/51-lw20-100-m.html>`__ are small but long range (100m) and accurate range finders.
There are two models, the LW20/Ser which uses a serial interface and LW20/I2C which uses an I2C interface.

.. image:: ../../../images/lightware-lw20.png

Serial connection
-----------------

For a serial connection you can use any spare UART. Connect the RX line
of the UART to the TX line of the Lidar, and the TX line of the UART to
the RX line of the Lidar. Also connect the GND and 5V lines. You do not
need flow control pins.

The diagram below shows how to connect to SERIAL4.

.. image:: ../../../images/pixhawk-lightware-lw20-serial.jpg

If using the caseless SF20 ensure the cable looks like below:

.. image:: ../../../images/lightware-lw20-serial-cable.png

You then need to setup the serial port and rangefinder parameters. If
you have used the SERIAL4/5 port on the Pixhawk then you would set the
following parameters (this can be done using the *Mission Planner*
**Config/Tuning \| Full Parameter List** page):

-  :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 9 (Lidar)
-  :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` = 115 (115200 baud)
-  :ref:`RNGFND_TYPE <RNGFND_TYPE>` = 8 (LightWareSerial)
-  :ref:`RNGFND_SCALING <RNGFND_SCALING>` = 1
-  :ref:`RNGFND_MIN_CM <RNGFND_MIN_CM>` = 5
-  :ref:`RNGFND_MAX_CM <RNGFND_MAX_CM>` = **9500**.  *This is the distance in centimeters that the rangefinder can reliably read.*
-  :ref:`RNGFND_GNDCLEAR <RNGFND_GNDCLEAR>` = 10 *or more accurately the distance in centimetres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

If you instead were using the Telem2 port on the Pixhawk then you would set :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 9, and :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 19200

I2C connection
--------------

Connect the SDA line of the Lidar to the SDA line of the I2C port on the Pixhawk, and the SCL line of the Lidar to the SCL line of the I2C port. Also connect the GND and 5V lines.

.. image:: ../../../images/pixhawk-lightware-lw20-i2c.jpg

If using the caseless SF20 ensure the cable looks like below:

.. image:: ../../../images/lightware-lw20-i2c-cable.png

You then need to configure the rangefinder parameters as shown below
(this cn be done suing the *Mission Planner* **Config/Tuning \| Full Parameter List** page):

-  :ref:`RNGFND_TYPE <RNGFND_TYPE>` = 7 (LightWareI2C)
-  :ref:`RNGFND_ADDR <RNGFND_ADDR>` = 102 (I2C Address of lidar in decimal).  *Note that this setting is in decimal. The default address is 0x66 hexademical which is 102 in decimal.*
-  :ref:`RNGFND_SCALING <RNGFND_SCALING>` = 1
-  :ref:`RNGFND_MIN_CM <RNGFND_MIN_CM>` = 5
-  :ref:`RNGFND_MAX_CM <RNGFND_MAX_CM>` = **9500**.  *This is the distance in centimeters that the rangefinder can reliably read.*
-  :ref:`RNGFND_GNDCLEAR <RNGFND_GNDCLEAR>` = 10 *or more accurately the distance in centimetres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "sonarrange".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
