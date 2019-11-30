.. _common-echologger-ect400:

=============================
Echologger ECT400 echosounder
=============================

The `EchoLogger ECT400 <http://www.echologger.com/products/9>`__ is an echosounder (aka underwater sonar) with a 100m range, 5 degree beam width and 1hz ~ 10hz update rate.
More details can be found in the `datasheet <http://www.echologger.com/media/main/product/6a5d0079-6a03-469d-b94a-fa0854e4f510.pdf>`__

.. image:: ../../../images/echologger-ect400.png

*image courtesy of echologger.com*

.. note::

   Support for this sensor was added to Rover-3.4.

Recommended Hardware
--------------------

- ECT400 can be purchased after `emailing echologger.com <http://echologger.com/contact>`__
- `Sparkfun RS232 To Serial converter <https://www.sparkfun.com/products/8780>`__
- `USB to RS232 converter <https://www.sparkfun.com/products/11304>`__ to allow testing and configuring the sensor with a PC

Connecting and Configuring
--------------------------

The ECT400 provides distance measurements using the NMEA protocol over serial at 115200 baud.

The sensor can be connected to any available serial/uart port on the autopilot.  In the diagram below the first sensor is connected to SERIAL2.

.. image:: ../../../images/echologger-ect400-pixhawk.png
    :target: ../_images/echologger-ect400-pixhawk.png

If the SERIAL2 is used then the following parameters should be set:

-  :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 9 (Lidar)
-  :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 115 (115200 baud)

Then the following range finder related parameters should be set:

-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 17 (NMEA)
-  :ref:`RNGFND1_MIN_CM <RNGFND1_MIN_CM>` = 13
-  :ref:`RNGFND1_MAX_CM <RNGFND1_MAX_CM>` = 10000 (i.e. 100m).  *This is the distance in centimeters that the rangefinder can reliably read.*
-  :ref:`RNGFND1_ORIENT <RNGFND1_ORIENT>` = 25 (i.e. down) if mounted on a boat

Configuring the sensor
----------------------

By default the sensor comes configured to sample the depth at only 1hz and to a maximum depth of 10m.

Using a `USB to RS232 converter <https://www.sparkfun.com/products/11304>`__ connect to the sensor from your PC using a terminal program like `Putty <https://www.putty.org/>`__ (Connection type of "Serial", Speed of "115200" and Serial line of the appropriate COM port).

If connected properly NMEA data should appear on the console.  Type:

::

    #help           (to display the help menu)
    #range 100000   (to set the range to 100m)
    #interval 0.5   (to set update rate to 2hz)
    #nmeadpt 0      (to disable dpt message)
    #nmeamtw 0      (to disable mtw message)
    #nmeaxdr 0      (to disable xdr message)
    #nmeaema 0      (to disable ema message

.. image:: ../../../images/echologger-ect400-sensor-config.png
    :target: ../_images/echologger-ect400-sensor-config.png

More info on NMEA message contents can be found `here <http://www.catb.org/gpsd/NMEA.html>`__

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "sonarrange".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
