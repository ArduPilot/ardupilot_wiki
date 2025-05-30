.. _common-wasp200-lidar:

=================================
Attollo Engineering Wasp200 Lidar
=================================

The `Wasp200 <https://www.attolloengineering.com/wasp-200-lrf.html>`__ has a range of up to 200m and an update rate up to 56hz.  More specifications can be found `here <https://attolloengineering.com/wp-content/uploads/2021/05/XM000002-007-User-Manual-WASP-200-LRF-Class-1.pdf>`__.

.. image:: ../../../images/wasp200-lidar.png


Where to Buy
------------

- The `Attollo Engineering online store <https://attolloengineering.com/store/>`__ sells the lidar directly

Connecting to the Autopilot
-----------------------------------

For a serial connection you can use any spare Serial/UART port.  The diagram below shows how to connect to SERIAL4.

.. image:: ../../../images/wasp200-pixhawk.jpg

If the SERIAL4 port on a Pixhawk is being used then the following parameters should be set for the first rangefinder:

-  :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 9 (Lidar)
-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 18 (Wasp200)
-  :ref:`RNGFND1_MIN <RNGFND1_MIN>` = 2
-  :ref:`RNGFND1_MAX <RNGFND1_MAX>` = 200.  *This is the distance in meters that the rangefinder can reliably read.*
-  :ref:`RNGFND1_GNDCLR <RNGFND1_GNDCLR>` = 0.1 *or more accurately the distance in metres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

If instead the Telem2 port was used then the serial parameters listed above should instead be:

-  :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 9

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "rangefinder1".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
