.. _common-benewake-tf02-lidar:

===================================
Benewake TF02 / TF03 lidar/ TF-Luna
===================================

The `Benewake TF02 lidar <http://en.benewake.com/product/detail/5c345c9de5b3a844c4723299.html>`__ has an indoor range of 22m, an outdoor range of 10m, an update rate of 100hz and weighs only 52g.

The `TF03 <http://en.benewake.com/product/detail/5c345cc2e5b3a844c472329a.html>`__ has a range of 50m to 180m (depending upon the surface) and weighs 77g.

The `TF-Luna <http://en.benewake.com/product/detail/5e1c1fd04d839408076b6255.html>`__ has a range of 8m indoor, and 3m outdoor and weighs 5g.

More details on both these lidar can be found in the `benewake.com's downloads area <http://en.benewake.com/download>`__

.. image:: ../../../images/benewake-tf02-topimage.jpg

.. note::

   Support for this sensor is available in Copter-3.6 (and higher) and Rover-3.4 (and higher)

Where to Buy
------------

- `Benewake.com Agents <http://en.benewake.com/news/detail/602f56204d839408076b62a3.html>`__ including `3DXR <https://www.3dxr.co.uk/sensors-c5/lidar-range-and-flow-sensors-c4>`__ and `Unmanned Tech UK <https://www.unmannedtechshop.co.uk/benewake-tf02-lidar-rangefinder-ip65-22m/>`__
- `Benewake's store on Alibaba.com <https://beixingguangzi.en.alibaba.com>`__

Connecting to the Autopilot
-----------------------------------

For a serial connection you can use any spare Serial/UART port.  The diagram below shows how to connect to SERIAL4.

.. image:: ../../../images/benewake-tf02-pixhawk.png

.. note:: TF-03 TX/RX UART wires are different color since it also has a CAN interface option. Consult manufacturer's data sheet.

If the SERIAL4 is being used then the following parameters should be set for the first rangefinder:

-  :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 9 (Lidar)
-  :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` = 115 (115200 baud)
-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 19 (Benewake TF02) for TF02, =20 (Benewake-Serial) for TF03 if Copter 3.6.12, =27 (Benewake TF03) for TF03 for Copter 4.0 and later, and =20 for TF-Luna. Note: using type = 20 on a TF03 will restrict its maximum range to that of a TF02.
-  :ref:`RNGFND1_MIN_CM <RNGFND1_MIN_CM>` = 30 for TFMini, 10 for TFminiPlus and TF-Luna.
-  :ref:`RNGFND1_MAX_CM <RNGFND1_MAX_CM>`: for TF02 use **2000** for indoor, **1000** for outdoor.  For TF03 use **3500** for indoor, **12000** for outdoor. For TF-Luna use **800** for indoor, **300** for outdoor. *This is the distance in centimeters that the rangefinder can reliably read.*
-  :ref:`RNGFND1_GNDCLEAR <RNGFND1_GNDCLEAR>` = 10 *or more accurately the distance in centimetres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

If instead the Telem2 port was used then the serial parameters listed above should instead be:

-  :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 9
-  :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 115

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "sonarrange".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
