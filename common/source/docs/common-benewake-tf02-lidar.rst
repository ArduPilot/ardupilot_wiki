.. _common-benewake-tf02-lidar:

====================================
Benewake TF02 / TF03 lidar / TF-Luna
====================================

The TF02 lidar has an indoor range of 22m, an outdoor range of 10m, an update rate of 100Hz and weighs only 52g.

The `TF02-Pro <https://en.benewake.com/TF02Pro/index.html>`__ has an indoor range of 40m, an outdoor range of 13.5m, an update rate of 100Hz and weighs only 50g.

The `TF03 <https://en.benewake.com/TF03/index.html>`__ has a range of 50m to 180m (depending upon the surface) and weighs 77g.

The `TF-Luna <https://en.benewake.com/TFLuna/index.html>`__ has a range of 8m indoor, and 3m outdoor and weighs 5g.

More details on both these lidar can be found in the `benewake.com's downloads area <https://en.benewake.com/DataDownload>`__

.. image:: ../../../images/benewake-tf02-topimage.jpg

Where to Buy
------------

- `Benewake.com Distributors <https://en.benewake.com/Agent/index.html>`__ including `3DXR <https://www.3dxr.co.uk/sensors-c5/lidar-range-and-flow-sensors-c4>`__ and `Unmanned Tech UK <https://www.unmannedtechshop.co.uk/benewake-tf02-lidar-rangefinder-ip65-22m/>`__
- `Benewake's store on Alibaba.com <https://beixingguangzi.en.alibaba.com>`__

Connecting to the Autopilot
-----------------------------------

For a serial connection you can use any spare Serial/UART port.  The diagram below shows how to connect to SERIAL4.

.. image:: ../../../images/benewake-tf02-pixhawk.png

.. note:: TF03 TX/RX UART wires are different color since it also has a CAN interface option. Consult manufacturer's data sheet.

If the SERIAL4 is being used then the following parameters should be set for the first rangefinder:

-  :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 9 (Lidar)
-  :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` = 115 (115200 baud)
-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 19 (Benewake TF02) for TF02, 27 (Benewake TF03) for TF03 and TF02-Pro, and 20 (Benewake-Serial) for TF-Luna
-  :ref:`RNGFND1_MIN <RNGFND1_MIN>` = 0.1
-  :ref:`RNGFND1_MAX <RNGFND1_MAX>`: for TF02 use **20** for indoor, **10** for outdoor.  For TF03 use **35** for indoor, **12** for outdoor. For TF-Luna use **8** for indoor, **3** for outdoor. *This is the distance in meters that the rangefinder can reliably read.*
-  :ref:`RNGFND1_GNDCLR <RNGFND1_GNDCLR>` = 0.1 *or more accurately the distance in metres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

If instead the Telem2 port was used then the serial parameters listed above should instead be:

-  :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 9
-  :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 115

.. note:: the TF03 can be programmed to use a DroneCAN interface instead of a serial interface. In this case the :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 34, and follow the :ref:`DroneCAN setup instructions <common-uavcan-setup-advanced>`.

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "rangefinder1".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
