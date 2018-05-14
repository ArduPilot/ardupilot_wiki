.. _common-benewake-tfmini-lidar:

=====================
Benewake TFmini lidar
=====================

The `Benewake TFmini lidar <http://www.benewake.com/en/tfmini.html>`__ is a very light weight (5g) lidar with a maximum range of about 7m.  More specifications can be found `here <http://www.benewake.com/en/canshu/show-171.html>`__.

.. image:: ../../../images/benewake-tfmini-topimage.jpg

.. note::

   These instructions are a work-in-progress and have not yet been fully verified by the development team

Where to Buy
------------

- the `Benewake.com Agents <http://www.benewake.com/en/agent.html>`__ page provides the full list of retailers including `3DXR <https://www.3dxr.co.uk/product/benewake-tfmini/>`__.

Connecting to the Flight Controller
-----------------------------------

For a serial connection you can use any spare Serial/UART port.  The diagram below shows how to connect to SERIAL4.

.. image:: ../../../images/benewake-tfmini-pixhawk.png

If the SERIAL4/5 port on a Pixhawk is being used then the following parameters should be set:

-  :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 9 (Lidar)
-  :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` = 115 (115200 baud)
-  :ref:`RNGFND_TYPE <RNGFND_TYPE>` = 8 (LightWareSerial)
-  :ref:`RNGFND_SCALING <RNGFND_SCALING>` = 1
-  :ref:`RNGFND_MIN_CM <RNGFND_MIN_CM>` = 5
-  :ref:`RNGFND_MAX_CM <RNGFND_MAX_CM>` = **700**.  *This is the distance in centimeters that the rangefinder can reliably read.*
-  :ref:`RNGFND_GNDCLEAR <RNGFND_GNDCLEAR>` = 10 *or more accurately the distance in centimetres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

If instead the Telem2 port was used then the serial parameters listed above should instead be:

-  :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 9
-  :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 115

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "sonarrange".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
