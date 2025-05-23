.. _common-leddar-one-lidar:

================
Leddar One Lidar
================

The `Leddar One Lidar <https://leddartech.com/modules/leddarone/>`__ is a light weight and reasonably priced lidar with 40m range, 70hz update rate and 3-degree diffuse beam.  For more details please refer to the `datasheet <https://leddartech.com/app/uploads/dlm_uploads/2016/02/Datasheet-LeddarOne.pdf>`__.

.. figure:: ../../../images/leddar-one.jpg
   :target: ../_images/leddar-one.jpg

Connecting to the AutoPilot
===========================

The sensor's serial connection can be connected to any spare serial port (i.e. Telem1, Teleme2, Serial4) on the autopilot.  Connect the lidar's RX line to the autopilot's UART TX line, the lidar's TX line to the UART's RX then also connect the GND and 5V lines.  The image below shows how the sensor output pins can be connected to a Pixhawk's Serial4 port.

.. figure:: ../../../images/leddar-one-pixhawk.jpg
   :target: ../_images/leddar-one-pixhawk.jpg

You then need to setup the serial port and rangefinder parameters. If
you have used the SERIAL4/5 port on the Pixhawk then you would set the
following parameters for the first rangefinder (this is done on the Mission Planner's
**Config/Tuning \| Full Parameter List** page):

-  :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = 9 (Lidar)
-  :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` = 115200
-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 12 (LeddarOne)
-  :ref:`RNGFND1_SCALING <RNGFND1_SCALING>` = 1
-  :ref:`RNGFND1_MIN <RNGFND1_MIN>` = 0.05
-  :ref:`RNGFND1_MAX <RNGFND1_MAX>` = **40** (40m) *This is the distance in meters that the rangefinder can reliably read.*
-  :ref:`RNGFND1_GNDCLR <RNGFND1_GNDCLR>` = 0.1 *or more accurately the distance in metres from the range finder to the ground when the vehicle is landed.  This value depends on how you have mounted the rangefinder.*

If you instead were using the Telem2 port on the Pixhawk then you would set :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 9, and :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 115200

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "rangefinder1".

.. image:: ../../../images/mp_rangefinder_lidarlite_testing.jpg
    :target: ../_images/mp_rangefinder_lidarlite_testing.jpg
