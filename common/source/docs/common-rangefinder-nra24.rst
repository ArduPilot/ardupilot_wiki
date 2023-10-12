.. _common-rangefinder-nooploop-nra24.rst:

================
Nanoradar NRA 24
================

.. note::
    Support for this sensor is available in firmware versions 4.5 and later.

The `NRA24 <http://en.nanoradar.cn/Article/detail/id/372.html>`__ is K-band radar commonly used for detecting height above terrain in all-weather conditions.
Currently only connection via CAN is supported. Serial versions of this Radar isn't officially supported.


Connecting via CAN
=========================

Setup the sensor through the NSM Software. In particular set the "ID" to a unique number if planning to use multiple NRA24 on a single CAN bus. You can also set the bitrate from the software.

.. note::
    ArduPilot does not currently support  same CAN rangefinders connected to different CAN ports. If you would like yo use multiple CAN rangefinders together, connect them to the same bus.


Connecting to the Autopilot
==============================
Setup the sensor through the NAssistant Software provided by the manufacturer. More details can be found `here <https://www.nooploop.com/download/>`__. Set the "ID" to a unique number if planning to use multiple NRA24 on a single CAN bus. You can also set the bitrate from the software to match the CAN bus bitrate (500000 is recommended).

For a serial connection you can use any spare CAN port. Since this is not a DroneCAN based sensor, you will not be able to use the connected port with any other sensor. The diagram below shows how to connect to CAN2.

.. image:: ../../../images/nra24_connection.png
    :target: ../_images/nra24_connection.png

Set the following parameters

-  :ref:`CAN_P2_DRIVER <CAN_P2_DRIVER>` = 1 (to enable the 1st CAN port)
-  :ref:`CAN_P2_BITRATE<CAN_P2_BITRATE>` = 500000 (Or as set in NAssistant)
-  :ref:`CAN_D2_PROTOCOL<CAN_D2_PROTOCOL>` = 14
-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = “39" (Reboot after setting this)
-  :ref:`RNGFND1_MAX_CM <RNGFND1_MAX_CM>` = "19000" (i.e. 200m max range and 10m buffer)
-  :ref:`RNGFND1_MAX_CM <RNGFND1_MIN_CM>` = "60"
-  :ref:`RNGFND1_RECV_ID <RNGFND1_RECV_ID>` = ID of the sensor (0 to accept all distances)


Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "rangefinder1".
