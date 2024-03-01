.. _common-gps-septentrio:

================================
Septentrio AsteRx UAS GPS Family
================================

The Septentrio `AsteRx-m2 UAS <http://www.septentrio.com/products/gnss-receivers/rover-base-receivers/oem-receiver-boards/asterx-m2-uas/>`__ RTK GPS and other AsteRX-m RTK GPS are relatively expensive but also highly accurate RTK GPS receivers.

.. image:: ../../../images/gps-septrino.png
	:target: ../_images/gps-septrino.png

.. note::

     Mission Planner GCS is the only GCS to support this feature currently.

Setup instructions
==================

To setup this using GPS1 input (serial3) configure these parameters:

- :ref:`GPS_INJECT_TO<GPS_INJECT_TO>` = 0 (send to first GPS)
- :ref:`GPS_RATE_MS<GPS_RATE_MS>` = 100
- :ref:`GPS_TYPE<GPS_TYPE>` = 10 (SBF)
- :ref:`SERIAL3_BAUD<SERIAL3_BAUD>` = 115
- :ref:`SERIAL3_PROTOCOL<SERIAL3_PROTOCOL>` = 5 (GPS)

To setup this using GPS2 input (serial4) configure these parameters:

- :ref:`GPS_INJECT_TO<GPS_INJECT_TO>` = 1 (send to 2nd GPS)
- :ref:`GPS_RATE_MS2<GPS_RATE_MS2>` = 100
- :ref:`GPS_TYPE2<GPS_TYPE2>` = 10 (SBF)
- :ref:`SERIAL4_BAUD<SERIAL4_BAUD>` = 115
- :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` = 5 (GPS)

The Septentrio driver currently does not use `GPS_RATE_MS` and `GPS_RATE_MS2`.
It is still important to set them to the expected value of 100 (10Hz) as they are used as weights when combining GPS results.

If you want to inject RTCM corrections to both GPS1 and GPS2 then use:

- :ref:`GPS_INJECT_TO<GPS_INJECT_TO>` = 127 (send to all)

Note:
Baud rate is auto-negotiated by the ArduPilot firmware, and might get set to something different than the 115200 baud.

GPS-for-Yaw
===========

:ref:`GPS-for-yaw<common-gps-for-yaw>` is possible using two separate GPSs or, in ArduPilot 4.5 (or higher), a single unit with two antennas may also be used.

If a single unit with two antennas is used please set the following parameters:

- :ref:`GPS_TYPE <GPS_TYPE>` = 26 (SBF-DualAntenna)
- :ref:`GPS_MB1_TYPE <GPS_MB1_TYPE>` = 1 (RelativeToCustomBase)
- :ref:`GPS_MB1_OFS_X <GPS_MB1_OFS_X>`, :ref:`GPS_MB1_OFS_Y <GPS_MB1_OFS_Y>` and :ref:`GPS_MB1_OFS_Z <GPS_MB1_OFS_Z>` to match the offset of the main antenna from the second in meters.  For example if the second antenna is 50cm to the right of main antenna set :ref:`GPS_MB1_OFS_Y <GPS_MB1_OFS_Y>` = -0.5.
- :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` = 2 (GPS)

See the :ref:`Antenna Offsets<antenna-offsets>` section for a diagram illustrating the directions of these offsets.

Note that the yaw calculation is only correct while the vehicle is upright,
so it should not be used on vehicle that spend significant time at extreme lean angles (e.g tail sitters).

RTK setup with Mission Planner and Septentrio AsteRx-m rover
============================================================
..  youtube:: HWJnG3tu9iM
    :width: 100%
