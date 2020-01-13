.. _common-zubax-gnss-positioning-module-gps-compass-and-barometer:

==========================================================
Zubax GNSS Positioning Module — GPS, Compass and Barometer
==========================================================

`Zubax GNSS 2 <https://zubax.com/products/gnss_2>`__ is a high-performance
positioning module for outdoor environments with doubly redundant `UAVCAN <https://uavcan.org>`__
bus interface. It includes a state-of-the-art GPS/GLONASS receiver, a
high-precision barometer and a 3-axis compass.

.. figure:: ../../../images/Zubax-gnss-top-bottom.jpg
   :target: ../_images/Zubax-gnss-top-bottom.jpg

   Zubax GNSS 2: GPS, Compass and Barometer

In order to use the module with ArduPilot, simply follow the instructions on enabling the CANBUS in the :ref:`common-canbus-setup-advanced` page, followed by :ref:`common-uavcan-setup-advanced` steps. Be sure to set the :ref:`GPS_TYPE<GPS_TYPE>` or :ref:`GPS_TYPE2<GPS_TYPE2>` parameter to 9 to enable use of the UAVCAN GPS.

-  `Product page <https://zubax.com/products/gnss_2>`__ for general information about the device.
