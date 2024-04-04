.. _common-zubax-gnss-positioning-module-gps-compass-and-barometer:

==========================================================
Zubax GNSS Positioning Module — GPS, Compass and Barometer
==========================================================

`Zubax GNSS 2 <https://zubax.com/products/gnss_2>`__ is a high-performance
positioning module for outdoor environments with doubly redundant `DroneCAN <https://dronecan.org>`__
bus interface. It includes a state-of-the-art GPS/GLONASS receiver, a
high-precision barometer and a 3-axis compass.

.. figure:: ../../../images/ZubaxGNSS2.jpg
   :target: ../_images/ZubaxGNSS2.jpg

   Zubax GNSS 2: GPS, Compass and Barometer

The following parameter should be set on the autopilot (and then reboot the autopilot):

- :ref:`CAN_P1_DRIVER <CAN_P1_DRIVER>` = 1 (to enable the 1st CAN port)
- :ref:`GPS1_TYPE <GPS1_TYPE>` = 9 (DroneCAN)

If the device does not work please follow the instructions on enabling the CANBUS in the :ref:`common-canbus-setup-advanced` page, followed by :ref:`common-uavcan-setup-advanced` steps and then set the :ref:`GPS1_TYPE <GPS1_TYPE>` or :ref:`GPS2_TYPE <GPS2_TYPE>` parameter to 9.

The `manufacturer's product page is here <https://zubax.com/products/gnss_2>`__ for general information about the device.
