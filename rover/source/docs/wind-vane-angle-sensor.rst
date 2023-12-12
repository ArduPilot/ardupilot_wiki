.. _wind-vane-angle-sensor:

=======================
Angle Sensor Wind Vane
=======================

An angle sensor can be configured as the wind-vane direction source. In order to use this option, you must first configure
the angle sensor as described in the :ref:`Angle Sensor library <common-angle-sensor>` documentation.

To select the angle sensor as the wind-vane direction source, set :ref:`WNDVN_TYPE <WNDVN_TYPE>` = 4. Then, select the absolute 
encoder instance via the :ref:`WNDVN_DIR_PIN <WNDVN_DIR_PIN>` parameter (0=AngleSensor1, 1=AngleSensor2).


Wind Vane Construction
++++++++++++++++++++++
A 3D-Printable model for an angle sensor wind-vane is available on `thingiverse <https://www.thingiverse.com/thing:4247123>`__. This example 
uses an AS5048B breakout board available `here <https://www.digikey.com/en/products/detail/ams/AS5048A-TS_EK_AB/3188612>`__.
