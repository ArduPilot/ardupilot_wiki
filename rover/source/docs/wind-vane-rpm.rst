.. _wind-vane-rpm:

=======================
RPM wind speed sensor
=======================
 
A cups type anemometer can be used via the :ref:`RPM library <common-rpm>` (as of master feb 2019). This is selected by setting :ref:`WNDVN_SPEED_TYPE <WNDVN_SPEED_TYPE>`
to 3. The RPM library must be setup accordingly using the first :ref:`RPM sensor <RPM_TYPE>`. The :ref:`RPM_SCALING <RPM_SCALING>` value must be calculated
such that wind speed in m/s is returned. 

.. image:: ../../../images/wind-vane-cups.jpg
    :target: ../_images/wind-vane-cups.jpg
    :width: 50%
