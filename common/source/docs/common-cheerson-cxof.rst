.. _common-cheerson-cxof:

===========================
Cheerson CX-OF Optical Flow
===========================

[copywiki destination="copter"]

The Cheerson CX-OF optical flow sensor is a leightweight and low cost optical flow sensor which can be used to improve horizontal position control especially in GPS denied environments.

..  youtube:: SSISkG58cDk
    :width: 100%


.. warning::

   Support for this sensor will be included in Copter-3.7 (and higher)

Where to Buy
------------

The sensor is available from various retailers incluing `Banggood.com <https://www.banggood.com/Cheerson-CX-OF-CXOF-RC-Quadcopter-Spare-Parts-Optical-Flow-Module-p-1215911.html>`__ and `AliExpress <https://www.aliexpress.com/item/Original-Cheerson-CX-OF-CXOF-RC-Quadcopter-Spare-Parts-Optical-Flow-Module-for-RC-Toys-Models/32838098799.html>`__.  More retailers can be found by searching for "CX-OF spare parts".

Connection to Flight Controller
-------------------------------

.. image:: ../../../images/cheerson-cxof-pixhawk.jpg
   :target: ../_images/cheerson-cxof-pixhawk.jpg
   :width: 450px

- The flow sensor should be mounted on the underside of the copter with the camera lens pointing downwards.  The side of the sensor with the "V2.0" label should be towards the front of the vehicle.  The image above is incorrect because the flight controller's arrow is pointing down while the sensor's "V2.0" label is close to the top.
- Connect the sensor's TX and VSS (aka GND) pins to one of the flight controller's serial ports.  In the image above the sensor is connected to a Pixhawk's Telem2 port
- Connect the sensor's VDD (aka VCC or 3.3V) to a 3.3V power source.  In the above diagram the Pixhawk's Switch port is used but another alternative would be the SPKT/DSM port's power pin
- Set :ref:`FLOW_ENABLE <FLOW_ENABLE>` = 1
- Set :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 18 if using Serial2/Telem2, if connected to another serial port use the corresponding SERIALx_PROTOCOL parameter
- Optionally set :ref:`EK2_GPS_TYPE <EK2_GPS_TYPE>` = 3 to force the EKF to only use the optical flow sensor and not use the GPS

Additional Notes
-----------------

- As with the :ref:`PX4Flow sensor <common-px4flow-overview>` a range finder is required to use the sensor for autonomous modes including :ref:`Loiter <loiter-mode>` and :ref:`RTL <rtl-mode>`
- :ref:`FlowHold <flowhold-mode>` does not require the use of a rangefinder
- The sensor has been successfully tested to altitudes of about 40m
- Uncheck the :ref:`ARMING_CHECK <ARMING_CHECK>` parameter's "Parameters" bit to remove the need to manually lift the vehicle to 1m once before takeoff (this pre-arm check is designed to ensure the range finder is working)
