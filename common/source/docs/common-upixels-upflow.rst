.. _common-upixels-upflow:

===========================
UPixels UPFlow Optical Flow
===========================

[copywiki destination="copter,plane,rover"]

The UPixels UPFlow optical flow sensor is a lightweight and low cost optical flow sensor which can be used to improve horizontal position control especially in GPS denied environments.

Where to Buy
------------

The sensor is available from various retailers.
Here are some links:

- `UPIXELS UP-FLOW Optical Flow Module <https://www.aliexpress.com/item/1005008676662121.html>`__

For those in China:

- `UPIXELS UP-FLOW  LC-302-3C <https://h5.m.taobao.com/awp/core/detail.htm?ft=t&id=663846404195>`__

Connection to Autopilot
-------------------------------
.. image:: ../../../images/upixels-upflow-heading.png
   :target: ../_images/upixels-upflow-heading.png
   :width: 200px

.. image:: ../../../images/upixels-upflow-pixhawk.jpg
   :target: ../_images/upixels-upflow-pixhawk.jpg
   :width: 450px

- The flow sensor should be mounted with the head of the little man facing forward and the camera facing down, as shown in the image above.
- Connect the sensor's GND and TX pin to one of the autopilot's serial ports. Note that the TX pin of the sensor should be connected to the RX pin of the autopilot. In the image above the sensor is connected to a Pixhawk's Telem2 port.
- Connect the sensor's VCC to a 3.3V or 5V power source.
- Set :ref:`FLOW_TYPE <FLOW_TYPE>` = 8
- Set :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 18 if using Serial2/Telem2, if connected to another serial port use the corresponding SERIALx_PROTOCOL parameter

Additional Notes
-----------------

- As with the :ref:`PX4Flow sensor <common-px4flow-overview>` a range finder is required to use the sensor for autonomous modes including :ref:`Loiter <loiter-mode>` and :ref:`RTL <rtl-mode>`
- :ref:`FlowHold <flowhold-mode>` does not require the use of a rangefinder
- Performance can be improved by setting the :ref:`sensors position parameters <common-sensor-offset-compensation>`.  For example if the sensor is mounted 2cm forward and 5cm below the frame's center of rotation set :ref:`FLOW_POS_X <FLOW_POS_X>` to 0.02 and :ref:`FLOW_POS_Z <FLOW_POS_Z>` to 0.05.

Testing and Setup
-----------------

- See :ref:`common-optical-flow-sensor-setup` for setup guides.
- Note that recommended value for :ref:`EK2_FLOW_DELAY <EK2_FLOW_DELAY>` or :ref:`EK3_FLOW_DELAY <EK3_FLOW_DELAY>` for this sensor is 10. 
