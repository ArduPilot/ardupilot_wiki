.. _common-hereflow:

================================
Hex HereFlow Optical Flow Sensor
================================

[copywiki destination="copter,plane,rover"]

The `HereFlow optical flow sensor <http://www.proficnc.com/all-products/185-pixhawk2-suite.html>`__ is a lightweight optical flow sensor including a short range lidar which uses the CAN protocol to communicate with the autopilot.  This can be used to improve horizontal position control especially in GPS denied environments.

..  youtube:: MKJB_7cA_0s
    :width: 100%

.. warning::

   The lidar included with the HereFlow is very short range especially outdoors.  We strongly recommend using a :ref:`longer range lidar instead <common-rangefinder-landingpage>`.

Where to Buy
------------

The sensor is available from `Hex resellers <http://www.proficnc.com/stores>`__

Connection to Autopilot
-----------------------

.. image:: ../../../images/hereflow-pixhawk.jpg
   :target: ../_images/hereflow-pixhawk.jpg
   :width: 450px

- The flow sensor should be mounted on the underside of the copter with the camera lens pointing downwards.  The side of the sensor with the lens should be towards the front of the vehicle.
- Connect the sensor to the autopilots' CAN port (if using a Cube note that pre-Apr 2019 boards have CAN1 and CAN2 swapped)
- Set :ref:`FLOW_TYPE <FLOW_TYPE>` = 6 (DroneCAN)
- Set :ref:`CAN_P1_DRIVER <CAN_P1_DRIVER>` = 1 to enable DroneCAN
- Set :ref:`CAN_D1_PROTOCOL <CAN_D1_PROTOCOL>` = 1 (DroneCAN)
- Optionally set :ref:`BRD_BOOT_DELAY <BRD_BOOT_DELAY>` = 3000 (3 seconds) to slow the autopilot's startup which allows the flow sensor to boot up first and avoid initialisation issues

To use the onboard lidar (not recommended):

- Set :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 24 (DroneCAN)
- Set :ref:`RNGFND1_MAX_CM <RNGFND1_MAX_CM>` = 300 to set range finder's maximum range to 3m

Additional Notes
-----------------

- As with the :ref:`PX4Flow sensor <common-px4flow-overview>` a range finder is required to use the sensor for autonomous modes including :ref:`Loiter <loiter-mode>` and :ref:`RTL <rtl-mode>`
- :ref:`FlowHold <flowhold-mode>` does not require the use of a rangefinder
- Performance can be improved by setting the :ref:`sensors position parameters <common-sensor-offset-compensation>`.  For example if the sensor is mounted 2cm forward and 5cm below the frame's center of rotation set :ref:`FLOW_POS_X <FLOW_POS_X>` to 0.02 and :ref:`FLOW_POS_Z <FLOW_POS_Z>` to 0.05.

Testing and Setup
-----------------

See :ref:`common-optical-flow-sensor-setup`
