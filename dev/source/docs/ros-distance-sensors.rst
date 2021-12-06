.. _ros-distance-sensors:

=========================
ROS distance sensor usage
=========================

Distance sensor message
=======================

ArduPilot support `distance sensor message <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`__ both as input and output.
The following chapters will explain you how to use distance sensor with ROS for :

- Rangefinder : how to receive data from rangefinder plug in a FCU (e.g. pixhawk) and how to send data from ROS to FCU
- Proximity : how to receive data from proximity lib from FCU and how to send 360 degrees lidar data to FCU


Rangefinder
===========

Receive from FCU
----------------

.. warning::

    Distance sensor message is currently support on :

    - ArduPilot Master
    - Copter >= 3.5.1 (Planned)
    - Rover >= 3.2.0 (Planned)
    - Plane >= 3.8.0 (Planned)

By default, ArduPilot will send distance sensors message for each rangefinder connected on FCU on Rover and only downward oriented (PITCH_270) on Copter and Plane.
Example : launch SITL and add a analog rangefinder, see `SITL instruction here <https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#adding-a-virtual-rangefinder>`__.
Launch MAVROS with default config. You should have rangefinder data in /mavros/distance_sensor/rangefinder_pub.

.. code-block:: none

    Configuration in apm_config.yaml:
    distance_sensor:    <== plugin name
      rangefinder_pub:  <== ROS publisher name
        id: 0             <== sensor instance number starting at 0
        frame_id: "lidar" <== frame name for TF
        #orientation: PITCH_270 # sended by FCU
        field_of_view: 0.0  <== sensor FOV see `ROS instructions here <http://docs.ros.org/api/sensor_msgs/html/msg/Range.html>`__

Send to FCU
-----------

ArduPilot support receiving rangefinder data coming from Companion Computer for example.
To do that, setup a MAVLink rangefinder on ArduPilot side and simply set a subscriber in MAVROS plugin :

.. code-block:: none

    Configuration in apm_config.yaml:
    distance_sensor:       <== plugin name
      rangefinder_sub:     <== ROS subscriber name
        subscriber: true   <== set subscriber type
        id: 1              <== sensor instance number must match the one set on ArduPilot side
        orientation: PITCH_270  <== only that orientation are supported by Copter 3.4+

Now, publish a sensor_msgs/Range message on /mavros/distance_sensor/rangefinder_sub.
Using a GCS, you can see data in rangefinder mesurements.

Proximity
=========

.. warning::

    Work in progress

We are keen to improve ArduPilot's support of ROS so if you find issues (such as commands that do not seem to be supported), please report them in the `ArduPilot issues list <https://github.com/ArduPilot/ardupilot/issues>`__ with a title that includes "ROS" and we will attempt to resolve them as quickly as possible.
