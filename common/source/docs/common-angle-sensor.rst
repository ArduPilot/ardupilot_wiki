.. _common-angle-sensor:
[copywiki destination="rover"]

=====
Angle Sensor
=====

This article describes how to attach an angle sensor so that your vehicle can sense angular position. This is used most commonly 
for :ref:`wind-vane sensors <wind-vane>` for sailing rovers.

Currently, only AS5048B Encoders are supported.

AutoPilot connection
--------------------
Connect the encoder to the autopilot's I2C Port using standard connections.

Autopilot Configuration
-----------------------

Connect to the autopilot with a ground station and set these parameters and then reload the parameters.

- :ref:`ANG_TYPE <ANG_TYPE>` = 1 (Enable)

After reloading parameters, set the following parameters:

- :ref:`ANG_BUS <ANG_BUS>` - set this to the serial bus ID for the I2C bus you've connected to.
- :ref:`ANG_ADDR <ANG_ADDR>` - set this to the I2C Address of your sensor (defaults to 64 for AS5048B).
- :ref:`ANG_OFFS <ANG_OFFS>` - set this to apply an offset in degrees to the zero position.
- :ref:`ANG_DIR <ANG_DIR>` - set this to -1 if you need to reverse the direction of rotation for the sensor.


