.. _rover-motor-and-servo-configuration:

=============================
Motor and Servo Configuration
=============================

This page describes the few parameters that should be set in order to support the steering and throttle method being used.
This page is closely related to the :ref:`Motor and Servo Connections <rover-motor-and-servo-connections>` page which describes the physical connections between the flight controller, motors and servos.

Separate Steering and Throttle
------------------------------

For separate steering and throttle vehicles these parameters values should be set (they should actually be set by default):

- :ref:`SERVO1_FUNCTION <SERVO1_FUNCTION>` = 26 (GroundSteering)
- :ref:`SERVO3_FUNCTION <SERVO3_FUNCTION>` = 70 (Throttle)

If using the mission planner, the Initial Setup >> Mandatory Hardware >> Servo Output page is a convenient way to do this

.. image:: ../images/rover-motor-and-servo-config1.png
    :target: ../_images/rover-motor-and-servo-config1.png

Skid Steering
-------------

For "Skid steering" vehicles (like R2D2) these parameters values will need to be set:

- :ref:`SERVO1_FUNCTION <SERVO1_FUNCTION>` = 73 (Throttle Left)
- :ref:`SERVO3_FUNCTION <SERVO3_FUNCTION>` = 74 (Throttle Right)

.. image:: ../images/rover-motor-and-servo-config2.png
    :target: ../_images/rover-motor-and-servo-config2.png

.. note::

   In Rover-3.1 (and earlier), skid steering output was enabled by setting the SKID_STEER_OUT parameter to "1".

Motor Driver Types
------------------

At least three different Motor Driver (aka ESC) types are supported which allows using ArduPilot with most motor drivers.  The :ref:`MOT_PWM_TYPE <MOT_PWM_TYPE>` parameter should be used to ensure the output from the flight controller board matches the input required by the motor driver.

- "Normal" is the most common and involves sending PWM values normally between 1000 and 2000 (1ms ~ 2ms)
- "Brushed With Relay" is for brushed motor drivers that use a :ref:`relay pin <common-relay>` to indicate whether it should rotate forwards or backwards.
- "Brushed BiPolar" is for brushed motor drivers that, a bit like "Normal" pwm interpret a low PWM values for reverse, a high PWM value for forward

ESC Configuration
-----------------

Some ESCs support three "Running Models":

#. Forward with brake
#. Forward and reverse with brake
#. Forward and Reverse

For Rover to have full and straight forward control of the throttle it is best to set the "Running Model" to the 3rd option, "Forward and Reverse".  An ESC programming card compatible with the ESC can normally be used to change the ESC's configuration.
