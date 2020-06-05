.. _common-brushed-motors:

==============
Brushed Motors
==============

ArduPilot supports multiple methods to control brushed motors

- Brushed motor ESCs that support RC PWM input (PWM signals that are 1ms to 2ms in width) are the easiest to use because no special configuration is required and the rest of this document can be ignored.  Most hobby grade RC Car ESCs are of this type
[site wiki="copter"]
- "Brushed" motors accept a duty cycle to control the speed but only spin in one direction
[/site]
[site wiki="rover"]
- "Brushed With Relay" motor drivers accept a duty cycle to control the speed and also have a separate pin used to control direction
- "Brushed BiPolar" motor drivers accept a duty cycle where 0% duty cycle means full speed in one direction, 100% duty cycle is full speed in the other direction and 50% duty cycle is stopped
[/site]

Verified Motor Drivers
----------------------

- `Sabertooth Dual 32A Motor Driver  <https://www.dimensionengineering.com/products/sabertooth2x32>`__ support "Brushed BiPolar"
- `Pololu G2 High-Power Motor Driver <https://www.pololu.com/product/2991>`__ support "BrushedWithRelay"
- `RoboClaw 2x7A Motor Controller <https://www.pololu.com/product/3284>`__ supports "Brushed BiPolar"
- :ref:`SkyRocket <copter:skyrocket>` drones use "Brushed" motors

Connection and Configuration
----------------------------

- Connect the motor drivers / ESCs to the autopilot's output pins just as you would for regular brushless ESCs but note the warning at the bottom regarding mixing brushed outputs with servos

[site wiki="copter"]
- Set :ref:`MOT_PWM_TYPE <MOT_PWM_TYPE>` = 3 for "Brushed" and reboot the autopilot
- Set :ref:`RC_SPEED <RC_SPEED>` = 16000 to set the refresh rate to 16k (other values from 1000 to 20000 are possible)
[/site]
[site wiki="rover"]
- If using "Brushed With Relay" connect the :ref:`Relay <common-relay>` pin(s) to the motor driver's direction pin(s).  For skid-steering vehicles Relay1 is for the left motor, Relay2 for the right motor
- Set :ref:`MOT_PWM_TYPE <MOT_PWM_TYPE>` = 3 for "BrushedWithRelay" or "4" for "BrushedBIPolar" and reboot the autopilot
- :ref:`MOT_PWM_FREQ <MOT_PWM_FREQ>` defaults to 16000 but can be changed to any value from 1000 to 20000 to change the output frequency
[/site]

.. warning::

    If the autopilot will also control servos (which use regular RC PWM) take care that the servos are not in the same "PWM output group" as any motor outputs.  The grouping is only documented on some of the :ref:`AutoPilot Hardware Options <common-autopilots>` pages but for a regular Pixhawk or Cube autopilot MAIN OUT 1 ~ 4 are in group1 meaning that if any brushed motors are configured for MAIN OUT 1 to 4 then servos must be connected to output 5 or higher

.. warning::

    ArduPilot does not currently support controlling both brushed and brushless **motors** at the same time

[copywiki destination="copter,rover"]
