.. _rover-motor-and-servo-connections:

===========================
Motor and Servo Connections
===========================

Two steering/throttle methods are supported and each requires slightly different wiring and configuration as described below.

Separate Steering and Throttle
------------------------------

Most RC cars are like full sized cars in that they have separate steering and throttle controls.
For these rovers, the steering servo (which normally turns the front wheels) should be connected to the autopilot's RC Output 1.  The motor's ESC (which normally controls the speed of the back wheels) should be connected to RC Output 3.

.. image:: ../images/rover-motor-connections.jpg
    :target: ../_images/rover-motor-connections.jpg

For this setup these parameters values should be set (they should actually be set by default).  More details can be found on the :ref:`Motor and Servo Configuration <rover-motor-and-servo-configuration>` page.

- :ref:`SERVO1_FUNCTION <SERVO1_FUNCTION>` = 26 (Ground Steering)
- :ref:`SERVO3_FUNCTION <SERVO3_FUNCTION>` = 70 (Throttle)

.. _rover-motor-and-servo-connections-skid-steering:

Skid Steering
-------------

"Skid steering" vehicles (like R2D2) control their direction and forward/reverse motions by varying the speed of two (or more) independent wheels.  For these style rovers the left wheel should be connected to RC Output 1 and the right wheel should be connected to RC Output 3.

.. image:: ../images/rover-skid-steer-motor-connections.jpg
    :target: ../_images/rover-skid-steer-motor-connections.jpg

For this setup these parameters values will need to be set.  More details can be found on the :ref:`Motor and Servo Configuration <rover-motor-and-servo-configuration>` page.

- :ref:`SERVO1_FUNCTION <SERVO1_FUNCTION>` = 73 (Throttle Left)
- :ref:`SERVO3_FUNCTION <SERVO3_FUNCTION>` = 74 (Throttle Right)

Omni Vehicles
-------------

Omni vehicles can move laterally (i.e. left and right) without changing their heading with the help of omni wheels (`like these from RobotShop <https://www.robotshop.com/en/6-duraomni-wheel.html>`__) or thrusters.  ArduPilot supports three configurations X, Plus and "3".  The autopilot motor outputs that should be connected to each motor are shown below.

.. image:: ../images/omni-motor-order.png
    :target: ../_images/omni-motor-order.png

For this setup these parameters values will need to be set

- :ref:`FRAME_TYPE <FRAME_TYPE>` = 1 (Omni3), 2 (OmniX) or 3 (OmniPlus)
- :ref:`SERVO1_FUNCTION <SERVO1_FUNCTION>` = 33 (motor1)
- :ref:`SERVO2_FUNCTION <SERVO2_FUNCTION>` = 34 (motor2)
- :ref:`SERVO3_FUNCTION <SERVO3_FUNCTION>` = 35 (motor3)
- :ref:`SERVO4_FUNCTION <SERVO4_FUNCTION>` = 36 (motor4)

Wheel Encoders
--------------

Up to two wheel encoders can be connected to the autopilot as described on the :ref:`Optional Hardware / Wheel Encoders <wheel-encoder>` page.

.. image:: ../../../images/wheel-encoder-pixhawk.png
    :target: ../_images/wheel-encoder-pixhawk.png
