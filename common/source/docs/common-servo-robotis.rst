.. _common-servo-robotis:

==============
Robotis Servos
==============

.. image:: ../../../images/servo-robotis.jpg

Robotis Dynamixel servos are relatively expensive but high-end "smart actuators" that can be controlled by ArduPilot once connected to the autopilot's serial port.

Robotis servos using the "`Robotis Protocol version 2.0 <http://emanual.robotis.com/docs/en/dxl/protocol2/>`__" are supported.  Look for "Half duplex Asynchronous Serial Communication" in the "H/W SPECS" section of each servo.  The `XM430-W350-T <http://www.robotis-shop-en.com/?act=shop_en.goods_view&GS=2923&keyword=XM430-W350-T>`__ in particular is known to work correctly with ArduPilot.

.. note::

   Support for these servos is available in Copter-4.0, Plane-3.10 and Rover-3.5 (or higher)

Where to Buy
------------

- `Robotis Shop's <http://www.robotis-shop-en.com/?act=main_en>`__ Actuator section
- 1x `U2D2 <http://www.robotis-shop-en.com/?act=shop_en.goods_view&GS=3288&keyword=U2D2>`__ is also recommended to allow easy configuration of the servos (`online manual <http://emanual.robotis.com/docs/en/parts/interface/u2d2/>`__)

Configuring the Servos
----------------------

The ID for each servo needs to be set using the R+ Manager configuration tool.

`DYNAMIXEL Wizard 2.0 <http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/>`__ is the latest DYNAMIXEL configuration tool that supports Windows / Linux / Mac OSX.

- Open the `robotis.us/roboplus2 <http://www.robotis.us/roboplus2/>`__ website, in the "R+ Manager 2.0" row, download and install the "2.0.1 Windows (exe)"
- Attach the U2D2 to the servos as described in the `online manual <http://emanual.robotis.com/docs/en/parts/interface/u2d2/>`__
- Start the R+ Manager, connect to the appropriate COM port and set each servo's ID to a value from 1 to 16.  The number chosen for each servo should correspond to the servo number used to configure and control the servo.  For example if the ID is set to "9", the :ref:`SERVO9_MIN <SERVO9_MIN>`, :ref:`SERVO9_MAX <SERVO9_MAX>`, :ref:`SERVO9_FUNCTION <SERVO9_FUNCTION>`, etc parameters will be used to configure the servo.

.. image:: ../../../images/servo-robotis-rmanager.png
    :target: ../_images/servo-robotis-rmanager.png
    :width: 450px

Connecting and Configuring
--------------------------

.. image:: ../../../images/servo-robotis-pixhawk.jpg
    :target: ../_images/servo-robotis-pixhawk.jpg

-  Connect one of the servos to any serial port on the autopilot.  In this example SERIAL4 is used but any serial port should work
-  Additional servos should be connected to the first by daisy chaining them as shown above
-  Set the following parameters on the autopilot

  - :ref:`SERIAL4_PROCOTOL <SERIAL4_PROTOCOL>` = 19 ("RobotisServo")
  - :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` = 57 (57600 baud)
  - :ref:`SERIAL4_OPTIONS <SERIAL4_OPTIONS>` = 4 to enable half-duplex mode

Testing with the Mission Planner
================================

The mission planner's Flight Data screen includes a "Servo" tab on the
bottom right that can be used to test that the servos are moving
correctly.

.. image:: ../../../images/Servo_TestingWithMP.jpg
    :target: ../_images/Servo_TestingWithMP.jpg
    :width: 450px
