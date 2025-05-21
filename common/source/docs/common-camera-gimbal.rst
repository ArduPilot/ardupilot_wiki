.. _common-camera-gimbal:

============
Servo Gimbal
============

.. image:: ../../../images/servo-gimbal.png
    :width: 450px

ArduPilot can stabilize a servo gimbal with up to three axis of motion using any of the free PWM output channels.
Once connected the camera gimbal can be controlled by the pilot using an RC transmitter, by sending commands from the ground stations or autonomously during missions

Supported Gimbals
=================

Any servo gimbal accepting PWM input should work.  Specific examples include

- Adafruit Mini Pan-Tilt Kit
- `Quanum Pan-Tilt Servo Gimbal <https://hobbyking.com/en_us/quanum-servo-based-pan-tilt.html>`__

Mounting the Camera and Gimbal
==============================

The camera needs to be mounted securely to the gimbal, but in such a way
that reduces/dampens vibrations from the motor.

Common methods for mounting the camera on the gimbal include using soft
foam, stiff foam, neoprene tubes (mount camera on tube side), surgical
tube, rubber bands, nylon bolts (direct stiff attachment) and velcro.

Connecting to the Autopilot
===========================

Connect the gimbal's roll, pitch, and/or yaw servos signal and ground pins to the autopilot's PWM output pins as shown below.  Most autopilots do not provide power on the servo rail meaning a separate BEC is required.

.. image:: ../../../images/pixhawk_to_gimbal_connection.jpg
    :target: ../_images/pixhawk_to_gimbal_connection.jpg

Configuration
=============

.. note::

   Mission Planner includes a "Camera Gimbal" configuration screen but it has not yet been updated to work with ArduPilot 4.3 (and higher).

Connect to the autopilot with a ground station and set the following parameters. These settings assume the autopilot's PWM outputs 9, 10 and 11 will control the gimbal's roll, pitch and yaw angles respectively. They also assume common angular ranges of the gimbal which may need adjusting to match the actual gimbal being used.

.. note:: Currently up to two mounts can be supported, MNT1 and MNT2. The following parameters are for the first mount. The second mount has the same parameters.

- :ref:`MNT1_TYPE <MNT1_TYPE>` to 1 (Servo) and reboot the autopilot
- :ref:`MNT1_PITCH_MIN <MNT1_PITCH_MIN>` to -90 (meaning the gimbal can pitch straight downwards)
- :ref:`MNT1_PITCH_MAX <MNT1_PITCH_MAX>` to 25 (meaning the gimbal can pitch up by 25 deg)
- :ref:`MNT1_ROLL_MIN <MNT1_ROLL_MIN>` to -30 (meaning the gimbal can roll right 30 deg)
- :ref:`MNT1_ROLL_MAX <MNT1_ROLL_MAX>` to 30 (meaning the gimbal can roll left 30 deg)
- :ref:`MNT1_YAW_MIN <MNT1_YAW_MIN>` to -180 (meaning the gimbal can yaw to the left 180deg)
- :ref:`MNT1_YAW_MAX <MNT1_YAW_MAX>` to 180 (meaning the gimbal can yaw to the right 180deg)
- :ref:`MNT1_RC_RATE <MNT1_RC_RATE>` to 90 (deg/s) to control speed of gimbal when using RC targetting

Typical input and output assignments are shown below, but any unused RC input channel or autopilot output channels can be assigned for some or all of these functions.

- :ref:`SERVO9_FUNCTION <SERVO9_FUNCTION>` to 8 (Mount1 Roll)
- :ref:`SERVO9_MIN <SERVO9_MIN>` and :ref:`SERVO9_MAX <SERVO9_MAX>` to match the min and max range of the roll servo
- :ref:`SERVO10_FUNCTION <SERVO10_FUNCTION>` to 7 (Mount1 Pitch)
- :ref:`SERVO10_MIN <SERVO10_MIN>` and :ref:`SERVO10_MAX <SERVO10_MAX>` to match the min and max range of the pitch servo
- :ref:`SERVO11_FUNCTION <SERVO11_FUNCTION>` to 6 (Mount1 Yaw)
- :ref:`SERVO11_MIN <SERVO11_MIN>` and :ref:`SERVO11_MAX <SERVO11_MAX>` to match the min and max range of the yaw servo
- :ref:`RC6_OPTION <RC6_OPTION>` = 213 ("Mount Pitch") to control the gimbal's pitch rate with RC channel 6
- :ref:`RC7_OPTION <RC7_OPTION>` = 214 ("Mount Yaw") to control the gimbal's yaw rate with RC channel 7
- :ref:`RC8_OPTION <RC8_OPTION>` = 163 ("Mount Lock") to switch between "lock" and "follow" mode with RC channel 8

Control and Testing
===================

See :ref:`Gimbal / Mount Controls <common-mount-targeting>` for details on how to control the gimbal using RC, GCS or Auto mode mission commands


Shutter configuration
=====================

See :ref:`Camera Shutter Configuration in Mission Planner <common-camera-shutter-with-servo>` for information on how to integrate shutter triggering with ArduPilot.

See :ref:`common-cameras-and-gimbals` page for links to various triggering methods.

See :ref:`common-mount-targeting` for mount control and targeting information.


Other Parameters
================

Since servos in the gimbal may react slower to position/angle changes in the vehicle's roll and pitch as the vehicle moves about a target, the camera shot may have some visible lag in it. This can be reduced by using these parameters to have the gimbal outputs move a bit ahead of the movements of the vehicle.


- :ref:`MNT1_LEAD_RLL<MNT1_LEAD_RLL>` 
- :ref:`MNT1_LEAD_PTCH<MNT1_LEAD_PTCH>`
