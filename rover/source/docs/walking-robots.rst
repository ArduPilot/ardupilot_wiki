.. _walking-robots:

==============
Walking Robots
==============

ArduPilot Rover 4.1 (and higher) includes basic support for four legged walking robots.  More details can be found in `this GSoC 2020 blog post <https://discuss.ardupilot.org/t/gsoc-2020-walking-robot-support-for-ardupilot/57080>`__

..  youtube:: TrwFPPqd1b8
    :width: 100%

.. warning::

    This page is still a work-in-progress

Hardware Required
-----------------

- 1x `Lynxmotion Phoenix 3DOF Hexapod frame <https://www.robotshop.com/jp/en/lynxmotion-phoenix-3dof-hexapod---black-no-servos---electronics.html>`__ (only 4 legs will be attached)
- 12x `Hitec HS-645MG Servo <https://www.robotshop.com/en/hitec-hs-645mg-servo-motor.html>`__
- 1x 2200mAh 2S lipo battery
- 1x `15Amp BEC <https://hobbyking.com/en_us/turnigy-8-15a-ubec-for-lipoly.html>`__
- :ref:`ArduPilot compatible autopilot <common-autopilots>` with at least 12 PWM outputs and ideally with the powerful STM32H7 CPU to give enough memory to run :ref:`Lua scripts <common-lua-scripts>` easily

Hardware Setup
--------------

Connect the AutoPilot's pwm outputs to each servo as listed below:

- Output1: front right coxa (hip) servo
- Output2: front right femur (thigh) servo
- Output3: front right tibia (shin) servo
- Output4: front left coxa (hip) servo
- Output5: front left femur (thigh) servo
- Output6: front left tibia (shin) servo
- Output7: back right coxa (hip) servo
- Output8: back right femur (thigh) servo
- Output9: back right tibia (shin) servo
- Output10: back left coxa (hip) servo
- Output11: back left femur (thigh) servo
- Output12: back left tibia (shin) servo

Software Configuration
----------------------

- Use a ground station to load Rover-4.1 (or higher) to the autopilot
- Connect with a ground station and set :ref:`SCR_ENABLE <SCR_ENABLE>` = 1 to enable Lua scripting and reboot the autopilot
- Download `quadruped.lua <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples>`__ from the ArduPilot Github repo to your PC
- Load the quadruped.lua script to the autopilot using MAVFTP or by directly copying to the SD Card's APM/scripts directory (`see video <https://youtu.be/3n80dYoJQ60?t=71>`__)


