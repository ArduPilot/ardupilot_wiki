.. _common-daiwa-winch:

[copywiki destination="copter"]
===========
Daiwa Winch
===========

.. image:: ../../../images/daiwa-winch.png
    :target: ../_images/daiwa-winch.png

The Daiwa winch is a high quality winch specially designed for drone delivery.  The winch includes a "thread end detector switch" that protects against pulling the line in too tightly and thus straining the device or breaking the thread.  The gripper includes a spring mechanism that automatically releases the package when it touches the ground.

- Weight: 630g
- Size: W:110mm H:82mm D:72mm
- Maximum Payload: 8kg
- Maximum Power: 55W
- Input Voltage: 7.2V ~ 22.2V
- Interface: 4xPWM or UART
- Designed and manufactured in Japan

.. note::

    ArduPilot does not yet include a special driver for this winch but it can be controlled using the autopilot's servo outputs which allows the winch to be operated from the pilot's transmitter and/or DO_SET_SERVO mission commands

Where To Buy
------------

- These winches can be purchased from the Japanese branch of Okaya (`japanese site <https://www.okaya.co.jp/>`__, `english site <https://www.okaya.co.jp/en/index.html>`__)

Connection and Configuration
----------------------------

Connect the winch to the autopilot as shown below

.. image:: ../../../images/daiwa-winch-pixhawk.png
    :target: ../_images/daiwa-winch-pixhawk.png
    :width: 400px

- A 5V BEC is required to provide power to the electronics within the winch
- A 7.2V to 22.2V power supply is required to power the motors
- Four separate 3-pin RC connectors can be used to control the winch.  Each has a different colour of cable tie on it

   - Winch control (which controls the upward or downward speed of the winch) has a red cable tie and can be connected to AuxOut 10 or any other PWM output
   - Clutch (which prevents the cable slipping) has a yellow cable tie and can be connected to AuxOut 9 or any other PWM output
   - Zero Reset (used to calibrate the winch's estimate of how much line remains) has a green cable tie and can be left disconnected
   - SBUS2 has a blue cable tie and can be left disconnected

Video
-----

..  youtube:: p4x97iomWZ0
    :width: 100%
