.. _common-rangefinder-hcsr04:

=========================
HC-SR04 Sonar Rangefinder
=========================

The HC-SR04 Sonar is a very inexpensive, short range (up to 2m) range finder primarily designed for
indoor use but which has been successfully used outdoors on Copter. It does provide more consistent height control below 2m than many barometers.


.. image:: ../../../images/hcsr04.jpg

.. warning::
   
   ``RNGFNDx_MAX_CM`` must be set to a tested, appropriate value.  If ``RNGFNDx_MAX_CM`` is set to a value  greater than 2m, the autopilot will not respond correctly to the data provided.

Connection to the autopilot
===========================

Two :ref:`GPIOs <common-gpios>` are required for the Trigger pin (starts the sonar pulse) and Echo pin (indicates reception of the echo). These can be a PWM output if the :ref:`BRD_PWM_COUNT<BRD_PWM_COUNT>` is set such that two are available, or if the autopilot has dedicated GPIO outputs (see the autopilot's :ref:`description <common-autopilots>` or datasheet).

To setup as the first rangefinder. Reboot after setting parameters:

-  :ref:`RNGFND1_MAX_CM<RNGFND1_MAX_CM>` = "200" (i.e. 2m max range)
-  :ref:`RNGFND1_MIN_CM<RNGFND1_MIN_CM>` = "20" (i.e. 20cm min range)
-  :ref:`RNGFND1_STOP_PIN<RNGFND1_STOP_PIN>` = Enter GPIO number for pin attached to HC-SRO4 "Trigger" pin. For example, on PixHawk with :ref:`BRD_PWM_COUNT<BRD_PWM_COUNT>` = 4, AUX6 (GPIO 55) could be used here, and AUX5 (GPIO54) could be used below.
-  :ref:`RNGFND1_PIN<RNGFND1_PIN>` = Enter GPIO number for pin attached to HC-SRO4 "Echo" pin.
-  :ref:`RNGFND1_TYPE<RNGFND1_TYPE>` = “30" (HC-SR04 sonar)
-  :ref:`RNGFND1_ORIENT<RNGFND1_ORIENT>` = "25" (Downward facing) if used for altitude control.

.. _gy-us42:
 
Other Rangefinders Compatible with this Rangefinder Type
========================================================

In addition to the HC-SR04 sensor, this :ref:``RNGFNDx_TYPE`` = “30" (HC-SR04 sonar), can also be used with the inexpensive, but longer range, GY-US42, which is similar to the Maxbotix I2C sensor, but can be configured to not only use I2C (use the :ref:`RNGFND1_TYPE<RNGFND1_TYPE>` = “2"), but also, as an Echo/Trigger sonar, if the center solder pad is shorted to the "L" pad. The  This device has a maximum useful range of 4m, so set -  :ref:`RNGFND1_MAX_CM<RNGFND1_MAX_CM>` = "400".

Pinout:

- "CR" pin is either the SCL (I2C mode) or Trigger input
- "DT" pin is either the  SDA (I2C mode) or Echo output
  

.. image:: ../../../images/gy-us42.jpg