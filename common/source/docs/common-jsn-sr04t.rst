===============================================
Waterproof JSN-SR04T Ultrasonic Distance Sensor
===============================================

The JSN-SR04T is an ultrasonic distance sensor with a range of 25 to 450 cm. The very short range makes it of limited use.

.. image:: ../../../images/JSN-SR04T.jpg
    :target: ../_images/JSN-SR04T.jpg


Connection to the autopilot
===========================

To setup as the first rangefinder. Reboot after setting parameters:

-  :ref:`RNGFND1_MAX_CM<RNGFND1_MAX_CM>` = "450" (i.e. 4.5m max range)
-  :ref:`RNGFND1_MIN_CM<RNGFND1_MIN_CM>` = "25" (i.e. 20cm min range)
-  :ref:`RNGFND1_STOP_PIN<RNGFND1_STOP_PIN>` = Enter GPIO number for pin attached to JSN-SR04T "Trigger" pin. For example, on PixHawk with ``BRD_PWM_COUNT`` = 4, AUX6 (GPIO 55) could be used here, and AUX5 (GPIO54) could be used below.
-  :ref:`RNGFND1_PIN<RNGFND1_PIN>` = Enter GPIO number for pin attached to JSN-SR04T "Echo" pin.
-  :ref:`RNGFND1_TYPE<RNGFND1_TYPE>` = “30" (HC-SR04 sonar)
-  :ref:`RNGFND1_ORIENT<RNGFND1_ORIENT>` = "25" (Downward facing) if used for altitude control.