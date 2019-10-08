.. _hold-mode:

=========
Hold Mode
=========

In Hold mode the vehicle should stop and for regular steering-throttle rovers, the steering will point straight ahead.

This is a good mode to :ref:`Arm and Disarm <arming-your-rover>` the vehicle in because the transmitter inputs required to arm/disarm will not affect the steering or motors.

The various :ref:`Failsafe <rover-failsafes>` features are often setup to switch the vehicle into this mode.

The output to the :ref:`servo and motors <rover-motor-and-servo-configuration>` will be the values held in the SERVOx_TRIM parameters (i.e. :ref:`SERVO1_TRIM <SERVO1_TRIM>`, :ref:`SERVO3_TRIM <SERVO3_TRIM>`)
