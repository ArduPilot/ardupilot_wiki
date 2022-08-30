.. _crash-detect:

Crash Detection
===============

Plane provides a means to disarm the motors if a crash is detected. This is provided only in AUTO mode and checks for sudden deceleration in the X axis (front/back).

Detection Threshold
-------------------

:ref:`CRASH_ACC_THRESH<CRASH_ACC_THRESH>` set the X-Axis deceleration threshold to notify the crash detector that there was a possible impact which helps disarm the motor quickly after a crash. This value should be much higher than normal negative x-axis forces during normal flight, check flight log files to determine the average IMU.x values for your aircraft and motor type. Higher value means less sensitive (triggers on higher impact). For electric planes that don’t vibrate much during fight a value of 25 is good (that’s about 2.5G). For petrol/nitro planes you’ll want a higher value. Set to 0 to disable the collision detector.

Enabling the Detection
----------------------

Setting bit 1 of the :ref:`CRASH_DETECT<CRASH_DETECT>` parameter enables disarming the Plane when :ref:`CRASH_ACC_THRESH<CRASH_ACC_THRESH>` has been exceeded.