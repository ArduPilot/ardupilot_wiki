.. _booster-motor:

=============
Booster Motor
=============

..  youtube:: RjjF_S69Ywk
    :width: 100%

Copter 3.6 (and higher) and QuadPlanes support additional motors placed in the middle of the frame to provide additional lift.  This feature is compatible with any of the multicopter frame types (i.e. quad, tri, hexa, octa, dodecahexa).  The video above shows this feature used along with an :ref:`internal combustion engine <common-ice>` but the feature can also be used with electric motors

.. note::

   Copter does not yet support horizontally facing motors to provide movement without leaning the vehicle (`enhancement request is here <https://github.com/ArduPilot/ardupilot/issues/10117>`__)

Configuration
-------------

-  Connect the booster motor's ESCs to one of the autopilot's RC Output ports (i.e. MAIN OUT 1 ~ 8 or AUX OUT 1 ~ 6)
-  Set SERVOx_FUNCTION = 81 for "Boost Throttle" (where "x" is the servo output number)
-  Optionally set :ref:`MOT_BOOST_SCALE <MOT_BOOST_SCALE>` to a value between 0 to 5 to scale the output for the booster based on the average output throttle sent to the standard motors.  A higher scaling factor will put more of the load on the booster motor.  1 will set the BoostThrottle equal to the main throttle.  The output to the main motor will always be between the ``SERVOx_MIN`` and ``SERVOx_MAX`` values (where "x" is the servo output number)
