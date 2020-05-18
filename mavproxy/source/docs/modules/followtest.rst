==============
Follow Me Test
==============

.. code:: bash

    module load followtest
    
This module commands the vehicle to follow another vehicle. The other vehicle must 
be sending ``HOME_POSITION`` packets to MAVProxy.

Upon reaching the minimum distance ``radius`` to the other vehicle, it will loiter in
a circle around it.

This module will activate when it is loaded.

It can be disabled by setting ``disable_msg`` to True.

Settings
========

The configurable settings for this module can be controlled by:

.. code:: bash

    followtest set <setting> <value>
    
The settings are:

===============================   =======================================   ===============================
Setting                           Description                               Default
===============================   =======================================   ===============================
radius                            Min distance from the other vehicle       100
altitude                          Desired altitude                          50
speed                             Desired speed to approach at              10
type                              Flight mode to use                        guided
vehicle_throttle                  Amount of throttle to use (0-1)           0.5
disable_msg                       Disable follow-me                         False
===============================   =======================================   ===============================


