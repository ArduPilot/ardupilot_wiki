.. _throw-mode:

==========
Throw Mode
==========

This slightly dangerous flight mode allows the pilot to throw the vehicle into the air (or drop the vehicle) in order to start the motors.
Once in the air, this mode does not accept any input from the pilot.  This mode requires GPS.

.. warning::

   Use with caution!  It is dangerous to get close to an armed multicopter as is required to throw the vehicle.  It is recommended to takeoff normally instead of using throw mode whenever possible.

..  youtube:: JIPMpDJqdJ8
    :width: 100%

How To Use
==========

#. Disarm copter
#. Switch to throw mode
#. Check GPS light is green
#. Arm copter and listen for ready tune (if vehicle has a buzzer).  The motors will not spin by default.
#. Pick up the vehicle and throw it up and away from you (it must climb by 50cm/s and reach a total speed of 5m/s)
#. Once the vehicle has stopped, switch the flight mode to Loiter (or other mode) to retake manual control

The motors should start when the vehicle reaches the apex of it's trajectory.
After the motors start this flight mode will first try to control it's attitude (return to level and stop rotating), then stop descending and finally it will attempt to stop moving horizontally.

Settings
========
- :ref:`THROW_TYPE <THROW_TYPE>` : set to 0 if throwing the vehicle up, 1 if dropping the vehicle.  If dropping, drop from a height of at least 10m.
- :ref:`THROW_MOT_START <THROW_MOT_START>` : controls whether the motors will spin slowly or not at all while waiting for the throw (0 = stopped, 1 = spinning slowly).  The default is 0 (will not spin after arming).
- :ref:`THROW_NEXTMODE <THROW_NEXTMODE>` : the vehicle will switch into this flight mode after stopping (Auto, Guided, RTL, Land and Brake are support).  Set to "Throw" (the default) to simply remain in Throw mode and wait for the pilot to switch modes manually

..  youtube:: ZnEFcJx1qko
    :width: 100%

Log Analysis
============
During the throw, THRO messages are written to the :ref:`dataflash log <common-downloading-and-analyzing-data-logs-in-mission-planner>`.  These can be useful in diagnosing problems in case the motors failed to start as part of a throw.  The graph below shows a successful throw in which the overall velocity climbs above 5m/s and the vertical velocity is over 0.5m/s.

   .. image:: ../images/throw_log.png
       :target: ../_images/throw_log.png
       :width: 500px
