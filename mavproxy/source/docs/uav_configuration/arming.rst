====================
Arming and Disarming
====================

The UAV can be remotely armed and disarmed.

Arming
======

The autopilot can be armed by:

.. code:: bash

    arm throttle
    
The autopilot can be force armed by:

.. code:: bash

    arm throttle force

Disarming
=========

The autopilot can be disarmed by:

.. code:: bash

    disarm

The autopilot can be force disarmed by:

.. code:: bash

    disarm force

Safety Switch
=============

The safety switch can be remotely turned on and off via:

.. code:: bash

    arm safetyon
    arm safetyoff

Arming Checks
=============

The individual arming checks can be enabled or disabled as per:

.. code:: bash

    arm check X
    arm uncheck X
    
Where X can be: all, baro, compass, gps, ins, params, rc, voltage, battery, airspeed, 
logging, switch, gps_config, mission, rangefinder, system

