==============
AIS Tracking
==============

.. code:: bash

    module load ais
    
The adsb module takes in any AIS data from the autopilot (via the ``AIS_VESSEL`` MAVLink message)
and shows any nearby ships on the map.

Settings
========

The configurable settings for this module can be controlled by:

.. code:: bash

    ais set <setting> <value>
    
The settings are:

===============================   ==========================================   ===============================
Setting                           Description                                  Default
===============================   ==========================================   ===============================
timeout                           Timeout (sec) if an ship is no longer seen   60
threat_radius                     Radius (m) to consider an ship a threat      200
===============================   ==========================================   ===============================



