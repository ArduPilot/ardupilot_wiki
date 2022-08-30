===========
User Alerts
===========

.. code:: bash

    module load useralerts

This module checks the currently connected firmware (ArduPilot) against the
:ref:`User Alerts <dev:user-alerts-developer>` database and will notify if any User Alerts are applicable.

An Internet connection is required to run the module.

After loading, it can be run via:

.. code:: bash

    useralerts check


Settings
========

The configurable settings for this module can be controlled by:

.. code:: bash

    useralerts set <setting> <value>
    
The settings are:

===============================   ========================================   ===============================
Setting                           Description                                Default
===============================   ========================================   ===============================
useTest                           Use the example User Alerts database       False
filterBoard                       Filter the User Alerts by board hardware   True
===============================   ========================================   ===============================

