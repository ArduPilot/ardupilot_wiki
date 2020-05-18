==================
Battery Management
==================

.. code:: bash

    module load battery
    
The battery module provides warnings to the user if any of the APM power 
sources reach a critical level.

Status
======

Prints out the current power level of all connected power sources

.. code:: bash

    bat

Settings
========

The configurable settings for this module can be controlled by:

.. code:: bash

    battery set <setting> <value>
    
The settings are:

===============================   =======================================   ===============================
Setting                           Description                               Default
===============================   =======================================   ===============================
battwarn                          Battery Warning Time                      1
batwarncell                       Battery cell Warning level (V)            3.7
servowarn                         Servo voltage warning level (V)           4.3
vccwarn                           Vcc voltage warning level (V)             4.3
numcells                          Number of series cells (S-rating)         0
===============================   =======================================   ===============================



