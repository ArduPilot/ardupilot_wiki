=========================
Gas Helicopter Management
=========================

.. code:: bash

    module load gasheli
    
The gesheli module provides handling for the starting and
stopping of gas heli engines that have ignition and starter
motor control.

It requires the ignition control on a RC channel and the 
starter motor on a RC channel.

Start/Stop
==========

.. code:: bash

    gasheli start
    
Begin the start sequence. The starter motor will engage, then the 
ignition will switch on after ``ignition_disable_time`` sec.

The starter motor will stop after ``starter_time`` sec.

.. code:: bash

    gasheli stop
    
Stop the engine. It does this by pulling the ignition channel low. After ``ignition_stop_time`` 
sec it hands back control to the RC transmitter.

Settings
========

The configurable settings for this module can be controlled by:

.. code:: bash

    gasheli set <setting> <value>
    
The settings are:

===============================   =======================================   ===============================
Setting                           Description                               Default
===============================   =======================================   ===============================
ignition_chan                     Ignition RC Channel                       0
ignition_disable_time             How long to wait after starter motor      0.5
ignition_stop_time                How long to have ignition override off    3
starter_chan                      Starter RC Channel                        0
starter_time                      How long to leave starter on              3
starter_pwm_on                    Starter PWM level when on                 2000
starter_pwm_off                   Starter PWM level when off                1000
===============================   =======================================   ===============================

