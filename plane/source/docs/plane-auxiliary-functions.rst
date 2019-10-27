.. _plane-auxiliary-functions:

===================
Auxiliary Functions
===================

This page describes how to set up additional features which can be invoked from the transmitter's auxiliary function switches.

.. note::
   Support for this feature has been added to Plane with firmware version 4.0

Configuring which transmitter channel is used
=============================================

Any RC input channel can be assigned any auxiliary function. However the RC channel should not be used by any other function. Typically channels 5,6,7, and above 8 are free to be used for auxiliary functions.

Assigning the Feature
=====================

The RCx_OPTION parameters control these features. For example, the :ref:`RC7_OPTION <RC7_OPTION>` parameter controls which feature is assigned to RC input 7. Each RC channel has its
own RCx_OPTION parameter that can be accessed in the full parameter list.


Supported Features
==================

+----------------------+----------------------------+
| **RCx_OPTION value** | **Feature Description**    |
+----------------------+----------------------------+
|        0             | Do Nothing (default)       |
+----------------------+----------------------------+
|        4             | RTL mode                   |
+----------------------+----------------------------+
|        9             | Camera Trigger             |
+----------------------+----------------------------+
|        16            | Auto mode                  |
+----------------------+----------------------------+
|        24            | Reset Auto Mission to start|
+----------------------+----------------------------+
|        28            | Relay 1 On/off             |
+----------------------+----------------------------+
|        29            | Landing Gear               |
+----------------------+----------------------------+
|        30            | Lost Plane Sound           |
+----------------------+----------------------------+
|        31            | Motor Emergency Stop       |
+----------------------+----------------------------+
|        34            | Relay 2 On/Off             |
+----------------------+----------------------------+
|        35            | Relay 3 On/Off             |
+----------------------+----------------------------+
|        36            | Relay 4 On/Off             |
+----------------------+----------------------------+
|        41            | Arm Disarm                 |
+----------------------+----------------------------+
|        43            | Inverted Flight            |
+----------------------+----------------------------+
|        46            | RC Override Enable         |
+----------------------+----------------------------+
|        51            | Manual mode                |
+----------------------+----------------------------+
|        55            | Guided mode                |
+----------------------+----------------------------+
|        58            | Clear Waypoints            |
+----------------------+----------------------------+
|        62            | Compass Learn              |
+----------------------+----------------------------+
|        64            | Reverse Throttle           |
+----------------------+----------------------------+
|        65            | GPS Disable                |
+----------------------+----------------------------+
|        66            | Relay 5 On/Off             |
+----------------------+----------------------------+
|        67            | Relay 5 On/Off             |
+----------------------+----------------------------+
|        72            | Mode Circle                |
+----------------------+----------------------------+

Check the channel range
=======================

.. image:: ../images/plane-aux-switch-check.png
    :target: ../_images/plane-aux-switch-check.png

The configured feature will be triggered when the auxiliary switch's pwm value becomes higher than 1800.  It will be deactivated when the value falls below 1200.

You can check the pwm value sent from the transmitter when the switch is high and low using the Mission Planner's Initial Setup >> Mandatory Hardware >> Radio Calibration screen.  If it does not climb higher than 1800 or lower than 1200, it is best to adjust the servo end points in the transmitter.
