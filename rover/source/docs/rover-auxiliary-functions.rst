.. _rover-auxiliary-functions:

===================
Auxiliary Functions
===================

This page describes how to set up additional features which can be invoked from the transmitter's auxiliary function switches.

Configuring which transmitter channel is used
=============================================

Any RC input channel can be assigned any auxiliary function. However the RC channel should not be used by any other 
function. Typically channels above 7 are free to be used for auxiliary functions.

Assigning the Feature
=====================

The :ref:`RC7_OPTION <RC7_OPTION>` parameter controls which feature is assigned to RC input 7. Each RC channel has its 
own RCx_OPTION parameter that can be accessed in the full parameter list.

If using the Mission Planner, the Config/Tuning >> Basic Tuning's "RC7 Opt", "RC8 Opt, "RC9 Opt" and "RC10 Opt" drop-down can be used to easily set the desired feature.

.. image:: ../images/rover-auxiliary-functions-MP.png
    :target: ../_images/rover-auxiliary-functions-MP.png

Supported Features
==================

+----------------------+----------------------------+
| **RCx_OPTION value** | **Feature Description**    |
+----------------------+----------------------------+
|        0             | Do Nothing                 |
+----------------------+----------------------------+
|        4             | RTL mode                   |
+----------------------+----------------------------+
|        7             | Save WP                    |
+----------------------+----------------------------+
|        9             | Camera Trigger             |
+----------------------+----------------------------+
|        16            | Auto mode                  |
+----------------------+----------------------------+
|        19            | Gripper                    |
+----------------------+----------------------------+
|        28            | Relay 1 On/off             |
+----------------------+----------------------------+
|        30            | Lost Rover Sound           |
+----------------------+----------------------------+
|        31            | Motor Emergency Stop       |
+----------------------+----------------------------+
|        34            | Relay 2 On/Off             |
+----------------------+----------------------------+
|        35            | Relay 3 On/Off             |
+----------------------+----------------------------+
|        36            | Relay 4 On/Off             |
+----------------------+----------------------------+
|        40            | Proximity Avoidance        |
+----------------------+----------------------------+
|        41            | Arm Disarm                 |
+----------------------+----------------------------+
|        42            | SmartRTL mode              |
+----------------------+----------------------------+
|        46            | RC Override Enable         |
+----------------------+----------------------------+
|        50            | LearnCruise                |
+----------------------+----------------------------+
|        51            | Manual mode                |
+----------------------+----------------------------+
|        52            | Acro mode                  |
+----------------------+----------------------------+
|        53            | Steering mode              |
+----------------------+----------------------------+
|        54            | Hold mode                  |
+----------------------+----------------------------+
|        55            | Guided mode                |
+----------------------+----------------------------+
|        56            | Loiter mode                |
+----------------------+----------------------------+
|        57            | Follow mode                |
+----------------------+----------------------------+
|        58            | Clear Waypoints            |
+----------------------+----------------------------+
|        59            | Simple mode                |
+----------------------+----------------------------+
|        62            | Compass Learn              |
+----------------------+----------------------------+
|        63            | Sailboat Tack              |
+----------------------+----------------------------+
|        65            | GPS Disable                |
+----------------------+----------------------------+
|        66            | Relay 5 On/Off             |
+----------------------+----------------------------+
|        67            | Relay 5 On/Off             |
+----------------------+----------------------------+

Check the channel range
=======================

.. image:: ../images/rover-aux-switch-check.png
    :target: ../_images/rover-aux-switch-check.png

The configured feature will be triggered when the auxiliary switch's pwm value becomes higher than 1800.  It will be deactivated when the value falls below 1200.

You can check the pwm value sent from the transmitter when the switch is high and low using the Mission Planner's Initial Setup >> Mandatory Hardware >> Radio Calibration screen.  If it does not climb higher than 1800 or lower than 1200, it is best to adjust the servo end points in the transmitter.