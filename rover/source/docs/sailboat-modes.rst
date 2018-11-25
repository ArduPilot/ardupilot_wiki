.. _sailboat-modes: 

=====
Modes
=====

This outlines the differences in sailboat behaviour from the standard Rover :ref:`modes <rover-control-modes>`.

Manual
------
Sail position is directly controlled by throttle position. Minimum throttle is sail fully in, maximum throttle 
is sail fully released. You may find the controller disarms when tacking to the left travelling upwind with the 
sails tight. If this is an issue disable :ref:`stick disarming <ARMING_RUDDER>`.

ACRO
----
The Sail is automatically trimmed to the wind direction using the wind vane. A tack can be triggered from an aux
switch; the vehicle will match its current angle to the true wind on the new tack.

Loiter
------
..  youtube:: NCUF66rQXFg
    :width: 75%

The vehicle will keep moving within the :ref:`loiter radius <LOIT_RADIUS>`.


RLT
---
The vehicle will tack upwind back to the home location if required.

AUTO
----
..  youtube:: zoNLZ-xE-_0
    :width: 75%

The vehicle will tack upwind to reach the next waypoint if required. Note that if the vehicle has to tack it will
not stay on the line between waypoints. While traveling upwind a tack can be triggered from an aux switch or by a
maximum cross track error defined by :ref:`WP_OVERSHOOT <WP_OVERSHOOT>`.
