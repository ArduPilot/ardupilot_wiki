.. circle-mode:

===========
Circle Mode
===========

Circle will orbit a point located :ref:`CIRC_RADIUS<CIRC_RADIUS>` meters in front
of the vehicle when the mode is entered.

Setting the :ref:`CIRC_RADIUS<CIRC_RADIUS>` to zero will cause the rover to simply stay
in place.

The speed of the vehicle (in meters/second) is set by the
:ref:`CIRC_SPEED<CIRC_SPEED>` parameter. And the direction is set by the :ref:`CIRC_DIR<CIRC_DIR>` parameter, "0" is CW, "1" CCW.

Operation Notes
===============

- The target point moving around the circle does not slowdown if the vehicle cannot keep up. This means that a too high a :ref:`CIRC_RADIUS<CIRC_RADIUS>` parameter value will lead to the vehicle driving at full throttle inside the desired radius. 
- The radius, speed and direction parameters only take effect when the vehicle enters circle mode (or begins a LOITER_TURNS mission command) so users cannot easily change the radius or direction while in the mode. Instead they must exit and re-enter the mode. In missions, the speed can be changed using the DO_CHANGE_SPEED MAVlink or mission command.