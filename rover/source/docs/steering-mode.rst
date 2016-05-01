.. _steering-mode:

=============
STEERING Mode
=============

Learning mode (sort of) plus obstacle avoidance.

-  Steering mode is a bit like learning mode, except that the controls
   use the same steering, throttle and obstacle avoidance code as AUTO
   mode does.

   -  The steering controls the "navigation bearing", which is what AUTO
      uses to navigate.
   -  The throttle controls the target speed, just like AUTO does.
   -  When it sees an obstacle it tries to avoid it in the same way that
      AUTO does.
   -  It even does turn speed scaling.

-  So you can use STEERING mode to test your rover while remaining fully
   in control.
-  Drive it at a obstacle, and it will avoid it, which is not only fun,
   it's great for tuning.
-  Momentary toggle (CH7) stores the current GPS position as a new
   waypoint.
