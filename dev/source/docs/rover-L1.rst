.. _rover-L1:

===================================
Rover: waypoint navigation overview
===================================

This page provides an overview of how Rover drives from one waypoint to the
next in the autonomous modes (Auto, Guided, RTL and SmartRTL).

For the user-facing behaviour of S-Curves and the position controller, and for
how to tune them, see :ref:`Tuning Navigation <rover:rover-tuning-navigation>`.

Libraries involved
------------------

- `AR_WPNav <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AR_WPNav>`__
  plans the path to the destination and outputs a desired speed and turn rate
- `AR_WPNav_OA <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AR_WPNav/AR_WPNav_OA.cpp>`__ extends AR_WPNav with :ref:`object avoidance <common-object-avoidance-landing-page>`
  path planning, and is the class Rover actually instantiates
- `AR_PivotTurn <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AR_WPNav/AR_PivotTurn.cpp>`__ handles pivot (point) turns for vehicles that can turn on the spot
- `AR_AttitudeControl <https://github.com/ArduPilot/ardupilot/tree/master/libraries/APM_Control>`__
  holds the lower level speed and turn rate controllers
- `AP_MotorsUGV <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AR_Motors>`__
  converts the steering and throttle demands into motor outputs

Code flow
---------

- On every iteration of the main loop a call is made to the active mode's
  ``update()`` method. While in Auto, Guided, RTL and SmartRTL, this calls into
  the Mode class's ``navigate_to_waypoint()`` method in
  `Rover/mode.cpp <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.cpp>`__.

- ``navigate_to_waypoint()`` applies the pilot's speed nudge and then calls
  ``AR_WPNav::update()``.

- ``AR_WPNav::update()`` advances the target along the path and then updates the
  position controller, which produces a desired speed and a desired turn rate.

- The desired speed is passed to ``calc_throttle()``, which uses
  AR_AttitudeControl's speed controller to produce a throttle output.

- The desired turn rate is passed to ``calc_steering_from_turn_rate()``, which
  uses AR_AttitudeControl's ``get_steering_out_rate()`` to produce a steering
  output.

- Both outputs are sent to AP_MotorsUGV using ``set_throttle()`` and
  ``set_steering()``.

Sailboats are a special case: when tacking upwind the desired heading comes from
the Sailboat library and steering is calculated with ``calc_steering_to_heading()``
instead of the turn rate above.

The two navigation controller types
-----------------------------------

AR_WPNav can advance its target in one of two ways, selected by which method the
vehicle code used to set the destination:

- **S-Curve** (``NAV_SCURVE``) is used when the destination is set with
  ``set_desired_location()``, as Auto, RTL and SmartRTL do. The path is built as
  a jerk-limited S-Curve, and successive legs are blended so the vehicle can
  round a corner without stopping. This is the normal waypoint case.

- **Position controller input shaping** (``NAV_PSC_INPUT_SHAPING``) is used when
  the destination is set with ``set_desired_location_expect_fast_update()``, for
  callers such as Guided that update the target rapidly. Here the target is fed
  straight into the position controller's input shaping rather than being planned
  as a curve ahead of time.

While a pivot turn is active, neither is advanced — the vehicle turns on the spot
first and then resumes following the path.

Parameters
----------

The path AR_WPNav produces is shaped by :ref:`WP_SPEED <rover:WP_SPEED>`,
:ref:`WP_RADIUS <rover:WP_RADIUS>`, :ref:`WP_ACCEL <rover:WP_ACCEL>` and
:ref:`WP_JERK <rover:WP_JERK>`, along with the vehicle's acceleration and turn
limits. :ref:`Tuning Navigation <rover:rover-tuning-navigation>` explains how
these interact and how to set them.
