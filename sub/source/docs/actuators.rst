.. _actuators:

===============
Actuators (PWM)
===============

A generic PWM-controlled device can be added by connecting it to a pin on the autopilot's servo rail for use as a button-controlled function, and assigning that pin's ``SERVOx_FUNCTION`` parameter as one of ``Actuator 1..6`` as desired. Limits and direction are configured using the pin's ``SERVOx_MIN``/``MAX``/``REVERSED`` parameters.

.. note: Where possible it is preferable to use dedicated output functions for the device type (e.g. :ref:`lights`, :ref:`grippers`, etc). Actuators serve as a convenience feature for controlling devices that have no dedicated configuration options, or where the options are insufficient for the device being controlled.


Joystick/Gamepad Control
========================

Actuator outputs can be controlled using the ``actuator_n_*`` :ref:`Button Functions <buttons>`, with options for:

.. csv-table:: Actuator Button Functions
   :header: "Function", "Description"

   "``actuator_n_min``", "Set the servo output channel assigned to Actuator N to its minimum value"
   "``actuator_n_max``", "Set the servo output channel assigned to Actuator N to its maximum value"
   "``actuator_n_center``", "Set the servo output channel assigned to Actuator N to its center value"
   "``actuator_n_inc``", "Increase Actuator N by the ``ACTUATORn_INC`` parameter value"
   "``actuator_n_dec``", "Decrease Actuator N by the ``ACTUATORn_INC`` parameter value"
   "``actuator_n_min_momentary``", "Set Actuator N to its minimum value while the button is held, then to its center value on release"
   "``actuator_n_max_momentary``", "Set Actuator N to its maximum value while the button is held, then to its center value on release"
   "``actuator_n_min_toggle``", "Toggle Actuator N between its minimum and center values"
   "``actuator_n_max_toggle``", "Toggle Actuator N between its maximum and center values"


Programmatic Control
====================

Support for MAVLink-based control (via `MAV_CMD_DO_SET_ACTUATOR <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ACTUATOR>`__) is `underway <https://github.com/ArduPilot/ardupilot/pull/32033>`__, but not yet available.

Lua script control is not supported.
