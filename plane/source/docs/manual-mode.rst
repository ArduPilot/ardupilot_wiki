.. _manual-mode:

===========
MANUAL Mode
===========

Regular RC control, no stabilization. All RC inputs are passed through
to the servo outputs set by their SERVOx_FUNCTION.

.. note:: This assumes that the RC channel is being used as a servo output function instead of one selected by its RCx_OPTION parameter. The only ways in which the input to a servo output function may be different from inputs in this mode is if a configured failsafe or geofence triggers, and Plane takes control.
