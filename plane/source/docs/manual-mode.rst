.. _manual-mode:

===========
MANUAL Mode
===========

Regular RC control, no stabilization. All RC inputs are passed through
to the servo outputs set by their SERVOx_FUNCTION.

.. note:: the :ref:`MAN_EXPO_ROLL<MAN_EXPO_ROLL>`, :ref:`MAN_EXPO_PITCH<MAN_EXPO_PITCH>`, and :ref:`MAN_EXPO_RUDDER<MAN_EXPO_RUDDER>` parameters will apply exponential to the stick inputs, if non-zero, in this mode. This is for users with transmitters which do not provide this function and desire to "soften" stick feel around neutral.

.. note:: This assumes that the RC channel is being used as a servo output function instead of one selected by its RCx_OPTION parameter. The only ways in which the input to a servo output function may be different from inputs in this mode is if a configured failsafe or geofence triggers, and Plane takes control.
