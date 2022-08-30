.. _savetrim:

==================
Save Steering Trim
==================

If the vehicle does not track straight in ``MANUAL`` mode with steering at neutral, you can either adjust the steering mechanics to correct, or use the ``RCx_OPTION`` =  5 (Save Trim) to correct. Using the steering channel's trim tab or sub-trim in the transmitter is not recommended (after RC calibration is completed), since it will appear as a constant pilot steering request in non-MANUAL modes.

Save Trim
~~~~~~~~~

Activating Save Trim on the channel with RCx_OPTION = 5, will instantly capture the current output of the Steering Channel servo (normally SERVO1) and store it as its SERVOx_TRIM value. This can be repeated by de-activating and re-activating the rc channel function. The output is saved only at the moment of activation. It can be used in any mode except ``LOITER`` or ``HOLD``.

.. tip:: This is best used in ``ACRO`` mode when the autopilot is maintaining heading. Autonomous modes like ``AUTO`` or ``RTL``, may capture the output during a maneuver to turn to a waypoint, crosstrack correct, or be loitering at the end, and store incorrect results. But it can be used if careful as to when its applied. It can also be used during ``MANUAL`` mode by trimming at the transmitter for straight track, activating, and then returning the trim to center.