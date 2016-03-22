.. _differential-spoilers:

=====================
Differential Spoilers
=====================

Usage
=====

Normal Flying-wing aircraft uses two control surfaces as Elevons to
control pitch and roll. In some cases, rudders are added to the winglets
to control yaw. Differential spoiler takes advantage of splitting
elevons to 4 independent control surfaces: normal elevon functions are
reserved for pitch and roll control, but yaw control is done by using
two surfaces on one side of the wing to create drag force thus
controlling yaw motion. If calibrated correctly, it will ensure pilot
has smooth yaw control during take-off and landing as well as
compensation during turning (similar to differential aileron).

+-----------------------------------------------+-------------------------------------------------+
| **Left Yaw by inducing drag on left wing:**   | **Right Yaw by inducing drag on right wing:**   |
+-----------------------------------------------+-------------------------------------------------+
| |4surfLeftYaw|                                | |4surfRightYaw|                                 |
+-----------------------------------------------+-------------------------------------------------+

Preparation
===========

To use differential spoiler function, the airframe is required to use
have 4 control surfaces (2 on each wing) and use elevon configuration.
Setup one surface on each side using output channel 1 (roll) and channel
2 (pitch) as elevons:

If you have not done so, please follow the tutorial to setup elevon
properly: :ref:`Archived:APM2.x Wiring QuickStart <archived-apm2x-wiring-quickstart>`

Setup
=====

Once you have setup two elevon channels successfully, connect the
additional 2 surfaces to auxiliary channels (5 to 11 on APM2.X) and set
corresponding \ **RC_FUNCTION**:

+--------------------------+-----------------------+----------------------------------------------------------------------------------------------------------------------+
| **Function Name**        | **Function Number**   | **Description**                                                                                                      |
+--------------------------+-----------------------+----------------------------------------------------------------------------------------------------------------------+
| Differential Spoiler 1   | 16                    | This should be set to the control surface on the same side of elevon connected to output channel 1 (roll channel)    |
+--------------------------+-----------------------+----------------------------------------------------------------------------------------------------------------------+
| Differential Spoiler 2   | 17                    | This should be set to the control surface on the same side of elevon connected to output channel 2 (pitch channel)   |
+--------------------------+-----------------------+----------------------------------------------------------------------------------------------------------------------+

By now, you should have normal elevon function in MANUAL mode. Now
switch to FBW mode and use RC controller to test roll, pitch and yaw.
Adjust corresponding \ ``RC_REV`` until the behavior is correct.

.. note::

   If you have servo direction reversed on either side of the wing,
   it is most likely that the servo orientations are different or rotation
   is reversed through programming on some digital servos

.. |4surfLeftYaw| image:: ../../../images/4surfLeftYaw.jpg
    :target: ../_images/4surfLeftYaw.jpg

.. |4surfRightYaw| image:: ../../../images/4surfRightYaw.jpg
    :target: ../_images/4surfRightYaw.jpg