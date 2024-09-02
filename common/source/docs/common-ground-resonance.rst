.. _common-ground-resonance:

=============================
Preventing Ground Oscillation
=============================

Often a perfectly tuned aircraft will have a tendency to shake of oscillate when landed or just before takeoff. This phenomenon is known as ground resonance oscillation and is caused by the attitude controller interacting with a flexible frame or flexible landing gear. Ardupilot allows you to reduce the gain of the attitude controllers in the roll, pitch and yaw axis to attenuate the PID loops while landed to remove this instability. This should be done with care as we do not want to compromise our roll and pitch attitude control performance as we takeoff any more than is absolutely necessary. We can be less conservative in the yaw axis as compromised yaw control in the first half second after takeoff does not pose a great risk.

The three parameters used to address ground resonance oscillation are :ref:`ATC_LAND_R_MULT<ATC_LAND_R_MULT>`, :ref:`ATC_LAND_P_MULT<ATC_LAND_P_MULT>`, and :ref:`ATC_LAND_Y_MULT<ATC_LAND_Y_MULT>`, which provide the ability to reduce the Rll,Pitch, and Yaw pid gains on the ground.

Evaluating Ground Resonance Oscillation
=======================================

#.Arm the throttle in Stabilize Mode
#.Lift the throttle to the point where the motors increase to MOT_SPIN_MIN
#.Observe the axis off oscillation,
#.Back off :ref:`ATC_LAND_Y_MULT<ATC_LAND_Y_MULT>` to 0.5
#.Revaluate Roll and Pitch oscillation for improvement
#.Back off axis where oscillation is observed in 25% steps until oscillation is manageable.

.. note:: Extreme care should be taken if :ref:`ATC_LAND_R_MULT<ATC_LAND_R_MULT>` or :ref:`ATC_LAND_P_MULT<ATC_LAND_P_MULT>` is reduced below 0.5.
