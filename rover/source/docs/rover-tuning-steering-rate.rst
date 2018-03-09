.. _rover-tuning-steering-rate:

====================
Tuning Steering Rate
====================

This page decribes how a Rover's Steering Rate controller (aka turn rate controller) can be tuned.  This is the lowest level controller but often the most important in order to achieve good steering control.

.. image:: ../images/rover-tuning-steering-rate.png
    :target: ../_images/rover-tuning-steering-rate.png

Steering Rate PID Tuning
------------------------

The Steering Rate controller attempts to achieve the desired turn rate (set by the pilot or autopilot) using a PID controller.  All modes except :ref:`Hold <hold-mode>` and :ref:`Manual <manual-mode>` use this controller.

The FF, P, I and D gains for this controller are held in the :ref:`ATC_STR_RAT_FF <ATC_STR_RAT_FF>`, :ref:`ATC_STR_RAT_P <ATC_STR_RAT_P>`, :ref:`ATC_STR_RAT_I <ATC_STR_RAT_I>` and :ref:`ATC_STR_RAT_D <ATC_STR_RAT_D>` parameters respectively.

Recommended steps for tuning this controller are:

- connect the ground station to the vehicle using a telemetry radio
- drive the vehicle in a mode that generates significant turns.  This could be :ref:`Acro <acro-mode>` mode (first set the :ref:`ACRO_TURN_RATE <ACRO_TURN_RATE>` parameter), :ref:`Steering <steering-mode>` or :ref:`Auto <auto-mode>`.  If using the higher level modes (Steering, Auto, etc) note that the higher level L1 controller may obscure whether the steering rate controller is well tuned.
- tune the :ref:`FF gain <ATC_STR_RAT_FF>` first.  This gain converts the desired rotation rate directly into a steering servo or motor output.  If the vehicle's turn rate response is sluggish then this parameter should be increased.  If the vehicle is constantly overshooting its desired turn rate then this parameter should be reduced.
- the :ref:`P gain <ATC_STR_RAT_P>` corrects for short-term error.  It can often be left at zero or another very low value if the FF value is set well.  If set too high the rotation rate may oscillate.  This :ref:`P gain <ATC_STR_RAT_P>` should always be lower than the :ref:`FF gain <ATC_STR_RAT_FF>`.
- the :ref:`I gain <ATC_STR_RAT_I>` corrects for long-term error.  If the vehicle never achieves the desired turn rate, then this parameter should be increased.  If the vehicle's turn rate slowly oscillates then this parameter should be reduced.
- the :ref:`D gain <ATC_STR_RAT_D>` is meant to stablize the output by fighting against short-term changes in turn rate.  This gain can normally be left at zero.

The desired vs actual (aka achieved) turn rate, along with the individual contributions to the steering output from the FF, P, I and D components can be seen in real-time by doing the following:

- connect the Mission Planner to the vehicle's flight controller using a telemetry radio
- set the :ref:`GCS_PID_MASK <GCS_PID_MASK>` to 1 (Steering)
- on the Flight Data screen, check the "Tuning" checkbox (bottom middle) and select "pidachieved", "piddesired"
- change the desired turn rate of the vehicle (perhaps using the transmitter's steering stick) and see how well the "achieved" follows the "desired"

.. image:: ../images/rover-throttle-and-speed2.png
    :target: ../_images/rover-throttle-and-speed2.png
