.. _balance_bot-tuning:

======================
Tuning the Balance Bot
======================

Tuning Manual/Hold Mode
=======================
Manual and Hold Modes use a PID controller to control pitch angle. This section describes the tuning process for the Pitch Controller. The parameters involved in tuning Manual/Hold Modes are:

- :ref:`BAL_PITCH_MAX<BAL_PITCH_MAX>` : Maximum pitch angle limit
- :ref:`BAL_PITCH_TRIM <BAL_PITCH_TRIM>` : Pitch trim to offset tilt of center of mass
- :ref:`ATC_BAL_P <ATC_BAL_P>` : P Gain
- :ref:`ATC_BAL_I <ATC_BAL_I>` : I Gain
- :ref:`ATC_BAL_D <ATC_BAL_D>` : D Gain

**1) Configure PID tuning plot** :
The tuning process will be easier when viewing the PID tuning plot. To select the Pitch Control PID for plotting, set:

- :ref:`GCS_PID_MASK <GCS_PID_MASK>` = 4 

Now, the plot can be viewed by selecting PID_Tuning from Misssion Planner.

**2) PID tuning with no RC input** :
Keep the vehicle upright and then arm it in Manual Mode. 
Tune the P, I and D gains so that the vehicle can stay upright. The vehicle may show tendency to drift off in either direction, but still must be able to hold on to zero degree pitch. This can be checked on the PID tuning plot.

.. warning:: Give throttle input for short durations only. Otherwise the vehicle will reach full speed and become unable to balance. Pull back throttle stick to stop the vehicle.

**3) PID tuning with RC input** :
Provide throttle input on the RC transmitter and see if the vehicle is able to follow the desired pitch angle, in the PID tuning plot.  Tweak the PID gains so that the vehicle is able to follow the desired pitch angle. Good tuning of the pitch controller is crucial for tuning other Control Modes.

.. _balance_bot-tuning-pitch-trim:

**4) Adjust pitch trim** :
Sometimes the vehicle may show tendency to drift off in one direction while upright. This is because the center of mass may be slightly off the vertical axis at zero pitch. Tweaking the :ref:`BAL_PITCH_TRIM <BAL_PITCH_TRIM>` parameter can offset this. This parameter sets the angle for zero-pitch or upright position. Change it slowly in steps of 0.1 till the vehicle doesnt drift anymore.

.. _balance_bot-tuning-acro:

Tuning Acro Mode
================
The following parameters must be set as specified below:

- :ref:`CRUISE_THROTTLE <CRUISE_THROTTLE>` = 0 (This parameter can cause conflicts otherwise, in Balance Bots)
- :ref:`CRUISE_SPEED <CRUISE_SPEED>` : Maximum speed limit for Speed Controller. To be set by trial and error


With the above parameters set, follow the tuning guides for :ref:`Speed <rover-tuning-throttle-and-speed>` and :ref:`Steering<rover-tuning-steering-rate>`. Another important thing to note is that the feed-forward term described in these pages does not make any positive contribution in a balance bot and is best, set to zero.

Guided, Auto, RTL
=================
To change mode to Acro after an Auto mission is completed set

- :ref:`MIS_DONE_BEHAVE <MIS_DONE_BEHAVE>` =2 

By default it switches to Hold Mode, which can cause the vehicle to crash.

