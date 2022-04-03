.. _tailsitter-tuning-guide:

======================
Tailsitter VTOL Tuning
======================

Tuning a tailsitter is different than tuning a normal STL (Separate Thrust for Lift) or Tilt-Rotor Quadplane. Those QuadPlanes tune very similarly to a Multirotor since the attitude in VTOL is controlled by motor speed/thrust in all axes (the exception being YAW in vectored yaw tilt rotor QuadPlanes).

In most tailsitters, VTOL attitude is usually controlled by some combination of fixed wing control surfaces and, in some configuratons, motor tilt for pitch and yaw. Roll is usually controlled by motor/speed thrust and can be tuned, and even AutoTuned, like a multicopter and follows the normal :ref:`QuadPlane<quadplane-vtol-tuning>` tuning for that axis.

Pitch and Yaw Tuning
====================

For pitch and yaw, control is provided by the fixed wing control surfaces, sometimes in conjunction with motor tilt, depending on type of tailsitter. These axes required a tuning approach more similar to normal fixed wing manual tuning, since the FF (Feed-Forward) component of the PID loop is the primary control path, with P and D PID terms providing disturbance corrections. The tuning process is as follows, starting from the default PID values:

- First, follow Steps 1 - 4 of the normal :ref:`QuadPlane Tuning Process Setup<quadplane-vtol-tuning-process>`.
- Normally, the vehicle is stable enough with the default PIDs to do a first test hover in QSTABILIZE or QHOVER. Do a short hover flight with logging enabled such that the :ref:`Harmonic Notch Filters<common-imu-notch-filtering>` can be setup. Its important to remove as much motor noise as possible in order to get a good tune using a throttle based harmonic notch filter.
- Once the notch filters are set, do another hover flight but briefly do short, but sharp pitch stick movements in both directions. Do not endanger the vehicle, but try to get several short full stick movements to the front and to the back. Do the same for yaw, but you should be able to hold full yaw stick each direction for a second or so, returning to neutral for a second in-between direction reversals.
- Download the :ref:`dataflash log<common-downloading-and-analyzing-data-logs-in-mission-planner>` for analysis. In order to determine the :ref:`Q_A_RAT_PIT_FF<Q_A_RAT_PIT_FF>` and :ref:`Q_A_RAT_YAW_FF<Q_A_RAT_YAW_FF>` terms, use a log analyzer like https://plot.ardupilot.org and setup the following plots on the same scale, using the "Add Expression" button:

.. code:: bash

   PIQP_P+PIQP_P+PIQP_D 
   X * (PIQP_Act) , where X is 0.2 to start

- Next adjust the X value above until the magnitudes of both plots are about equal. This will now be value for the :ref:`Q_A_RAT_PIT_FF<Q_A_RAT_PIT_FF>` term. Set the :ref:`Q_A_RAT_PIT_I<Q_A_RAT_PIT_I>` term to equal this.
- Now do the same for the YAW axis using:

.. code:: bash

   PIQY_P+PIQY_P+PIQY_D   
   X * (PIQP_Act) , where X is 0.2 to start

Below is an image of this where the X value is adjusted until the two curves are the same amplitude, with a value of x = 0.1. This is then set as the :ref:`Q_A_RAT_YAW_FF<Q_A_RAT_YAW_FF>` and :ref:`Q_A_RAT_YAW_I<Q_A_RAT_YAW_I>` values.

.. image:: ../../../images/FF-calculation.png
   :target: ../_images/FF-calculation.png

- Set :ref:`Q_A_RAT_PIT_SMAX<Q_A_RAT_PIT_SMAX>` and :ref:`Q_A_RAT_YAW_SMAX<Q_A_RAT_YAW_SMAX>`, **temporarily**, to "0" to avoid the oscillation protection from masking when oscillation occurs.
- Now you can hover again, and begin increasing the :ref:`Q_A_RAT_PIT_D<Q_A_RAT_PIT_D>` term, either iteratively, or using :ref:`common-transmitter-tuning`, until it oscillates and then reduce it to 1/2 to 1/3 that value.
- Then increase the :ref:`Q_A_RAT_PIT_P<Q_A_RAT_PIT_P>` term,, until it oscillates and then reduce it  1/2 to 1/3 that value.
- Do the same for the YAW axis.
- Now return :ref:`Q_A_RAT_PIT_SMAX<Q_A_RAT_PIT_SMAX>` and :ref:`Q_A_RAT_YAW_SMAX<Q_A_RAT_YAW_SMAX>` to appropriate values. These are usually set to no more than 25% of the actuators maximum slew rate, ie for a 60 deg/0.1s tilt servo, 150 or less would be used.

Roll Tuning
===========

This should allow you to get a reasonable tune for Pitch and Yaw. Roll is tuned like STEP 10 for Roll in :ref:`QuadPlane<quadplane-vtol-tuning>`. Roll axis can even be AutoTuned using QAUTOTUNE, if restricted to only the roll axis using :ref:`Q_AUTOTUNE_AXES<Q_AUTOTUNE_AXES>`.

