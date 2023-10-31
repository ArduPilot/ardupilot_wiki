.. _ac_rollpitchtuning:

===============================
Manual tuning of Roll and Pitch
===============================

Manual tuning may be required to provide a stable tune before Autotune is run, or if Autotune does not produce an acceptable tune. The process below can be done on roll and pitch at the same time for a quick manual tune **provided the aircraft is symmetrical**. If the aircraft is not symmetrical, then the process should be done for roll and pitch individually.

The pilot should be especially careful to ensure that :ref:`ATC_THR_MIX_MAN <ATC_THR_MIX_MAN>` and :ref:`MOT_THST_HOVER <MOT_THST_HOVER>` are set correctly before manual tuning is started.

When oscillations start do not make large or sudden stick inputs. Reduce the throttle smoothly to land the aircraft while using very slow and small roll and pitch inputs to control the aircraft position.
For each axis:

If the vehicle already oscillates in an axis, first lower the P, D, and I terms in 50% steps until stable, before starting manual tuning.

1. Increase the D term in steps of 50% until oscillation is observed
2. Reduce the D term in steps of 10% until the oscillation disappears
3. Reduce the D term by a further 25%
4. Increase the P term in steps of 50% until oscillation is observed
5. Reduce the P term in steps of 10% until the oscillation disappears
6. Reduce the P term by a further 25%

Each time the P term is changed set the I term equal to the P term. Those parameters can be changed on ground and preferably disarmed. A confident pilot could set them in flight with GCS or transmitter tuning knob (see :ref:`Transmitter based tuning<common-transmitter-tuning>` page for setup of this feature).

.. _ac_rollpitchtuning_in-flight_tuning:

If using :ref:`Transmitter based tuning<common-transmitter-tuning>`, set the minimum value of the tuning range to the current safe value and the upper range to approximately 4 times the current value. Be careful not to move the slider before the parameter list is refreshed to recover the set value. Ensure the transmitter tuning is switched off before setting the parameter value or the tuning may immediately overwrite it.

Video of in-flight tuning
~~~~~~~~~~~~~~~~~~~~~~~~~

..  youtube:: NOQPrTdrQJM&t=145s
    :width: 100%
