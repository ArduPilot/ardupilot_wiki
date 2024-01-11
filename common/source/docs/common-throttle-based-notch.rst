.. _common-throttle-based-notch:

[copywiki destination="copter,plane"]
==================================
Throttle Based Dynamic Notch Setup
==================================

.. _common-imu-notch-filtering-throttle-based-setup:

If you do not have ESC telemetry or an RPM sensor, and are not using an autopilot capable of using the in-flight FFT mode, then throttle-based (:ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 1 , or :ref:`INS_HNTC2_MODE <INS_HNTC2_MODE>` = 1) center frequency control is best.

In order to configure the throttle-based dynamic harmonic notch filter it is important to identify  the motor noise at the hover throttle level. To do this we need to use the :ref:`common-raw-imu-logging` to obtain logs for analysis.

Once this is done, the center frequency of the notch(s) can be set and other parameters configured:

Harmonic Notch Configuration for Throttle Based Mode
====================================================

- Set :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` and/or :ref:`INS_HNTC2_MODE <INS_HNTC2_MODE>` = 1.
- Set :ref:`INS_HNTCH_ENABLE <INS_HNTCH_ENABLE>` and/or :ref:`INS_HNTC2_ENABLE <INS_HNTC2_ENABLE>` = 1 to enable the harmonic notch
- Set :ref:`INS_HNTCH_REF <INS_HNTCH_REF>` and/or :ref:`INS_HNTC2_REF <INS_HNTC2_REF>` = *hover_thrust* to set the harmonic notch reference value
- Set :ref:`INS_HNTCH_FREQ <INS_HNTCH_FREQ>` and/or :ref:`INS_HNTC2_FREQ <INS_HNTC2_FREQ>` = *hover_freq* to set the harmonic notch reference frequency
- Set :ref:`INS_HNTCH_BW <INS_HNTCH_BW>` and/or :ref:`INS_HNTC2_BW <INS_HNTC2_BW>` = *hover_freq* / 2 to set the harmonic notch bandwidth
- Set :ref:`INS_HNTCH_FM_RAT<INS_HNTCH_FM_RAT>` and/or :ref:`INS_HNTC2_FM_RAT<INS_HNTC2_FM_RAT>` to the percentage of :ref:`INS_HNTCH_FREQ <INS_HNTCH_FREQ>` and/or :ref:`INS_HNTC2_FREQ <INS_HNTC2_FREQ>` that you desire the notch frequency to track below hover throttle. Note that lower frequency notch filters will have more phase lag and can impact stability. If you want throttle based notch filtering to be effective at a throttle up to 30% below the configured notch frequency then set this parameter to 0.7. The default of 1.0 means the notch will not go below the frequency in the :ref:`INS_HNTCH_FREQ <INS_HNTCH_FREQ>` and/or :ref:`INS_HNTC2_FREQ <INS_HNTC2_FREQ>` parameter.

Check the performance of the filter(s) after setup by doing another post filter configuration test flight as discussed in the :ref:`common-imu-batchsampling` or :ref:`common-raw-imu-logging` page for this and analyze the logs.

Advanced Notch Frequency Scaling Adjustment in Throttle Based Mode
==================================================================

The harmonic notch is designed to match the motor noise frequency as it changes by interpreting the throttle value. The frequency is scaled up from the hover frequency and will never go below this frequency. However, in dynamic flight it is quite common to hit quite low motor frequencies during propwash. In order to address this it is possible to change the ref value in order to scale from a lower frequency.

- First perform a long dynamic flight using your current settings and post-filter batch logging. Examine the FFT and look at how far the motor noise peak extends below the hover frequency. Use this frequency - *min_freq* - as the lower bound of your scaling. Then in order to calculate an updated value of the throttle reference use:

:ref:`INS_HNTCH_REF <INS_HNTCH_REF>` and/or :ref:`INS_HNTC2_REF <INS_HNTC2_REF>` = *hover_thrust* * SQUARE(*min_freq / hover_freq*)
