.. _common-esc-telem-based-notch:
[copywiki destination="copter,plane"]

========================================
ESC Telemetry Based Harmonic Notch Setup
========================================

.. _esc-telemetry-based-dynamic-notch-filter:


If ESCs with telemetry are used (either via a separate telemety wire or bi-directional dshot) then the harmonic notch reference frequency can be set dynamically using ESC telemetry.  The harmonic notch reference frequency parameter, :ref:`INS_HNTCH_FREQ <INS_HNTCH_FREQ>` and/or :ref:`INS_HNTC2_FREQ <INS_HNTC2_FREQ>`, is used to indicate the lowest motor speed for which the ESC telemetry should be used to dynamically set the harmonic notch reference frequency.  It is recommended that this be set to below the hover frequency but above the :ref:`INS_GYRO_FILTER<INS_GYRO_FILTER>` frequency.

- Set the :ref:`INS_HNTCH_REF <INS_HNTCH_REF>` and/or :ref:`INS_HNTC2_REF <INS_HNTC2_REF>` parameter to 1, which will disable scaling of the harmonic notch, and set :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` and/or :ref:`INS_HNTC2_MODE <INS_HNTC2_MODE>` to 3 to select ESC telemetry.
- Set :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` and/or :ref:`INS_HNTC2_MODE <INS_HNTC2_MODE>` = 3 to use BLHeli ESC telemetry support to set the harmonic notch frequency. This requires that your ESCs are configured correctly to support BLHeli telemetry via :ref:`a serial port<blheli32-esc-telemetry>` or are Bi-directional DShot capable and using Ardupilot autopilot and firmware which supports that capability.
- Set :ref:`INS_HNTCH_ENABLE <INS_HNTCH_ENABLE>` and/or :ref:`INS_HNTC2_ENABLE <INS_HNTC2_ENABLE>` = 1 to enable the harmonic notch
- Set :ref:`INS_HNTCH_FREQ <INS_HNTCH_FREQ>` and/or :ref:`INS_HNTC2_FREQ <INS_HNTC2_FREQ>` = below the hover frequency - you can easily determine this by performing a gentle hover and looking at the ESC telemetry data
- Set :ref:`INS_HNTCH_BW <INS_HNTCH_BW>` and/or :ref:`INS_HNTC2_BW <INS_HNTC2_BW>` = half of INS_HNTCH_FREQ

.. note::

   If you set the dynamic harmonics option (i.e. a notch per motor) mentioned below, the bandwidth should not be half the frequency. It should be greatly reduced as more notches cause more phase lag (i.e. latency), thus the bandwidth needs to be reduced to maintain a reasonable phase lag, else oscillation and a poorer tune will result despite the more accurate filtering compared to throttle-based filtering. For example, with four motors, :ref:`INS_HNTCH_FREQ<INS_HNTCH_FREQ>` / 4 is suggested as a starting point.
   This is because for notch filters, a wider bandwidth causes a greater phase lag per notch.
   You can also use the `Filter Tool <https://firmware.ardupilot.org/Tools/FilterTool/>`__ to check the phase lag for your chosen filtering settings. If your phase lag is higher than it was with the throttle-based notch, then you can try reducing the bandwidth even further, balancing the phase lag and the amount of noise in the system.

Center Frequency Slewing
========================

The rate at which the harmonic notch frequency is updated has a big impact on noise in the PID loops. Slower update rates mean that the frequency has larger step changes which result in what is called shot noise. Faster update rates reduce this and is the primary reason why using bi-directional dshot with ESC Telemetry reporting of RPM benefits the system overall.

By default the update rate is 200Hz and where the source of frequency information is slower than that - for instance when using ESC telemetry where the maximum rate that can be sustained is about 100Hz - ArduPilot will slew the frequency changes at 200Hz to avoid large steps. The slewed rate is the rate that is reported by ESC telemetry, although the raw rate can be seen in the logs as well.

On systems with faster CPUs (H7 based autopilots) it is possible to update the harmonic notch at the main loop rates used for VTOL aircraft (typically 300-400Hz set by :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>`) by setting bit 3 of the notch options, i.e.

- :ref:`INS_HNTCH_OPTS<INS_HNTCH_OPTS>` and/or :ref:`INS_HNTC2_OPTS<INS_HNTC2_OPTS>` = 4

Slewing ensures that the step changes at each update tick are smooth, but for optimum system performance you can use bi-directional Dshot ESCs which can deliver frequency updates at 400Hz, using the above option, if possible .

Checking Harmonic Notch Effectiveness
=====================================
Once the notch filter(s) are setup, the effectiveness of them can be checked by again measuring the  frequency spectrum of the output of the filters. Refer back to the :ref:`common-imu-batchsampling`  or :ref:`common-raw-imu-logging` page for this.


While the log analysis of noise frequencies is not absolutely required prior to notch setup for RPM based mode, the logging and analysis done for the Confirmation flight using that method can be done in order to confirm the noise elimination, if desired.

Dynamic Harmonics
=================

By default the ESC based harmonic notch will use an average of the individual motor frequencies in order to drive the center frequency of the notch, this average is then used as the first harmonic and other harmonic notches are added at higher frequency multiples of the first harmonic. It is also possible to configure the harmonics to instead be first harmonics per motor. This gives four notches - one for each motor - that exactly tracks the motor speed. In dynamic flight this can give much better noise attenuation.

To configure this option set :ref:`INS_HNTCH_OPTS <INS_HNTCH_OPTS>` and/or :ref:`INS_HNTCH_OPTS <INS_HNTCH_OPTS>` to "2"
