.. _common-imu-fft:

[copywiki destination="copter,plane"]

========================================
In-Flight FFT-Based Harmonic Notch Setup
========================================

.. note:: This feature is included in autopilots with 2MB of memory. Check your autopilot's :ref:`binary-features` to determine if your autopilot has this feature (GyroFFT). Also, only 1 FFT-based Notch can be set up.

.. _common-imu-fft-pre-flight-setup:

ArduPilot comes pre-configured with appropriate defaults for all FFT settings. The only initial setup required is:

- Set :ref:`FFT_ENABLE <FFT_ENABLE>` = 1 to enable the FFT engine. This then requires that you reboot your autopilot after which FFT support will be enabled and other FFT parameters should be visible in your GCS. With default parameter settings the FFT engine will run a self-check for frequency matching on your hardware. If you do not see any FFT errors then things are working properly.

- With FFT enabled it is best to first perform a test flight to check that your aircraft's particular noise frequencies are being captured and to monitor CPU load. See :ref:`Initial Analysis Flight <common-imu-fft-test-flight>`. Normally, results from this will show clear noise recognition and acceptable cpu loading, and  then you can use the FFT to drive the :ref:`harmonic notch<common-imu-notch-filtering>` by setting these parameters:

- Set :ref:`INS_HNTCH_ENABLE <INS_HNTCH_ENABLE>` and/or :ref:`INS_HNTC2_ENABLE <INS_HNTC2_ENABLE>` = 1 to enable the harmonic notch = 1 to enable the harmonic notch
- Set :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` and/or :ref:`INS_HNTC2_MODE <INS_HNTC2_MODE>` = 4 to use the FFT detected frequency for controlling the harmonic notch frequency.
- Set :ref:`INS_HNTCH_REF <INS_HNTCH_REF>` and/or :ref:`INS_HNTC2_REF <INS_HNTC2_REF>` = 1 to set the harmonic notch reference value, which for FFT analysis generally means no scaling

For most uses with other FFT-related advanced parameters at their default, this is all that is required. The user can do optimization of the filtering setup by analyzing the test flight logs and adjusting notch bandwidth, if desired, by following the :ref:`In-flight FFT Advanced Setup <common-imu-fft-advanced-setup>` instructions.

.. note:: Setting up the FFT parameters can be done automatically using the ``RCx_OPTION`` auxiliary function "162" on a transmitter switch. Set the function to a switch on the transmitter. Hover the vehicle, switch it on (high) for 30 seconds, and switch back low and land. The parameters will have been set up and the switch function removed. NOTE: do not use this feature in firmware version 4.3!

.. note:: Using In-Flight FFT can result in poorer performance than a properly set up :ref:`Throttle-Based <common-throttle-based-notch>` notch filter since the FFT computations take time and can lag the actual required center frequency. In-Flight FFT is useful when the rotor frequencies of the vehicle vary widely as in heavy lift vehicles operating with high and low loads. It can be useful in setting up :ref:`Throttle-Based <common-throttle-based-notch>` notch filters, however, see :ref:`In-flight FFT Advanced Setup <common-imu-fft-advanced-setup>` instructions for more information.

FFT Dynamic Harmonic Notch Frequency Tracking
=============================================

FFT mode tracking sets the base frequency to the largest noise peak. Normally, when multiple harmonic notch filters are then enabled, the center frequency of each harmonic is locked to the base frequency of the first filter as an integer multiple, as determined by :ref:`INS_HNTCH_HMNCS <INS_HNTCH_HMNCS>`. Setting bit 1 of :ref:`INS_HNTCH_OPTS<INS_HNTCH_OPTS>`, or :ref:`INS_HNTC2_OPTS<INS_HNTC2_OPTS>`, will enable each harmonic filter to track the largest noise peaks, individually.

.. note:: setting bit 1 of the notch options will also change the default value of :ref:`INS_HNTCH_HMNCS <INS_HNTCH_HMNCS>` to 1 instead of its normal 3. This is to maintain backwards compatibility with previous fimware versions. You can set :ref:`INS_HNTCH_HMNCS <INS_HNTCH_HMNCS>` back to 3, or whatever is desired, after setting bit 1 of :ref:`INS_HNTCH_HMNCS <INS_HNTCH_HMNCS>`.

FFT Options
===========

There are two options that can be selected by setting the appropriate bit in the :ref:`FFT_OPTIONS<FFT_OPTIONS>` parameter that affect FFT operation:

Post Filter Chain FFT Analysis Window
-------------------------------------

Normally, the FFT analysis for adjusting the center frequency is done by measuring the noise directly at the output of the unfiltered gyro data. However, if bit 0 of :ref:`FFT_OPTIONS<FFT_OPTIONS>` is set, then the measurement window takes into account the effects of the low pass filter and any configured notch filter(s). This is useful if there is high-frequency noise, which impacts the control response less than lower-frequency noise due to the low pass at the end of the filter chain but may be targeted by the FFT measurement. Setting this bit will only track those lower frequency, and more critical, noise peaks.

Motor Noise Check
-----------------

If bit 1 of :ref:`FFT_OPTIONS<FFT_OPTIONS>` is set and ESC motor rpm telemetry is available, then the measurement window for the FFT is centered about the motor(s) frequency as reported by ESC telemetry. This will generate a GCS warning message if noise from any motor greater than 40db is passing through the filter chain, and identify its level, motor number, and frequency. This bit must be used with bit 0 also set.

Typical Use
===========

A typical use of the FFT notch filter is in addition to other dynamic harmonic notch filters (Throttle, ESC, or RPM-based). In these configurations, using the post-LPF FFT Window :ref:`FFT_OPTIONS<FFT_OPTIONS>` bit will yield the best overall filtering results by positioning the FFT filter to target noise not filtered by the other notch filter and gyro LPF (:ref:`INS_GYRO_FILTER<INS_GYRO_FILTER>`.

Additional Information
======================

For those interested in the details of how this feature works and tradeoffs in some of the advanced parameters, not normally adjusted by users, the :ref:`In-Flight FFT: How it Works <common-imu-fft-how-it-works>` document describes the operation and these advanced parameters.

.. toctree:: 
    :maxdepth: 1

    Initial Analysis Flight <common-imu-fft-test-flight>
    In-Flight FFT Advanced Setup <common-imu-fft-advanced-setup>
    In-Flight FFT: How it Works <common-imu-fft-how-it-works>

