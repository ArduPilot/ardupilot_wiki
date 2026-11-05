.. _common-raw-imu-logging:

================================
Raw IMU Logging for FFT Analysis
================================

Using the :ref:`IMU Batch Sampler<common-imu-batchsampling>` to gather data for spectral analysis of noise to setup notch filtering is the historical method and still valid but with firmware 4.5 and later, and if using an H7-based autopilot, a simpler and better method is to enable Raw IMU Logging combined with the `Filter Review tool <https://firmware.ardupilot.org/Tools/WebTools/FilterReview/>`__.

The downside to this method is that it produces very large logs and unless the data gathering is restricted to a single gyro, some slower autopilots (F4-based, for example), may not be able to keep up resulting in logging drop-outs. Also the data produced can only be analyzed by the `Filter Review tool <https://firmware.ardupilot.org/Tools/WebTools/FilterReview/>`__ on the `ArduPilot Firmware "Web Tools" page <https://firmware.ardupilot.org/Tools/WebTools/>`__.

Using this new method greatly simplifies the notch filtering setup.  The required data can be gathered in a single test flight and the filters can then be setup interactively with the `Filter Review tool <https://firmware.ardupilot.org/Tools/WebTools/FilterReview/>`__.

Filter Setup Process
====================

#. Select the method that will be used for dynamically adjusting the notch(s) center frequency and do any required setup, ie if :ref:`ESC telemetry<common-esc-telemetry>` will be used make sure its setup, if Throttle based make sure your motor voltage compensation is setup (see ``Q_M_BATT_x`` params for QuadPlane or ``MOT_BAT_x`` params for Copter).
#. :ref:`INS_RAW_LOG_OPT<INS_RAW_LOG_OPT>`: any bits set will enable raw IMU logging.  To prevent overloading the logging, especially on slower autopilot CPUs, set :ref:`INS_RAW_LOG_OPT<INS_RAW_LOG_OPT>` bit 0 and bit 3 (value = "9"). This will restrict logging to the primary gyro and log both pre and post filtering.
#. .. _common-raw-imu-logging-determination-flight:

   Perform a regular flight (not just a gentle hover) of at least 30sec and :ref:`download the dataflash logs <common-downloading-and-analyzing-data-logs-in-mission-planner>`.

#. Load the log into the `Filter Review tool <https://firmware.ardupilot.org/Tools/WebTools/FilterReview/>`__ and set up the filters as described in :ref:`common-raw-imu-logging-filter-review` below (refer back to the [site wiki="copter,plane,dev,planner"]:ref:`common-imu-notch-filtering`\ [/site][site wiki="rover,sub"]Managing Gyro Noise with the Dynamic Harmonic Notch Filters[/site] page for details of what each parameter means). Remember that each additional harmonic filter adds cpu computational load. Usually only two or three per filter will be sufficient.

.. note:: if the FilterReview tool complains that not enough data is in the log, this usually means that logging has not been able to keep up and has dropped some packets. Either reduce the FFT window size to 512 from 1024, or, preferably, try just clearing the pre and post logging option in :ref:`INS_RAW_LOG_OPT<INS_RAW_LOG_OPT>` and retry the test flight to log only the input noise and use the estimated post-filter trace to setup the filters.

#. Save those parameters and load them into the autopilot.
#. .. _common-raw-imu-logging-post-configuration-flight:

   Optionally, do another flight to confirm the filter setup. On this flight more normal flight maneuvers can be done.

#. Once verified, clear the :ref:`INS_RAW_LOG_OPT<INS_RAW_LOG_OPT>` parameter to avoid large logs from being saved unnecessarily.

.. _common-raw-imu-logging-filter-review:

Using the Filter Review Tool
============================

#. **Load the log** with the file button. The log type and filter version are filled in from the log.
#. **Pick a hover.** In the Flight Data plot, click and drag to zoom in on the steady hover, leaving out take off, landing and big throttle changes. This sets the Analysis time. Then click Calculate.
#. **Find the noise.** The IMU Spectrum shows how much noise the gyros see at each frequency. Peaks in the Pre-filter lines are noise to be removed. On multicopters these are usually motor noise at the motor rotation frequency and its multiples (harmonics). The aim is to remove those peaks with as little filtering as possible, not to make the Post-filter line as low as it can go. Every filter adds delay (phase lag), which makes control worse; a very low gyro filter would give a flat line and a vehicle that flies badly. The Frequency Response plot shows the delay the filters add.
#. **Check the notch.** Tick "Show notch 1" (and "Show notch 2" if used). For each harmonic, the vertical line is the notch's average centre frequency over the analysis time (solid for notch 1, dotted for notch 2), and the grey shaded band is the range the notch moved over. The line and band should sit over the noise peaks. The IMU Spectrogram shows whether the notch follows the noise as the throttle changes.
#. **Try changes.** Edit the values in Filter Configuration and click Recalculate filters. The Estimated post-filter lines show what the noise would look like with the new settings, without another flight. Hover over a parameter name for its description.
#. **Save Parameters** and load the file into the autopilot.

[copywiki destination="copter,plane,rover,dev,sub,planner"]