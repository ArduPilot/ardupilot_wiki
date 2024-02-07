.. _common-raw-imu-logging:

================================
Raw IMU Logging for FFT Analysis
================================

Using the :ref:`IMU Batch Sampler<common-imu-batchsampling>` to gather data for spectral analysis of noise to setup notch filtering is the historical method and still valid but with firmware 4.5 and later, and if using an H7-based autopilot, a simpler and better method is to enable Raw IMU Logging combined with the `Filter Review tool <https://firmware.ardupilot.org/Tools/WebTools/FilterReview/>`__.

The downside to this method is that it produces very large logs and unless the data gathering is restricted to a single gyro, some slower autopilots (F4-based, for example), may not be able to keep up resulting in logging drop-outs. Also the data produced can only be analyzed by the `Filter Review tool <https://firmware.ardupilot.org/Tools/WebTools/FilterReview/>`__ on the `ArduPilot Firmware "Web Tools" page <https://firmware.ardupilot.org/Tools/WebTools/>`__.

Using this new method greatly simplifies the notch filtering setup.  The required data can be gathered in a single test flight and the filters can then be setup interactively with the `Filter Review tool <https://firmware.ardupilot.org/Tools/WebTools/FilterReview/>`__.

Filter Setup Process
====================

#. Select the method that will be used for dynamically adjusting the notch(s) center frequency and do any required setup, ie if :ref:`ESC telemtry<common-esc-telemetry>` will be used make sure its setup, if Throttle based make sure your motor voltage compensation is setup (see ``Q_M_BATT_x`` params for QuadPlane or ``MOT_BAT_x`` params for Copter).
#. :ref:`INS_RAW_LOG_OPT<INS_RAW_LOG_OPT>`: any bits set will enable raw IMU logging.  To prevent overloading the logging, especially on slower autopilot CPUs, set :ref:`INS_RAW_LOG_OPT<INS_RAW_LOG_OPT>` bit 0 and bit 3 (value = "9"). This will restrict logging to the primary gyro and log both pre and post filtering.
#. Perform a regular flight (not just a gentle hover) of at least 30sec and :ref:`download the dataflash logs <common-downloading-and-analyzing-data-logs-in-mission-planner>`.
#. Using the `Filter Review tool <https://firmware.ardupilot.org/Tools/WebTools/FilterReview/>`__ load the log. Select a steady hover portion of the log and press "Calculate". Then you can enable one or both of the filters in the tool and experiment with the parameters(refer back to the :ref:`common-imu-notch-filtering` page for details of what each parameter means) to get the lowest noise in the estimated post-filter spectrum, especially in the lower frequencies of the control band (0-40Hz). Remember that each additional harmonic filter adds cpu computational load. Usually only two or three per filter will be sufficient.

.. note:: if the FilterReview tool complains that not enough data is in the log, this usually means that logging has not been able to keep up and has dropped some packets. Either reduce the FFT window size to 512 from 1024, or, preferably, try just clearing the pre and post logging option in :ref:`INS_RAW_LOG_OPT<INS_RAW_LOG_OPT>` and retry the test flight to log only the input noise and use the estimated post-filter trace to setup the filters.

#. Save those parameters and load them into the autopilot.
#. Do another flight to confirm the filter setup. This this flight more normal flight maneuvers can be done.
#. Once verified, clear the :ref:`INS_RAW_LOG_OPT<INS_RAW_LOG_OPT>` parameter to avoid large logs from being saved unnecessarily.
[copywiki destination="copter,plane,rover,dev,planner"]