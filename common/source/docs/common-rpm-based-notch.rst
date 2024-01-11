.. _common-rpm-based-notch:
[copywiki destination="copter,plane"]

====================================
RPM Sensor Based Dynamic Notch Setup
====================================

- Setup an :ref:`RPM sensor<common-rpm>`.
- Set :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` and/or :ref:`INS_HNTC2_MODE <INS_HNTC2_MODE>` = 2 or 5 to use an RPM sensor to set the harmonic notch frequency.
- Set :ref:`INS_HNTCH_ENABLE <INS_HNTCH_ENABLE>` and/or :ref:`INS_HNTC2_ENABLE <INS_HNTC2_ENABLE>` = 1 to enable the harmonic notch
- Set :ref:`INS_HNTCH_REF <INS_HNTCH_REF>` and/or :ref:`INS_HNTC2_REF <INS_HNTC2_REF>` = 1 to set the harmonic notch reference value to unscaled, unless the RPM sensor is reporting a fraction of the actual rotor speed. This might occur in a geared drive train setup. If the reported RPM is 1/3 the actual, for example, set this to 0.33.
- Set :ref:`INS_HNTCH_FREQ <INS_HNTCH_FREQ>` and/or :ref:`INS_HNTC2_FREQ <INS_HNTC2_FREQ>` = below the hover frequency.  In the RPM based dynamic notch, this parameter sets the lower frequency limit of the notch.  So even if the RPM sensor reads a lower value, the notch's center frequency will remained constrainded to the value of this parameter.  You can easily determine the hover frequency by performing a hover and looking at the RPM sensor log data (e.g. RPM.rpm1).
- Set :ref:`INS_HNTCH_BW <INS_HNTCH_BW>` and/or :ref:`INS_HNTC2_BW <INS_HNTC2_BW>` = half of INS_HNTCH_FREQ

Checking Harmonic Notch Effectiveness
=====================================
Check the performance of the filter(s) after setup by doing another post filter configuration test flight as discussed in the :ref:`common-imu-batchsampling` or :ref:`common-raw-imu-logging` page for this and analyze the logs.


While the log analysis of noise frequencies is not absolutely required prior to notch setup for RPM based mode, the logging and analysis done for the Confirmation flight using that method can be done in order to confirm the noise elimination, if desired.
