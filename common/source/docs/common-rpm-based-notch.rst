.. _common-rpm-based-notch:
[copywiki destination="copter,plane"]

====================================
RPM Sensor Based Dynamic Notch Setup
====================================

- Set :ref:`INS_HNTCH_ENABLE <INS_HNTCH_ENABLE>` = 1 to enable the harmonic notch
- Set :ref:`INS_HNTCH_REF <INS_HNTCH_REF>` = 1 to set the harmonic notch reference value to unscaled, unless the RPM sensor is reporting a fraction of the actual rotor speed. This might occur in a geared drive train setup. If the reported RPM is 1/3 the actual, for example, set this to 0.33.
- Set :ref:`INS_HNTCH_FREQ <INS_HNTCH_FREQ>` = below the hover frequency - you can easily determine this by performing a gentle hover and looking at the RPM sensor log data (RPM.rpm1)
- Set :ref:`INS_HNTCH_BW <INS_HNTCH_BW>` = half of INS_HNTCH_FREQ

Checking Harmonic Notch Effectiveness
=====================================
After setting up the harmonic notch, the effect on the control signal data can be checked using the instructions for :ref:`common-imu-notch-filtering-post-configuration-flight-and-post-flight-analysis`  in the throttle based setup.

While the log analysis required for the Throttle-based mode's setup is not required for RPM based mode, the logging and analysis done for the Confirmation flight in that method can be done in order to confirm the noise elimination, if desired.
