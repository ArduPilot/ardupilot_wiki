.. _high-loop-rate-tuning:

===========================
Aggressive Rate Loop Tuning
===========================

Normally, the attitude rate PID controllers update at the :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>`, the main loop speed and noise filtering at half the loop rate. This means that PID rate error corrections can happen only at 1/loop rate. For 400hz, this is 2.5ms. In ArduCopter 4.7 and later, it is possible to run the PID attitude rates closer to the rate at which the gyros provide measurements, which can be up to 4KHz, resulting in 2.5uS rate corrections. Tuning is more sensitive and complex, but for aerobatic applications on vehicles with large thrust to inertia ratios (ie very agile) could yield much more precise and responsive control.

.. note:: this is much more computationally intensive, resulting in higher CPU loads. On H7 based autopilots it is acceptable, but F4 based autopilots might not be able to sustain this increased load.

To setup running the PID rate loops closer to the higher gyro sampling rates, rather than the :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>`:

Configuration
=============
There are only two new settings for enabling fast rate attitude, but these have an effect on several other settings:

- :ref:`FSTRATE_ENABLE<FSTRATE_ENABLE>`:
    - 0: Disables this feature.
    - 1: Enable dynamic fast rate. On high CPU loading (as determined by either the main loop being slow or the gyro sample queue filling up) the rate will be reduced to an integer divisor of the gyro data rate until the CPU stabilizes. This is the safest way of trying the support.
    - 2: Enable dynamic fast rate, but fix to the requested rate on arming. This allows un-armed operations such as log download to proceed effectively. Note that you must be sure that you have sufficient CPU to enable this option or control can be compromised.
    - 3: Enable fixed fast rate. This is useful for testing but not usually used.

- :ref:`FSTRATE_DIV<FSTRATE_DIV>`: this defaults to 1 and represents the minimum fast rate gyro divisor. So a value of 2 will yield a rate loop of gyro rate / 2 (:ref:`INS_GYRO_RATE<INS_GYRO_RATE>` / 2). This allows a fixed fast rate that is lower than the gyro rate. This parameter is can be modified in-flight.

The easiest way to try this feature is to set :ref:`FSTRATE_ENABLE<FSTRATE_ENABLE>` = 1 (Dynamic rate). If your flight controller is struggling it will reduce the attitude loop rate until normality is restored. :ref:`FSTRATE_ENABLE<FSTRATE_ENABLE>` = 2 (Fixed Rate) is not recommended until you have seen reasonable performance with dynamic rates.

One way to gain more CPU is to switch off additional IMUs using :ref:`INS_ENABLE_MASK<INS_ENABLE_MASK>` or reduce :ref:`INS_GYRO_RATE<INS_GYRO_RATE>`. The main CPU load is from the INS and attitude control threads.

When this feature is enabled, several other settings are affected:

- :ref:`SERVO_DSHOT_RATE<SERVO_DSHOT_RATE>` - this represents a multiplier of the main loop rate to output to the motors. When the fast rate loop is enabled this is normally set to 1, however if the calculated rate will not exceed the gyro rate, higher values are allowed. For instance if :ref:`INS_GYRO_RATE<INS_GYRO_RATE>` = 2(4KHz) and :ref:`FSTRATE_DIV<FSTRATE_DIV>` = 4 (1KHz)then values of :ref:`SERVO_DSHOT_RATE<SERVO_DSHOT_RATE>` up to 4(4x loop rate) are allowed.
- Fast attitude and PID logging enabled in :ref:`LOG_BITMASK<LOG_BITMASK>` - setting these bits usually logs at the main loop rate, however if fast attitude is enabled these will be logged at the minimum of the fast attitude rate and 1kHz
- :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>` - since all the heavy lifting of attitude control is being done in a separate thread, :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>` can be run at normal values - for instance 400Hz or even less.

Tuning Setup
============
Its extremely important that the :ref:`Notch filtering <common-imu-notch-filtering>` be setup as well as the :ref:`tuning-process-instructions` be followed before tuning with this feature enabled.

- :ref:`INS_GYRO_FILTER<INS_GYRO_FILTER>` - as high as you can get without cooking your motors. Often copters have resonance around 120Hz so it is hard to get higher than this without notching out frame resonance.
- each axis' ``ATC_RAT_xxx_FLTD`` should be :ref:`INS_GYRO_FILTER<INS_GYRO_FILTER>`/2 on roll and pitch and :ref:`INS_GYRO_FILTER<INS_GYRO_FILTER>`/4 on yaw. This is because D is more active, lets more noise through and is more likely to cook your motors. Setting it above 0.75 * :ref:`INS_GYRO_FILTER<INS_GYRO_FILTER>` is not recommended.
- each axis' ``ATC_RAT_xxx_FLTT`` should be at the highest rate you want the EKF or your fingers to control the copter. Values of 30Hz should be fine, but higher has diminishing returns but does not affect the tune.
