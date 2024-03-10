.. _tuning-process-instructions:

===========================
Tuning Process Instructions
===========================

The tuning process consists of the following steps:

1. :ref:`Setting up the parameters to prepare for the first tuning flights<setting-up-for-tuning>`
2. :ref:`Initial tuning flight <initial-tuning-flight>` to obtain a stable, but not necessarily optimized, tune.
3. :ref:`Evaluation of the initial stable tuning <evaluating-the-aircraft-tune>`
4. Setup Noise notch filters, see :ref:`common-imu-notch-filtering`
5. :ref:`Manual tuning of Roll and Pitch<ac_rollpitchtuning>` and/or use :ref:`quiktune` scripting (required before attempting to use AUTOTUNE!)
6. :ref:`AUTOTUNE<autotune>`.
7. Setting the :ref:`Input Shaping<input-shaping>` parameters to obtain the desired "feel"


The initial tune of the aircraft should be done **in the aircrafts most agile configuration**. This generally means that the aircraft will be at its minimum take off weight with fully charged batteries.

.. note:: Following each of the above steps will usually result in a safe tuning process and adequate tune for  most users. For commercial, expensive and/or high lift capacity vehicles, `this extensive, very detailed tuning procedure <https://discuss.ardupilot.org/t/how-to-methodically-tune-almost-any-multicopter-using-arducopter-4-4-x/110842>`__ is recommended, but is probably "overkill" for hobbyist/casual users.

Advanced Tuning
---------------

ArduCopter has an extremely flexible controller design that has been used with great results on aircraft from 100g to 500kg. There are a number of difficult control problems that provide a greater depth of understanding that can be provided here. Some of these issues include:

- High gyro noise levels
- Flexible airframes
- Soft vibration dampers
- Large payloads on flexible or loose mounts
- Rate limited actuators
- Non-Linear actuators
- Extremely aggressive or dynamic flight
