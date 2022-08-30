.. _traditional-helicopter-tips:

===========================
Traditional Helicopter Tips
===========================

- Be sure to follow the wiki instructions, step by step!
- Be sure the CG is in line with the main rotor shaft
- Blade tracking must be within a quarter blade thickness or better to avoid excessive vibration
- Vibration is the arch enemy of Helicopter autopilots!
   - Internal Combustion Engines and Turbines can created high frequency vibrations that do not appear in the VIBE logs and can be aliased to low frequencies which can impact EKF and Attitude control (see below for ground tests to check for this). This also often occurs in electric helicopters.
   - The autopilot should be well isolated for vibration. Using autopilots with internal IMU vibration damping and vibration reducing mounts is recommended.
- Traditional Helicopter *is* capable of inverted flight and using full negative collective ranges.

Checking for High Frequency Vibration
=====================================

High frequency vibrations in this context are vibrations close to the IMU sampling frequency, or its multiples, which will be aliased down into the same range as control inputs for the attitude controllers. Symptoms of high frequency vibration can be that the attitude estimate becomes incorrect causing it to lean significantly even though the pilot is commanding level flight in stabilized modes. In the extreme, it can result in an :ref:`ekf-inav-failsafe` or even a crash. Excessive vibration can also make altitude hold imprecise or even produce uncontrollable climbs/descents.

One method of testing this, is to secure the helicopter to the ground, and run up the main rotor, watching the artificial horizon in the ground control station. If it moves away from level, you probably have a vibration issue which needs to be addressed. Testing should be done at various collective levels and rotor speeds. Be careful not to over-stress the helicopter with collective extremes.

Setup for ground test:

#. Secure vehicle to the ground.
#. Temporarily reduce Roll, Pitch, and Yaw  P,I,and D, PIDs to zero. Be sure to restore them after this test!
#. Set :ref:`ACRO_OPTIONS<ACRO_OPTIONS>` = 2, temporarily, if not being used already.
#. Change to ACRO mode. 
#. Arm and engage motor interlock, allowing the vehicle to spool up at 0 degree pitch, watching for any change from level in the GSC of the artificial horizon. Any significant tilt indicates noise is disrupting the attitude estimate and should be investigated and eliminated.
#. Change the target main rotor speed a bit above and below the nominal target using the :ref:`H_RSC_SETPOINT<H_RSC_SETPOINT>` (or TX throttle curve if using Passthru mode) and repeat.
#. Re-check the blade tracking also while doing this test.

Be sure to also eliminate the possibility of lower frequency vibrations from the main or tail rotors causing the issue. Unlike vibrations near the sampling rate and its harmonics, which cannot be eliminated by software filtering, rotor vibrations can be improved using ArduPilot's notch filtering feature. See :ref:`common-measuring-vibration` and :ref:`Helicopter Dynamic Notch Filter Setup<common-imu-notch-filtering-helicopter-setup>`.

Possible False Landing Detections
=================================

Under extreme circumstances (flying in extremely turbulent conditions, for example), it is possible for the firmware, to falsely detect a landing condition, although still flying. This can lead to lack of control in position control modes and even possible disarming. An RC Auxiliary Function ("159") is provided to allow an RC to enable (needed when actually landing) or disable (to prevent false detects during missions) the landing detection algorithm under pilot control.
