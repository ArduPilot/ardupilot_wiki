.. _qwiktune:

=============
VTOL QwikTune
=============

ArduPlane provides a quick means of obtaining a good tune for Quadplanes in VTOL modes. The process slowly increases the relevant gains until it detects an oscillation.  It then reduces the gains by 60% and moves onto the next gain. Once all the gains have been tuned the tune completes and the user can decide to save or discard the new gains.

.. note:: be sure that you have prepared the vehicle for tuning by setting up parameters discussed here: :ref:`quadplane-vtol-tuning-process` Steps 1 to 11 and then use this mechanism in Step 12. Also to assure the best tune, setup the noise notch filtering, see :ref:`common-imu-notch-filtering`. You may run QuickTune without this step to obtain initial fine tune, see :ref:`ac_rollpitchtuning` if the vehicle cant do an initial hover stably. Then setup the filters and retune for best results.

**NEEDS updating:**
The script attempts to tune all these parameters (in the given order)

- :ref:`ATC_RAT_RLL_D<ATC_RAT_RLL_D__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_RLL_P<ATC_RAT_RLL_P__AC_AttitudeControl_Multi>` and :ref:`ATC_RAT_RLL_I<ATC_RAT_RLL_I__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_PIT_D<ATC_RAT_PIT_D__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_PIT_P<ATC_RAT_PIT_P__AC_AttitudeControl_Multi>` and :ref:`ATC_RAT_PIT_I<ATC_RAT_PIT_I__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_YAW_D<ATC_RAT_YAW_D__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_YAW_P<ATC_RAT_YAW_P__AC_AttitudeControl_Multi>` and :ref:`ATC_RAT_YAW_I<ATC_RAT_YAW_I__AC_AttitudeControl_Multi>`

The advantage over QAUTOTUNE is that QwikTune is safer because the vehicle does not need to move or twitch and QAUTOTUNE can occasionally result in instablity. Therefore QAUTOTUNE is not longer recommended for use, except by expects.
**needs updating with correct params**
The disadvantage is that QwikTune cannot find the vehicle's maximum rotational accelerations (e.g. :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>`, :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>`, :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>`)

Setup
=====

**Talk about setting up sw and its operation options**
**Talk about which QWIK params might need changing before using**

Tuning Flight
=============
- Wait for a calm day and go to an open area with good GPS reception
- Use an OSD, TX OSD (using :ref:`Yaapu <common-frsky-yaapu>`), or GCS. This is where output from the tune will appear
- Move the RC switch to the low position OR push MP's Aux Function's "Low" button
- Arm and takeoff in Loiter mode and climb to a height of about 3m
- Begin the tune by moving the RC switch to the middle position OR push MP's Aux Function's "Mid" button
- Monitor the progress of the tune using the GCS's Messages tab
- If necessary reposition the vehicle using the RC transmitter.  This will temporarily pause tuning and restore the original gains.  Tuning will resume a few seconds after the RC sticks are returned to their center position
- If the vehicle begins oscillating violently cancel the tune by moving the RC switch to the low position OR push MP's Aux Function's "Low" button
- Once the tune has completed accept the new gains by moving the RC aux switch to the high position OR push MP's Aux Function's "High" button
- Land and disarm the vehicle

Advanced Configuration
======================

**ADD every QWIK param not discussed so far and explain its use and when to change from default**