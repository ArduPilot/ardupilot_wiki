.. _quicktune:

==============
VTOL QUICKTUNE
==============

.. note: QUICKTUNE can only be used in QHOVER,QLOITER, or GUIDED VTOL modes.

ArduPlane provides a quick means of obtaining a good tune for Quadplanes in VTOL modes. The process slowly increases the relevant gains until it detects an oscillation.  It then reduces the gains by 60% and moves onto the next gain. Once all the gains have been tuned the tune completes and the user can decide to save or discard the new gains.

.. note:: be sure that you have prepared the vehicle for tuning by setting up parameters discussed here: :ref:`quadplane-vtol-tuning-process` Steps 1 to 11 and then use this mechanism in Step 12. Also to assure the best tune, setup the noise notch filtering, see :ref:`common-imu-notch-filtering`. You may run QUICKTUNE without this step to obtain initial fine tune, see :ref:`ac_rollpitchtuning` if the vehicle cant do an initial hover stably. Then setup the filters and retune for best results.

The script attempts to automatically tune the P/I and D gain of each axis enabled for tuning when an two or three position RC switch, whose ``RCx_OPTION`` is "181", is moved to the "tune" position. It will, by default, also adjust some PID loop filter values based on the :ref:`INS_GYRO_FILTER<INS_GYRO_FILTER>` setting.

The advantage over QAUTOTUNE is that QUICKTUNE is safer because the vehicle does not need to move or twitch and QAUTOTUNE can occasionally result in instablity. Therefore QAUTOTUNE is not longer recommended for use, except by experts.

The disadvantage is that QUICKTUNE cannot find the vehicle's maximum rotational accelerations or tune the attitude angle outer loops from their default or pre-tune adjusted (:ref:`quadplane-vtol-tuning-process`) values. However, tuning only the inner rate loops usually will yield a stable, safe flying vehicle.

Setup
=====
- Set :ref:`QWIK_ENABLE<QWIK_ENABLE>` = "1" to enable QUICKTUNE.
- Set :ref:`QWIK_AXES<QWIK_AXES>` to which axes (Roll/Pitch/Yaw).

Activation RC Switch
--------------------
Set up a two or three position RC switch with `RCx_OPTION`` = "181". If it is a two position switch set :ref:`QWIK_OPTIONS<QWIK_OPTIONS>` bit 0 to "1", and you probably want to set the :ref:`QWIK_AUTO_SAVE<QWIK_AUTO_SAVE>` parameter to autosave the tune once completed.

With a three position switch, moving the switch to "high" (>1800us) is the "save" position which will save the current tune parameters, even if the tune is not completed yet. Moving the switch to the middle position activates the tuning process ("tune"). Moving the switch to the "low" position (<1200us), stops the tuning and reverts the parameters to their values before tuning began.

.. note:: RC AUX switches only operate on transitions of the switch. Their position at boot does nothing, so if a switch is "high" at boot, in order to get the "high" function, it must be moved to another position and then back "high".

With a two position switch, "high" is the "tune" position, and "low" stops/reverts tuning. In order to save parameters, you must have set the :ref:`QWIK_AUTO_SAVE<QWIK_AUTO_SAVE>` parameter or use a GCS AUX switch facility.

Tuning Flight
=============
- Wait for a calm day and go to an open area with good GPS reception
- Use an OSD, TX OSD (using :ref:`Yaapu <common-frsky-yaapu>`), or GCS. This is where output from the tune will appear
- Move the RC switch to the low position OR push MP's Aux Function's "Low" button
- Arm and takeoff in QLOITER mode and climb to a height of about 3m
- Begin the tune by moving the RC switch to the "tune" position OR push MP's Aux Function's "Mid" button
- Monitor the progress of the tune using the GCS's Messages tab
- If necessary reposition the vehicle using the RC transmitter.  This will temporarily pause tuning.  Tuning will resume a few seconds after the RC sticks are returned to their center position
- If the vehicle begins oscillating violently you can cancel the tune by moving the RC switch to the low position OR push MP's Aux Function's "Low" button. If the vehicle rocks or pitches more than :ref:`QWIK_ANGLE_MAX<QWIK_ANGLE_MAX>` the tune will automatically abort.
- Once the tune has completed accept the new gains by moving the RC aux switch to the "save" position, wait for the :ref:`QWIK_AUTO_SAVE<QWIK_AUTO_SAVE>` timeout, OR push MP's Aux Function's "High" button
- Land and disarm the vehicle

Advanced Configuration
======================
Additional advanced parameters are shown below, but normally do not need to be changed and, then only by experienced users:

- :ref:`QWIK_AUTO_FILTER<QWIK_AUTO_FILTER>`
- :ref:`QWIK_DOUBLE_TIME<QWIK_DOUBLE_TIME>`
- :ref:`QWIK_GAIN_MARGIN<QWIK_GAIN_MARGIN>`
- :ref:`QWIK_OSC_SMAX<QWIK_OSC_SMAX>`
- :ref:`QWIK_YAW_P_MAX<QWIK_YAW_P_MAX>`
- :ref:`QWIK_YAW_D_MAX<QWIK_YAW_D_MAX>`
- :ref:`QWIK_RP_PI_RATIO<QWIK_RP_PI_RATIO>`
- :ref:`QWIK_Y_PI_RATIO<QWIK_Y_PI_RATIO>`
- :ref:`QWIK_REDUCE_MAX<QWIK_REDUCE_MAX>`
- :ref:`QWIK_ANGLE_MAX<QWIK_ANGLE_MAX>`