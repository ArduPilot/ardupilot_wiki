.. _quiktune:

========
QuikTune
========

..  youtube:: K_T9ikEQmlc
    :width: 100%

The `VTOL QuikTune <https://github.com/ArduPilot/ardupilot/blob/Copter-4.5/libraries/AP_Scripting/applets/VTOL-quicktune.md>`__ Lua script simplifies the process of finding a good tune for a multicopter's attitude control parameters.

The script slowly increases the relevant gains until it detects an oscillation.  It then reduces the gains by 60% and moves onto the next gain.
Once all the gains have been tuned the tune completes and the user can decide to save or discard the new gains.

.. note:: be sure that you have prepared the vehicle for tuning by setting up parameters discussed here: :ref:`setting-up-for-tuning`. Also to assure the best tune, setup the noise notch filtering, see :ref:`common-imu-notch-filtering`. You may run QuickTune without this step to obtain initial fine tune, see :ref:`ac_rollpitchtuning` if the vehicle cant do an initial hover stably. Then setup the filters and retune for best results.

The script attempts to tune all these parameters (in the given order)

- :ref:`ATC_RAT_RLL_D<ATC_RAT_RLL_D__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_RLL_P<ATC_RAT_RLL_P__AC_AttitudeControl_Multi>` and :ref:`ATC_RAT_RLL_I<ATC_RAT_RLL_I__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_PIT_D<ATC_RAT_PIT_D__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_PIT_P<ATC_RAT_PIT_P__AC_AttitudeControl_Multi>` and :ref:`ATC_RAT_PIT_I<ATC_RAT_PIT_I__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_YAW_D<ATC_RAT_YAW_D__AC_AttitudeControl_Multi>`
- :ref:`ATC_RAT_YAW_P<ATC_RAT_YAW_P__AC_AttitudeControl_Multi>` and :ref:`ATC_RAT_YAW_I<ATC_RAT_YAW_I__AC_AttitudeControl_Multi>`

The advantage over :ref:`AutoTune <autotune>` is that QuikTune is safer because the vehicle does not need to move or twitch.
The disadvantage is that QuikTune cannot find the vehicle's maximum rotational accelerations (e.g. :ref:`ATC_ACCEL_R_MAX <ATC_ACCEL_R_MAX>`, :ref:`ATC_ACCEL_P_MAX <ATC_ACCEL_P_MAX>`, :ref:`ATC_ACCEL_Y_MAX <ATC_ACCEL_Y_MAX>`)

Installing the Script
=====================

- Set :ref:`SCR_ENABLE <SCR_ENABLE>` = 1 to enable scripting and then reboot the autopilot
- Download `VTOL-quicktune.lua <https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5/libraries/AP_Scripting/applets/VTOL-quicktune.lua>`__ to your PC
- Copy the script to your autopilot's SD card's APM/scripts directory.  If using MP it may be easiest to use the Config, MAVFtp screen

  .. image:: ../images/quiktune-mp-mavftp.png
      :target: ../_images/quiktune-mp-mavftp.png
      :width: 450px

- Reboot the autopilot and set QUIK_ENABLE = 1
- If an RC switch will be used to start/stop the tune set RCx_OPTION = 300 where "x" is the RC input channel number.  Alternatively set one of Mission Planner's Aux Function tab's rows to "Scripting1"

  .. image:: ../images/quiktune-mp-auxfunction.png
      :target: ../_images/quiktune-mp-auxfunction.png
      :width: 450px

Running QuikTune
================

- Wait for a calm day and go to an open area with good GPS reception
- Connect with a ground station (e.g Mission Planner or QGC) and ensure that the Messages tab can be seen.  This is where output from the tune will appear
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

The full list of available `parameter settings are here <https://github.com/ArduPilot/ardupilot/blob/Copter-4.5/libraries/AP_Scripting/applets/VTOL-quicktune.md>`__
