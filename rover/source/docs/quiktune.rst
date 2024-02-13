.. _quiktune:

========
QuikTune
========

..  youtube:: yGB9uLD4dkM
    :width: 100%

The `Rover QuikTune <https://github.com/ArduPilot/ardupilot/blob/Rover-4.5/libraries/AP_Scripting/applets/rover-quicktune.md>`__ Lua script simplifies the process of finding good gains for a vehicle's steering and speed controllers.

The script should be run while the vehicle is circling in :ref:`Circle mode <circle-mode>`.
It records the steering and throttle outputs and the resulting response (e.g. vehicle's turn rate and speed) for at least 10 seconds in order to calculate the feed-forward gains.
The P and I gains are then set to a ratio of the feed-forward's.
Once complete the gains are saved automatically.

By default the gains will be tuned in this order:

- :ref:`ATC_STR_RAT_FF <ATC_STR_RAT_FF>`, then :ref:`ATC_STR_RAT_P <ATC_STR_RAT_P>` and I are set to ratios of the FF
- :ref:`CRUISE_SPEED <CRUISE_SPEED>` and :ref:`CRUISE_THROTTLE <CRUISE_THROTTLE>`, then :ref:`ATC_SPEED_P <ATC_SPEED_P>` and I are set to ratios of the FF

The script will also adjust filter settings:

- :ref:`ATC_STR_RAT_FLTD <ATC_STR_RAT_FLTD>` and FLTT will be set to half of the :ref:`INS_GYRO_FILTER <INS_GYRO_FILTER>` value

Installing the Script
=====================

- Set :ref:`SCR_ENABLE <SCR_ENABLE>` = 1 to enable scripting and then reboot the autopilot
- Download `rover-quicktune.lua <https://raw.githubusercontent.com/ArduPilot/ardupilot/Rover-4.5/libraries/AP_Scripting/applets/rover-quicktune.lua>`__ to your PC
- Copy the script to your autopilot's SD card's APM/scripts directory.  If using MP it may be easiest to use the Config, MAVFtp screen

  .. image:: ../images/quiktune-mp-mavftp.png
      :target: ../_images/quiktune-mp-mavftp.png
      :width: 450px

- Reboot the autopilot and set RTUN_ENABLE = 1
- If an RC switch will be used to start/stop the tune set RCx_OPTION = 300 where "x" is the RC input channel number.  Alternatively set one of Mission Planner's Aux Function tab's rows to "Scripting1"

  .. image:: ../images/quiktune-mp-auxfunction.png
      :target: ../_images/quiktune-mp-auxfunction.png
      :width: 450px

Running QuikTune
================

- Go to an open area with good GPS reception
- Connect with a ground station (e.g Mission Planner or QGC) and ensure that the Messages tab can be seen.  This is where output from the tune will appear
- Move the RC switch to the low position OR push MP's Aux Function's "Low" button
- Optionally set :ref:`CIRC_SPEED <CIRC_SPEED>` to about half the vehicle's maximum speed
- Arm the vehicle, drive to an open area and switch to :ref:`Circle mode <circle-mode>`
- Begin the tune by moving the RC switch to the middle position OR push MP's Aux Function's "Mid" button
- Monitor the progress of the tune using the GCS's Messages tab
- If the RC steering and throttle sticks are moved from their center position the tuning will be paused until a few seconds after they are moved back to their center position
- If the vehicle begins oscillating violently cancel the tune by moving the RC switch to the low position OR push MP's Aux Function's "Low" button
- Once the tune has completed the new gains will be saved automatically
- Drive the vehicle home and disarm

Advanced Configuration
======================

The full list of available `parameter settings are here <https://github.com/ArduPilot/ardupilot/blob/Rover-4.5/libraries/AP_Scripting/applets/rover-quicktune.md>`__
