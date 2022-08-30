.. _traditional-helicopter-first-flight-tests:

==================
First Flight Tests
==================


First Spool Up
==============

Check the CG of helicopter. It should be exactly on the point of the main shaft. If not, adjust battery until it is.

For the first spool up, before trying to lift into hover, you should verify that the main rotor blades are tracking reasonably well. If not, excessive vibration will be created, perhaps high enough to cause attitude control errors or even cause an :ref:`ekf-inav-failsafe`.

- Arm the vehicle. If there is a pre-arm failure, resolve it before proceeding. Usually you will need to wait until the GPS obtains lock and the EKF sets its origin as indicated by the ground control station messages (and, of course, the failure to arm!)
- Before the automatic disarming timing elapses, engage the Motor Interlock switch and/or raise the collective above 1/4 stick (if using passthru RSC mode)to enable Motor Interlock. The rotor should spool up to speed. Usually, one spools up at 0 deg collective pitch setting to avoid stress while on the ground
- At this point, check that the blades are tracking. If not, adjust a blade's pitch link to the swashplate and re-try until the plane of both blades is close to identical.

.. youtube:: 3g3hJtBhSJ4

.. tip:: See :ref:`Traditional Helicopter Tips <traditional-helicopter-tips>` for more on vibration issues.

First Hover
===========

Once tracking has been verified, you can do your first test hover. In STABILIZE, repeat the above, but once the rotor has finished spooling up, increase collective slowly and lift off. Be prepared to  put in small cyclic corrections to hover the vehicle in place. If the vehicle seems too unstable in attitude, you might try setting the PID values to those used for beginning the fine tuning of the attitude control loops, discussed in the following section (:ref:`traditional-helicopter-tuning` ).

Hover Trim
==========

Trimming the helicopter in pitch and roll axes is an important step to keep the
aircraft from drifting in modes like Stabilize and Althold.  The trim attitude 
in the roll axis is affected by the tail rotor thrust.  All conventional single-
rotor helicopters with a torque-compensating tail rotor hover either right skid 
low or left skid low, depending on which way the main rotor turns. The 
ArduCopter software has a parameter, :ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>`, to compensate for this phenomenon. 
Dual rotor helicopters have counter rotating rotor systems which mostly cancel the torque.  For these aircraft,
the :ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>` should be set to zero.
Longitudinal CG location will affect the trim attitude in the pitch
axis.  There is no parameter to tell the autopilot what pitch attitude 
the aircraft hovers with no drift. It always targets zero deg pitch as measured
by the autopilot. Therefore the actual pitch attitude the aircraft 
hovers may be 5 deg nose high but the autopilot AHRS Trim value is set
to make it think the attitude is zero deg. 

In order to trim the aircraft, set the :ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>` parameter to zero. 
During the initial setup of the autopilot, the ``AHRS_TRIM_x`` values are set 
during the accelerometer calibration on the last step that has you level the 
aircraft. For that step you should have made certain that the shaft was 
perfectly straight up in pitch and roll. For this trim procedure, it is 
recommended that you check it and using the method below.

Measure the actual frame angle (on a portion of the frame that is perpendicular 
to the mainshaft) in pitch and roll with your digital pitch gauge. Connected to 
your ground station software with MavLink, note the pitch and roll angle the 
autopilot is "seeing". Adjust the :ref:`AHRS_TRIM_X<AHRS_TRIM_X>` and :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>` 
values so the autopilot "sees" the identical frame angle you measured with the digital pitch gauge. 
You can use the Level Horizon function in your ground station to level the horizon with the helicopter 
at actual level. That function will make the adjustments to the AHRS_TRIM's for you.

The above is necessary so we can accurately measure the roll angle to set the
:ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>`. The autopilot now "knows" when the mainshaft is
perfectly vertical.

Load the helicopter with its normal payload, and hover the helicopter
in no-wind conditions in Stabilize flight mode. Land it and pull the log, noting
the roll angle and pitch angle that you had to hold with the stick to keep the helicopter from
drifting. Enter the roll angle value in the :ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>` parameter in centidegrees.
For a CW turning main rotor if it took 3.5 degrees of right roll to compensate,
enter 350. Negative values are for a CCW turning main rotor that requires left
roll to compensate. If the pitch angle was not zero to keep the helicopter from drifting, then enter the pitch attitude 
required for no drift in the :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>` in radians (which is the angle in degrees divided by 57.3). 
If it took 2 degrees nose down (-2 deg) to hover then you would enter -0.0349 radians into :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>`.

..  warning:: Do not use the radio trims at all. Make sure they are at the same position as when the :ref:`common-radio-control-calibration` was done. 

Your helicopter is now trimmed properly. If you hover the aircraft and it still drifts, check that the aircraft is 
actually holding the hover attitude determined above with the stick centered.  If it does not, then consult the 
Setting ILMI and IMAX section of the :ref:`Additional Tuning Topics <traditional-helicopter-tuning-other-topics>` wiki. 
This trimming procedure makes the difference between a helicopter 
that is difficult to handle vs one that flies with true scale quality and handling. 

Enable Notch Filtering
======================

Assuming that first low in-place hover was successful, which it should be if you have followed the wiki, you should setup a dynamic harmonic notch filter. Tuning of the control loops, which is the next step, is greatly enhanced if the vehicle's generated vibrations are attenuated by the filter.

Follow the instructions in :ref:`Helicopter Dynamic Notch Filter Setup<common-imu-notch-filtering-helicopter-setup>`.

.. note:: There usually are two rotational vibration sources in Traditional Helicopters, the main rotor and the tail rotor. The tail rotor frequency may appear as if its a harmonic of the main rotor, but not quite at an integer multiple of the main rotor fundamental frequency. For example, if the tail rotor has a 1:3.8 gearing to the main rotor, it will appear near what would be the 4th harmonic of the main rotor, but is slightly lower (ie 3.8x the fundamental of the main rotor). In this case, the harmonic notch setup for the 4th harmonic may still be effective if the bandwidth of the notches, :ref:`INS_HNTCH_BW<INS_HNTCH_BW>`,are wide enough.

After the harmonic notch is setup, proceed to :ref:`traditional-helicopter-tuning`.
