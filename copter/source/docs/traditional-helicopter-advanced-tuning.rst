.. _traditional-helicopter-advanced-tuning:


======================================================================
Advanced Tuning for Hover Trim, Loiter Flight Mode and Waypoint Flying
======================================================================

At this point you should have completed the :ref:`basic tuning<traditional-helicopter-tuning>` and have a helicopter that is responsive and yet stable. But we need to trim the helicopter so it hovers pretty much hands-off in
Stabilize flight mode. And adjust the I-gains for Auto flight mode so it tracks
attitude properly under full autopilot control.

Hover Trim
==========

Trimming the helicopter in pitch and roll axes is an important step to keep the
aircraft from drifting in modes like Stabilize and Althold.  The trim attitude 
in the roll axis is affected by the tail rotor thrust.  All conventional single-
rotor helicopters with a torque-compensating tail rotor hover either right skid 
low or left skid low, depending on which way the main rotor turns. The 
ArduCopter software has a parameter, :ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>`, to compensate for this phenomenon. Longitudinal CG location will affect the trim attitude in the pitch
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
autopilot is "seeing". Adjust the :ref:`AHRS_TRIM_X<AHRS_TRIM_X>` and :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>` values so the autopilot "sees" the identical frame angle you measured with the digital pitch gauge. You can use the Level Horizon function in your ground station to level the horizon with the helicopter at actual level. That function will
make the adjustments to the AHRS_TRIM's for you.

The above is necessary so we can accurately measure the roll angle to set the
:ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>`. The autopilot now "knows" when the mainshaft is
perfectly vertical.

Load the helicopter with its normal payload, and hover the helicopter
in no-wind conditions in Stabilize flight mode. Land it and pull the log, noting
the roll angle that you had to hold with the stick to keep the helicopter from
drifting. Enter this value in the :ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>` parameter in centidegrees.
For a CW turning main rotor if it took 3.5 degrees of right roll to compensate,
enter 350. Negative values are for a CCW turning main rotor that requires left
roll to compensate.

..warning:: **Important Note** - do not use the radio trims at all. Make sure they are
at the same position as when the :ref:`common-accelerometer-calibration` was done. 

After setting the :ref:`ATC_HOVR_ROL_TRM<ATC_HOVR_ROL_TRM>` now hover the helicopter again. If it still
drifts make small adjustments to the :ref:`SERVO1_TRIM<SERVO1_TRIM>` , :ref:`SERVO2_TRIM<SERVO2_TRIM>` and :ref:`SERVO3_TRIM<SERVO3_TRIM>` .
The chances of getting the swashplate perfectly level during bench setup is very
low and this dynamic tuning is needed to trim the helicopter. If it requires
large deviation from your original ``SERVOx_TRIM`` values it is likely you have a CG
problem, or your initial setup when leveling the swashplate was not very
accurate.

Your helicopter is now trimmed properly. This trimming procedure makes the
difference between a helicopter that is difficult to handle vs one that flies
with true scale quality and handling. 

Adjusting I-gains For High-Speed Autonomous Flight
==================================================

Prepare a mission with your ground station software that will fly the 
helicopter, preferably in a figure-8 pattern to make both right and left turns,
at a speed of 6 m/s. Fly the helicopter on this mission, pull the logs from the
microSD card and look at the AHRS desired vs actual pitch, roll and yaw
attitudes in dynamic flight. They should track within 1-2 degrees. If they do
not, increase the ``ATC_RAT_xxx_I`` value for that axis until they do.

Now, fly the same mission, but at higher speed of 9-10 m/s, and analyze the logs
the same way. Make further adjustments to the I-gains and IMAX values as
required. It is not clear what I-gain values will be required as no two
helicopters are the same. But I-gain values from 0.25 - 0.38 are common in pitch
and roll, and 0.18 - 0.30 in yaw. IMAX values of 0.40 - 0.45 are common, however
refer to the 'Setting the I gain, IMAX, and ILMI' section on how to determine
what the IMAX value should be.

Preventing I term Build Up
==========================

A downside of the I term is that it can build if the vehicle is constrained in its movement, so as not to be able to respond and reduce any long term errors, as when still on the ground. The I term can build to its maximum value and could possibly tip over the vehicle. Two methods are provided to avoid this.

The first is by restricting by providing a "leak" mechanism (constantly reduced at a specified rate) when on the ground or hover(actually when moving with a GPS speed of less than 5m/s). To prevent the I term from completely disappearing at low speeds or hover, the parameters :ref:`ATC_RAT_RLL_ILMI<ATC_RAT_RLL_ILMI>` and :ref:`ATC_RAT_PIT_ILMI <ATC_RAT_PIT_ILMI>`, only lets it leak off so much. So the integrator leak along with the leak minimum parameter keep 
enough of the integrator to make it effective in keeping the attitudes matching
but not so powerful to cause the aircraft to roll over.

The second is to use ArduPilot's landing and takeoff detection algorithms to zero the I terms when on the ground, and allow it to operate normally after takeoff.

Which method is used is determined by the :ref:`H_OPTIONS<H_OPTIONS>` bitmask. Setting bit 0 (ie "1" for the parameter value) will use the first method with a leaky integrator. Since this was the only method used in versions prior to 4.1, it is the default method to avoid changes in behavior upon upgrading to version 4.1. But clearing that bit (value = "0"), will use the second method, which is now usually preferred.