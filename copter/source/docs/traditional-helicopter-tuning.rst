.. _traditional-helicopter-tuning:

===============================
Traditional Helicopter – Tuning
===============================

This tuning guide is applicable to all versions of ArduCopter for traditional
helicopters. However, it is written for Copter 3.4 and newer. Since the names of
parameters, and scaling, were changed from Copter 3.3 to 3.4 the
:ref:`old tuning guide is archived here <traditional-helicopter-archived-
tuning>`

General ArduCopter Flight Control Law Description
===================================================
Users should generally understand the flight control laws before tuning. At
a high level, the arducopter control laws are designed as a model following
architecture where the software converts the pilot input into a commanded
attitude (Stabilize Mode) or commanded rate (Acro mode) and controls the
aircraft to achieve that commanded value. In the background, the software keeps
track of or predicts where the aircraft should be in space (i.e. pitch and roll
attitude) based on the inputs of the pilot or autopilot. It has two controllers
(attitude and rate) that work together to ensure the actual aircraft is
following the software’s predicted pitch and roll rates and attitudes.
 
The pilot’s commands are limited by the amount of acceleration that can be
commanded through the ATC_ACCEL_P_MAX for pitch and ATC_ACCEL_R_MAX for roll.
The initial responsiveness (crispness/sluggishness) of the aircraft to the pilot
input can be adjusted through the RC_FEEL parameter. The pilot input and these
parameters are used to determine the requested rate required to achieve the
desired response that is fed to the rate controller.
 
The attitude controller is used to ensure the actual attitude of the aircraft
matches the predicted attitude of the flight controller. It uses the
ATC_ANG_PIT_P in pitch and the ATC_ANG_RLL_P in roll to determine a rate that is
fed to the rate controller that will drive the aircraft to the predicted
attitude. 

The rate controller receives the sum of the requested rate resulting
from the pilot input and the rate from the attitude controller and determines
the swashplate commands required to achieve the input rate. The rate controller
uses a PID control algorithm and a feed forward path to control the aircraft and
achieve the input rate. The feed forward path uses the input rate and applies
the ATC_RATE_PIT_VFF gain for pitch and ATC_RATE_RLL_VFF gain for roll to
determine its portion of the swashplate command. The PID algorithm uses the
error between the actual rate and input rate to determine its portion of the
swashplate command. These are summed and sent to the mixing unit where the servo
positions are determined.

So this tuning method uses the VFF gain initially to ensure the requested rates
match the actual rates.  However the rates can vary from the requested due to
disturbances.  The P and D gains are then used to guard against disturbances
that cause the actual rates to deviate from the requested rates. So the P and D
gain may not be able to keep the actual rates exactly matching the requested
rates.  Since the software is tracking where the orientation of the aircraft
should be, then any error between the requested and actual rates will result in
attitude error. So there is a feature called the integrator that continually
sums the rate errors which effectively calculates the error in attitude.  The
I gain is multiplied by the integrator and summed with the other outputs of the
rate controller.  The integrator is limited by the ATC_RATE_RLL_IMAX in roll and
ATC_RATE_PIT_IMAX in pitch.  When ground speed is less than 5 m/s, the
integrator is leaked off (reduced at a specified rate) and another parameter, 
ATC_RATE_RLL_ILMI and ATC_RATE_PIT_ILMI, only lets it leak off so much.  If the 
ILMI, or integrator leak minimum, is zero then the integrator will not be 
allowed to grow and the attitude will not be driven to exactly match the 
software’s predicted attitude.  However, if this is non zero or large enough for
attitude errors that may be encountered at low speeds and in a hover then the 
actual attitude will track the predicted attitude.  The reason for the leak and 
ILMI parameter is that a larger amount of integrator is needed for forward 
flight however in a hover and in particular during air ground transition 
allowing large amounts of integrator can cause the aircraft to flip itself on
its side.  So the integrator leak along with the leak minimum parameter keep 
enough of the integrator to make it effective in keeping the attitudes matching
but not so powerful to cause the aircraft to roll over.

Initial Setup of Pitch and Roll Tuning Parameters
===================================================
Below are the initial parameters values that should be used to start the tuning
of your helicopter.  Use the suggested parameters in the section below for the
tail until the pitch and roll axes are tuned. The helicopter will be easily 
controllable with just the FF set to 0.15 on pitch and roll in the event that
you need to modify the tail settings from what was suggested.  

+---------------------+---------+
| ATC_ACCEL_P_MAX     | 90000   |
+---------------------+---------+
| ATC_ACCEL_R_MAX     | 90000   |
+---------------------+---------+
| ATC_ANG_PIT_P       | 4.5     |
+---------------------+---------+
| ATC_ANG_RLL_P       | 4.5     |
+---------------------+---------+
| ATC_RAT_PIT_D       | 0       |
+---------------------+---------+
| ATC_RAT_PIT_FILT    | 20      |
+---------------------+---------+
| ATC_RAT_PIT_I       | 0       |
+---------------------+---------+
| ATC_RAT_PIT_ILMI    | 0       |
+---------------------+---------+
| ATC_RAT_PIT_IMAX    | 0.40    |
+---------------------+---------+
| ATC_RAT_PIT_P       | 0       |
+---------------------+---------+
| ATC_RAT_PIT_VFF     | 0.15    |
+---------------------+---------+
| ATC_RAT_RLL_D       | 0       |
+---------------------+---------+
| ATC_RAT_RLL_FILT    | 20      |
+---------------------+---------+
| ATC_RAT_RLL_I       | 0       |
+---------------------+---------+
| ATC_RAT_RLL_ILMI    | 0       |
+---------------------+---------+
| ATC_RAT_RLL_IMAX    | 0.40    |
+---------------------+---------+
| ATC_RAT_RLL_P       | 0       |
+---------------------+---------+
| ATC_RAT_RLL_VFF     | 0.15    |
+---------------------+---------+
| RC_FEEL             | 50      |
+---------------------+---------+

Setting VFF and ACCEL_MAX for Desired Pitch and Roll Response
===============================================================
In both pitch and roll axes, the VFF gain is set so that the actual aircraft
rate matches the desired rate. To do this, the RATE message in the log is
required to compare the P.des and P signals for pitch and the R.des and R
signals for roll. With the VFF gains set to 0.15, takeoff and establish a hover,
then make some sharp stick inputs in both pitch and roll. Land and pull the log
and look at the signals. If the actual rate is more than the desired rate then
you'll want to decrease VFF and if it is less, increase VFF. If the desired and
actual rates are offset by some amount, it means that your swash was not
properly leveled in the setup or the cg is not right.  In this case, just make
sure the change in rate is similar between desired and actual.  Once you get the
rates to match and they feel like they are too fast then you can reduce the
ATC_ACCEL_MAX parameter and repeat the process above to match the desired and
actual rates.  

If while tuning the VFF gain the aircraft starts to oscillate, reduce the 
ATC_ANG_XXX_P gain for that axis until the oscillations stop.  However for most 
helicopteters the suggested values above should not cause this problem.

With a flybar head where the linkage rate is normally lower, it is recommended
to start with 0.22 VFF for both pitch and roll and you will likely have to go
higher with VFF. But for a flybarless head, VFF shouldn't be more than 0.22 
unless you have really really slow servos or slow linkage rate. With all 
helicopters the VFF gain compensates for differences in servo and linkage speed. 

The final setting for ATC_ACCEL_MAX parameters will depend on the size of the
helicopter.  Large 800-900 class machines will typically be in the 36000-52000 
range; smaller 450-500 class machines will typically be in the 90000-110000 
range. You may want to experiment with the RC_FEEL parameter as well to get the
initial aircraft response the way you like it.  It is recommended to keep the
RC_FEEL parameter between 25 and 50. Once this process is complete, the aircraft
should have the desired feel in snappiness and rate.

Below is a graph showing an example of Rate Roll Desired vs actual Rate Roll.
The peak corresponds to a rapid stick input and the amplitude (height) of the
peaks should be approximately the same with no more than 100 milliseconds 
offset.

.. image:: ../images/TradHeli_tuning_example1_1.png

Tuning the D and P gain
=========================
Once you have the heli responding nicely with the rate VFF gain, now tune the
PID gains. The rate PID controller provides stability to reject disturbances and
keep the actual aircraft following the software predicted rates.
 
Start with the D gain.  Use the tuning feature of ArduCopter which is linked to
channel 6 on your radio.  Make the following parameter changes.
TUNING = 21
TUNING_LOW = 0
TUNING_HIGH = 30 (for futaba radios this equates to one increment in the knob to
0.001)

Adjust the tuning knob until the ATC_RATE_RLL_D and ATC_RATE_PIT_D gains are
0.001. Lift into a hover and make some sharp stick inputs in roll.  Most
helicopters will see roll oscillations before they see pitch oscillations.
That is why roll inputs are suggested.  If it doesn't shake, increase the gain
by 0.001 and try it again. At the value where you get the rapid shaking, cut
that value in half and enter it as the final tuning value for ATC_RATE_RLL_D and
ATC_RATE_PIT_D.  Test hover the heli and make some rapid stick movements in both
pitch and roll to make sure it's stable.

Now tune the P gains.  Make the following tuning parameter changes.
TUNE = 4
TUNE_LOW = 0
TUNE_HIGH = 300 (for futaba radios this equates to one increment in the knob to
0.01)

Adjust the tuning knob until the ATC_RATE_RLL_P and ATC_RATE_PIT_P  gains are
0.05. Lift into a hover and roll aggressively from side to side.  If it doesn't
shake, increase the gain by 0.01 and try it again. At the value where you get
the rapid shaking, cut that value in half and enter it as the final tuning value
for ATC_RATE_RLL_P and ATC_RATE_PIT_P.  Test hover the heli and make some rapid
stick movements in both pitch and roll to make sure it's stable.  

After tuning the P and D gain the aircraft should feel much smoother.

Setting the I gain, IMAX, and ILMI
====================================
It is recommended to set the ATC_RATE_PIT_I gain equal to the ATC_RATE_PIT_VFF
gain and the ATC_RATE_RLL_I gain equal to the ATC_RATE_RLL_VFF gain.  The IMAX
value limits amount of integrator error that can be stored to counter large
disturbances in attitude.  In the pitch axis this is set by the integrator error
required to hold the aircraft attitude at high forward speeds.  The starting
value is 0.4.  To check this set the value to IMAX = 1, fly the aircraft at the
maximum desired speed.  Pull the log and look at what the maximum I value is in
the PIDP message.  Set IMAX for 0.1 above the maximum value.  You could do the
same for the roll axis but typically 0.4 should be sufficient.  ILMI is set for
the maximum amount of integrator that you want to retain in a hover to help
maintain attitude.  It is recommended that this value is no larger than 0.1

Below is a graph of desired roll attitude vs actual roll attitude for a
helicopter in high-speed autonomous flight with the ILMI parameters set to zero.
The effect of the I-gain and IMAX parameters, properly set, will make the
helicopter track the desired attitude very closely at speed exceeding 5m/s for
more than 2 seconds (what we call “dynamic flight”). It should be within 1-2
degrees of desired in dynamic flight. Towards the right side of the graph the
helicopter came to a stop in hover and the pilot switched to Stabilize flight
mode. You will notice a discrepancy between the actual and desired roll attitude
at that point. This is the effect of having ILMI set to zero. The ILMI can be
considered to be a sort of “auto trim” for hover that will reduce the
discrepancy between desired and actual pitch and roll attitude when the
helicopter is not in dynamic flight.

.. image:: ../images/TradHeli_tuning_example2_1.png

Tuning the Yaw Axis (Rudder)
====================================
The yaw axis can be tuned using the method provided above for the pitch and roll
axes. However it is recommended to get the tuning complete for the pitch and 
axes before tuning the yaw axis with this method.  A good set of paramters for 
the yaw axis to start with are given below.  

+---------------------+---------+
| ATC_ACCEL_Y_MAX     | 36000   |
+---------------------+---------+
| ATC_ANG_YAW_P       | 4.5     |
+---------------------+---------+
| ATC_RAT_YAW_D       | ??      |
+---------------------+---------+
| ATC_RAT_YAW_FILT    | 20      |
+---------------------+---------+
| ATC_RAT_YAW_I       | ??      |
+---------------------+---------+
| ATC_RAT_YAW_ILMI    | ??      |
+---------------------+---------+
| ATC_RAT_YAW_IMAX    | ??      |
+---------------------+---------+
| ATC_RAT_YAW_P       | ??      |
+---------------------+---------+
| ATC_RAT_YAW_VFF     | ??      |
+---------------------+---------+
