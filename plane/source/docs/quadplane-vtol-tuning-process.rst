.. _quadplane-vtol-tuning-process:

===========================
Tuning Process Instructions
===========================

Setting the aircraft up ready for tuning
----------------------------------------

The following parameters should be set correctly based on the specifications of your aircraft.

Battery setting
^^^^^^^^^^^^^^^
Parameters used to linearize your motor thrust curve.

- :ref:`Q_M_BAT_VOLT_MAX <Q_M_BAT_VOLT_MAX>` : 4.2v x No. Cells
- :ref:`Q_M_BAT_VOLT_MIN <Q_M_BAT_VOLT_MIN>` : 3.3v x No. Cells
- :ref:`Q_M_THST_EXPO <Q_M_THST_EXPO>` : 0.55 for 5 inch props, 0.65 for 10 inch props, 0.75 for 20 inch props. This parameter can be derived by thrust stand measurements for optimum results (don’t trust manufacturer data). See :ref:`motor-thrust-scaling` for details

.. image:: ../images/tuning-process-instructions-1.hires.png
    :target: ../_images/tuning-process-instructions-1.hires.png

Motors setup
^^^^^^^^^^^^
Parameters used to define the output range sent to the ESC.

- :ref:`Q_M_PWM_MAX <Q_M_PWM_MAX>` : Check ESC manual for fixed range or 2000us
- :ref:`Q_M_PWM_MIN <Q_M_PWM_MIN>` : Check ESC manual for fixed range or 1000us
- :ref:`Q_M_SPIN_ARM <Q_M_SPIN_ARM>` : use the :ref:`motor test feature <connect-escs-and-motors_testing_motor_spin_directions>`
- :ref:`Q_M_SPIN_MAX <Q_M_SPIN_MAX>` : 0.95
- :ref:`Q_M_SPIN_MIN <Q_M_SPIN_MIN>` : use the :ref:`motor test feature <connect-escs-and-motors_testing_motor_spin_directions>`
- :ref:`Q_M_THST_HOVER <Q_M_THST_HOVER>` : 0.25, or below the expected hover thrust percentage (low is safe)

PID Controller Initial Setup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- :ref:`INS_ACCEL_FILTER <INS_ACCEL_FILTER>` :  10Hz to 20Hz
- :ref:`INS_GYRO_FILTER <INS_GYRO_FILTER>` : 80Hz for 5 inch props, 40Hz for 10 inch props, 20Hz for 20 inch props
- :ref:`Q_A_ACCEL_P_MAX <Q_A_ACCEL_P_MAX>` : 110000 for 10 inch props, 50000 for 20 inch props, 20000 for 30 inch props
- :ref:`Q_A_ACCEL_R_MAX <Q_A_ACCEL_R_MAX>` : 110000 for 10 inch props, 50000 for 20 inch props, 20000 for 30 inch props
- :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>` : 27000 for 10 inch props, 18000 for 20 inch props, 9000 for 30 inch props
- :ref:`Q_A_RAT_YAW_P <Q_A_RAT_YAW_P>` : 0.5 x :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>` / 4500


- :ref:`Q_A_RAT_PIT_FLTD <Q_A_RAT_PIT_FLTD>` : :ref:`INS_GYRO_FILTER <INS_GYRO_FILTER>` / 2
- :ref:`Q_A_RAT_PIT_FLTT <Q_A_RAT_PIT_FLTT>` : :ref:`INS_GYRO_FILTER <INS_GYRO_FILTER>` / 2
- :ref:`Q_A_RAT_RLL_FLTD <Q_A_RAT_RLL_FLTD>` : :ref:`INS_GYRO_FILTER <INS_GYRO_FILTER>` / 2
- :ref:`Q_A_RAT_RLL_FLTT <Q_A_RAT_RLL_FLTT>` : :ref:`INS_GYRO_FILTER <INS_GYRO_FILTER>` / 2
- :ref:`Q_A_RAT_YAW_FLTE <Q_A_RAT_YAW_FLTE>` : 2
- :ref:`Q_A_RAT_YAW_FLTT <Q_A_RAT_YAW_FLTT>` : :ref:`INS_GYRO_FILTER <INS_GYRO_FILTER>` / 2



.. image:: ../images/tuning-process-instructions-2.hires.png
    :target: ../_images/tuning-process-instructions-2.hires.png

.. image:: ../images/tuning-process-instructions-3.hires.png
    :target: ../_images/tuning-process-instructions-3.hires.png

.. image:: ../images/tuning-process-instructions-4.hires.png
    :target: ../_images/tuning-process-instructions-4.hires.png

The initial tune of the aircraft should be done **in the aircrafts most agile configuration**. This generally means that the aircraft will be at its minimum take off weight with fully charged batteries.

Pilot's preparation for first flight
------------------------------------

The first takeoff of an untuned VTOL vehicle is the most dangerous seconds of the aircraft’s life. This is where the aircraft could be very unstable causing a sudden increase in power when then results in the aircraft jumping into the air, or it may be so poorly tuned that you have insufficient control over the aircraft once it is airborne. The pilot should be extremely diligent during the tuning flights to avoid a situation that could result in injury or damage.

There are several things that the pilot can do to minimize the risk during the early tuning process:

1. The pilot should conduct a motor number and orientation check (see :ref:`Checking the motor numbering with the Mission Planner Motor test <connect-escs-and-motors_testing_motor_spin_directions>`). Care should be taken to ensure that the correct frame type is selected. Incorrect frame type can result in a very fast yaw rotation or complete loss of control. Take note of the output percentage required to spin the propellers and ensure that:

- :ref:`Q_M_SPIN_ARM <Q_M_SPIN_ARM>` is set high enough to spin the motors cleanly.
- :ref:`Q_M_SPIN_MIN <Q_M_SPIN_MIN>` is set high enough to spin the motors win a minimal level of thrust.

2. All flights after a significant tuning change should be done in QSTABILIZE. QSTABIILIZE provides the pilot with significantly more control over the aircraft in the event that the attitude controllers are unstable.
3. The pilot should not take off in QHOVER until the altitude controller has been tested in flight. This should be done by taking off in QSTABILIZE and switching to QHOVER. Although QHOVER is rarely a problem unless the aircraft has a very low hover throttle.
4. For the initial flights the pilot should ensure that these parameters are set:

- :ref:`Q_A_THR_MIX_MAN <Q_A_THR_MIX_MAN>` to 0.1
- :ref:`Q_M_THST_HOVER <Q_M_THST_HOVER>` to 0.25 (or lower than the expected hover throttle)

5. Use a radio and calibrate the radio correctly (see :ref:`common-radio-control-calibration`).
6. Configure an ARM/DISARM switch and test it (see :ref:`Auxiliary Functions <common-auxiliary-functions>`).
7. Do tuning flights in low-wind condition and normal weather (no rain and between 15°C/59°F and 25°C/77°F).
8. Practice QSTABILIZE flight in simulator or on a low-end drone first. You should be confident to be able to takeoff and land with your untuned aircraft.


First Flight
------------

The first take off is the most dangerous time for any QuadPlane. Care must be taken to ensure the aircraft is not destroyed in the first seconds of flight and nobody is injured.

- **Ensure that all spectators are at a safe distance**.
- **Ensure the pilot is at a safe distance and position**.
- The pilot should refresh themselves on the method used to disarm the aircraft (using :ref:`Auxiliary Functions <common-auxiliary-functions>` for Motor Interlock or Arm/Disarm may be beneficial).

This flight will allow to setup your aircraft in a "flyable for tuning" state.

1. Ensure the aircraft is in QSTABILIZE mode
2. Arm the aircraft
3. Immediately disarm the aircraft to ensure your disarm procedure is correct
4. Arm the aircraft
5. Slowly increase the throttle looking for signs of oscillation. (long or flexible landing gear may cause some landing gear oscillation that will only go away after the aircraft leaves the ground)
6. As soon as the aircraft lifts off the ground immediately put the aircraft back down as gently as possible
7. Disarm the aircraft
8. Evaluate what you observed to decide if you need to make adjustments to the tuning parameters or if it is safe to take off again
9. Arm and increase the throttle to initiate a takeoff
10. Hover at approximately 1m altitude and apply small (5 degrees) control inputs into roll and pitch
11. Immediately land if any oscillation is observed

Next section will explain how to remove the oscillations.

Initial aircraft tune
---------------------

The first priority when tuning a QuadPlane is to establish a stable tune, free of oscillations, that can be used to do further tests.

1. Arm the aircraft in QSTABILIZE
2. Increase the throttle slowly until the aircraft leaves the ground
3. If the aircraft starts to oscillate immediately abort the takeoff and/or land the aircraft, and:
4. Reduce all the following parameters by 50%

a. :ref:`Q_A_RAT_PIT_P <Q_A_RAT_PIT_P>`
b. :ref:`Q_A_RAT_PIT_I <Q_A_RAT_PIT_I>`
c. :ref:`Q_A_RAT_PIT_D <Q_A_RAT_PIT_D>`
d. :ref:`Q_A_RAT_RLL_P <Q_A_RAT_RLL_P>`
e. :ref:`Q_A_RAT_RLL_I <Q_A_RAT_RLL_I>`
f. :ref:`Q_A_RAT_RLL_D <Q_A_RAT_RLL_D>`

This process is repeated until the aircraft can hover without oscillations being detectable visually or audibly.

If the aircraft has very long or flexible landing gear then you may need to leave the ground before ground resonance stops.

Be aware that in this state the aircraft may be very slow to respond to large control inputs and disturbances. The pilot should be extremely careful to put minimal stick inputs into the aircraft to avoid the possibility of a crash.

Test QHOVER
-----------

This test will allow to test the altitude controller and ensure the stability of your aircraft.

1. Check :ref:`Q_M_HOVER_LEARN <Q_M_HOVER_LEARN>` is set to 2. This will allow the controller to learn by itself the correct hover value when flying.

2. Take off in QSTABILIZE and increase altitude to 5m. Switch to QHOVER and be ready to switch back to QSTABILIZE. If the aircraft is hovering at a very low hover throttle value you may hear a reasonably fast oscillation in the motors. Ensure the aircraft has spent at least 30 seconds in hover to let the hover throttle parameter converge to the correct value. Land and disarm the aircraft.

3. Set these parameters on ground and preferably disarm  (A confident pilot could set them in flight with GCS):

  - :ref:`Q_P_ACCZ_I <Q_P_ACCZ_I>` to 2 x :ref:`Q_M_THST_HOVER <Q_M_THST_HOVER>`
  - :ref:`Q_P_ACCZ_P <Q_P_ACCZ_P>` to :ref:`Q_M_THST_HOVER <Q_M_THST_HOVER>`

 If the QuadPlane in QHOVER starts to move up and down, the vertical position and velocity controllers may need to be reduced by 50%. These values are: :ref:`Q_P_POSZ_P <Q_P_POSZ_P>` and :ref:`Q_P_VELZ_P <Q_P_POSZ_P>`.

Evaluating the aircraft tune
----------------------------

Most pilots will look to move to :ref:`QAUTOTUNE<qautotune-mode>` as quickly as possible once their aircraft can hover safely in QHOVER. Before QAUTUTUNE mode is run, the pilot should ensure that the current tune is good enough to recover from the repeated tests run in QAUTOTUNE mode. To test the current state of tune:

1. Take off in QHOVER or QSTABILIZE
2. Apply small roll and pitch inputs. Start with 5 degree inputs and releasing the stick to centre, pitch, left, right, roll forward back, then all 4 points on the diagonal
3. Increase inputs gradually to full stick deflection
4. Go to full stick deflection, quickly momentarily, and let the sticks spring back to centre

If the aircraft begins to overshoot significantly or oscillate after the stick input, halt the tests before the situation begins to endanger the aircraft. The aircraft may require manual tuning described below before QAUTOTUNE can be run.

To test the stabilization loops independent of the input shaping, set the parameter: :ref:`Q_A_RATE_FF_ENAB <Q_A_RATE_FF_ENAB>` to 0.

1. Take off in QHOVER or QSTABILIZE
2. Hold a roll or pitch input
3. Release the stick and observe the overshoot as the aircraft levels itself
4. Gradually increase the stick deflection to 100%

Halt the tests if the aircraft overshoots level significantly or if the aircraft oscillates, the aircraft may require manual tuning (:ref:`see next section <ac_rollpitchtuning>`) before QAUTOTUNE can be run.

Set :ref:`Q_A_RATE_FF_ENAB <Q_A_RATE_FF_ENAB>` to 1 after the tests are complete.

Manual tuning of Roll and Pitch
-------------------------------

Manual tuning may be required to provide a stable tune before QAUTOTUNE is run, or if QAUTOTUNE does not produce an acceptable tune. The process below can be done on roll and pitch at the same time for a quick manual tune provided the aircraft is symmetrical. If the aircraft is not symmetrical then the process should be repeated for both roll and pitch individually.

The pilot should be especially careful to ensure that :ref:`Q_A_THR_MIX_MAN <Q_A_THR_MIX_MAN>` and :ref:`Q_M_THST_HOVER <Q_M_THST_HOVER>` are set correctly before manual tuning is started.

When oscillations start do not make large or sudden stick inputs. Reduce the throttle smoothly to land the aircraft while using very slow and small roll and pitch inputs to control the aircraft position.

1. Increase the D term in steps of 50% until oscillation is observed
2. Reduce the D term in steps of 10% until the oscillation disappears
3. Reduce the D term by a further 25%
4. Increase the P term in steps of 50% until oscillation is observed
5. Reduce the P term in steps of 10% until the oscillation disappears
6. Reduce the P term by a further 25%

Each time the P term is changed set the I term equal to the P term. Those parameters can be changed on ground and preferably disarmed. A confident pilot could set them in flight with GCS.

The transmitter can be used to do these in the air. See :ref:`common-transmitter-tuning`

QAUTOTUNE
---------

If the aircraft appears stable enough to attempt QAUTOTUNE, follow the instructions in the :ref:`QAUTOTUNE<qautotune-mode>` page.

There a number of problems that can prevent QAUTOTUNE from providing a good tune. Some of the reason QAUTOTUNE can fail are:

- High levels of gyro noise.
- Incorrect value of :ref:`Q_M_THST_EXPO <Q_M_THST_EXPO>`.
- Flexible frame or payload mount.
- Overly flexible vibration isolation mount.
- Non-linear ESC response.
- Very low setting for :ref:`Q_M_SPIN_MIN <Q_M_SPIN_MIN>`.
- Overloaded propellers or motors.

If QAUTOTUNE has failed you will need to do a manual tune.

Some signs that QAUTOTUNE has been successful are:

- An increase in the values of :ref:`Q_A_ANG_PIT_P <Q_A_ANG_PIT_P>` and :ref:`Q_A_ANG_RLL_P <Q_A_ANG_RLL_P>`.
- :ref:`Q_A_RAT_PIT_D <Q_A_RAT_PIT_D>` and :ref:`Q_A_RAT_RLL_D <Q_A_RAT_RLL_D>` are larger than :ref:`Q_AUTOTUNE_MIN_D <Q_AUTOTUNE_MIN_D>`.

QAUTOTUNE will attempt to tune each axis as tight as the aircraft can tolerate. In some aircraft this can be unnecessarily responsive. A guide for most aircraft:

- :ref:`Q_A_ANG_PIT_P <Q_A_ANG_PIT_P>` should be reduced from 10 to 6
- :ref:`Q_A_ANG_RLL_P <Q_A_ANG_RLL_P>` should be reduced from 10 to 6
- :ref:`Q_A_ANG_YAW_P <Q_A_ANG_YAW_P>` should be reduced from 10 to 6
- :ref:`Q_A_RAT_YAW_P <Q_A_RAT_YAW_P>` should be reduced from 1 to 0.5
- :ref:`Q_A_RAT_YAW_I <Q_A_RAT_YAW_I>` : :ref:`Q_A_RAT_YAW_P <Q_A_RAT_YAW_P>` x 0.1

These values should only be changed if QAUTOTUNE produces higher values. Small aerobatic aircraft may prefer to keep these values as high as possible.

Setting the input shaping parameters
------------------------------------

QuadPlane has a set of parameters that define the way the aircraft feels to fly. This allows the aircraft to be set up with a very aggressive tune but still feel like a very docile and friendly aircraft to fly.

The most important of these parameters is:

- :ref:`Q_A_RAT_YAW_P <Q_A_RAT_YAW_P>` : yaw rate x 45 degrees/s
- :ref:`Q_ANGLE_MAX <Q_ANGLE_MAX>` :  maximum lean angle
- :ref:`Q_A_ACCEL_P_MAX <Q_A_ACCEL_P_MAX>` : Pitch rate acceleration
- :ref:`Q_A_ACCEL_R_MAX <Q_A_ACCEL_R_MAX>` : Roll rate acceleration
- :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>` : Yaw rate acceleration
- :ref:`Q_A_ANG_LIM_TC <Q_A_ANG_LIM_TC>` : Aircraft smoothing time

QAUTOTUNE will set the :ref:`Q_A_ACCEL_P_MAX <Q_A_ACCEL_P_MAX>`, :ref:`Q_A_ACCEL_R_MAX <Q_A_ACCEL_R_MAX>` and :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>` parameters to their maximum based on measurements done during the QAUTOTUNE tests. These values should not be increased beyond what QAUTOTUNE suggests without careful testing. In most cases pilots will want to reduce these values significantly.

For aircraft designed to carry large directly mounted payloads, the maximum values of :ref:`Q_A_ACCEL_P_MAX <Q_A_ACCEL_P_MAX>`, :ref:`Q_A_ACCEL_R_MAX <Q_A_ACCEL_R_MAX>` and :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>` should be reduced based on the minimum and maximum takeoff weight (TOW):

- :ref:`Q_A_ACCEL_P_MAX <Q_A_ACCEL_P_MAX>`  x (min_TOW / max_TOW)
- :ref:`Q_A_ACCEL_R_MAX <Q_A_ACCEL_R_MAX>`  x (min_TOW / max_TOW)
- :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>`  x (min_TOW / max_TOW)

:ref:`Q_A_RAT_YAW_P <Q_A_RAT_YAW_P>` should be set to be approximately 0.5 x :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>` / 4500 to ensure that the aircraft can achieve full yaw rate in approximately half a second.

:ref:`Q_A_ANG_LIM_TC <Q_A_ANG_LIM_TC>` may be increased to provide a very smooth feeling on the sticks at the expense of a slower reaction time.

Aerobatic aircraft should keep the :ref:`Q_A_ACCEL_P_MAX <Q_A_ACCEL_P_MAX>`, :ref:`Q_A_ACCEL_R_MAX <Q_A_ACCEL_R_MAX>` and :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>` provided by QAUTOTUNE and reduce :ref:`Q_A_ANG_LIM_TC <Q_A_ANG_LIM_TC>` to achieve the stick feel desired by the pilot. 

The full list of input shaping parameters are:


- :ref:`Q_A_RAT_YAW_P <Q_A_RAT_YAW_P>`
- :ref:`Q_ANGLE_MAX <Q_ANGLE_MAX>`
- :ref:`Q_A_ACCEL_P_MAX <Q_A_ACCEL_P_MAX>`
- :ref:`Q_A_ACCEL_R_MAX <Q_A_ACCEL_R_MAX>`
- :ref:`Q_A_ACCEL_Y_MAX <Q_A_ACCEL_Y_MAX>`
- :ref:`Q_A_ANG_LIM_TC <Q_A_ANG_LIM_TC>`
- :ref:`Q_A_RATE_P_MAX <Q_A_RATE_P_MAX>`
- :ref:`Q_A_RATE_R_MAX <Q_A_RATE_R_MAX>`
- :ref:`Q_A_RATE_Y_MAX <Q_A_RATE_Y_MAX>`
- :ref:`Q_A_SLEW_YAW <Q_A_SLEW_YAW>`
- :ref:`Q_LOIT_ACC_MAX <Q_LOIT_ACC_MAX>`
- :ref:`Q_LOIT_ANG_MAX <Q_LOIT_ANG_MAX>`
- :ref:`Q_LOIT_BRK_ACCEL <Q_LOIT_BRK_ACCEL>`
- :ref:`Q_LOIT_BRK_DELAY <Q_LOIT_BRK_DELAY>`
- :ref:`Q_LOIT_BRK_JERK <Q_LOIT_BRK_JERK>`
- :ref:`Q_LOIT_SPEED <Q_LOIT_SPEED>`


