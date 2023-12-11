.. _initial-tuning-flight:

=====================
Initial Tuning Flight
=====================

Pilot's preparation for first flight
====================================

The first takeoff of an untuned multirotor is the most dangerous seconds of the aircraft’s life. This is where the aircraft could be very unstable causing a sudden increase in power when then results in the aircraft jumping into the air, or it may be so poorly tuned that you have insufficient control over the aircraft once it is airborne. The pilot should be extremely diligent during the tuning flights to avoid a situation that could result in injury or damage.

There are several things that the pilot can do to minimise the risk during the early tuning process:

1. The pilot should conduct a motor number and orientation check (see :ref:`Checking the motor numbering with the Mission Planner Motor test <connect-escs-and-motors_testing_motor_spin_directions>`). Care should be taken to ensure that the correct frame type is selected. Incorrect frame type can result in a very fast yaw rotation or complete loss of control. Take note of the output percentage required to spin the propellers and ensure that:

- :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>` is set high enough to spin the motors cleanly.
- :ref:`MOT_SPIN_MIN <MOT_SPIN_MIN>` is set high enough to spin the motors win a minimal level of thrust.

2. All flights after a significant tuning change should be done in Stabilize. Stabilize provides the pilot with significantly more control over the aircraft in the event that the attitude controllers are unstable.
3. The pilot should not take off in AltHold until the altitude controller has been tested in flight. This should be done by taking off in Stabilize and switching to Alt Hold. While Alt Hold is rarely a problem unless the aircraft has a very low hover throttle.
4. For the initial flights the pilot should ensure that these parameters are set:

- :ref:`ATC_THR_MIX_MAN <ATC_THR_MIX_MAN>` to 0.1
- :ref:`MOT_THST_HOVER <MOT_THST_HOVER>` to 0.25 (or lower than the expected hover throttle)

5. Use a radio and calibrate the radio correctly (see :ref:`common-radio-control-calibration`).
6. Configure an Emergency Stop Motors switch and test it (see :ref:`Auxiliary Functions <common-auxiliary-functions>`).
7. Do tuning flights in low-wind condition and normal weather (no rain and between 15°C/59°F and 25°C/77°F).
8. Practice STABILIZE flight in simulator or on a low-end drone first, you should be confident to be able to takeoff and land with your untuned aircraft.


First Flight
============

The first take off is the most dangerous time for any multirotor. Care must be taken to ensure the aircraft is not destroyed in the first seconds of flight and nobody is injured.

- **Ensure that all spectators are at a safe distance**.
- **Ensure the pilot is at a safe distance and position**.
- The pilot should refresh themselves on the method used to disarm the aircraft (using :ref:`Auxiliary Functions <common-auxiliary-functions>` for Motor Interlock or Arm/Disarm may be beneficial).

This flight will allow to setup your aircraft in a "flyable for tuning" state.

1. Ensure the aircraft is in STABILIZE mode
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

The first priority when tuning an multirotor aircraft is to establish a stable tune, free of oscillations, that can be used to do further tests.

1. Arm the aircraft in STABILIZE
2. Increase the throttle slowly until the aircraft leaves the ground
3. If the aircraft starts to oscillate immediately abort the takeoff and/or land the aircraft
4. Reduce all the following parameters by 50%

a. :ref:`ATC_RAT_PIT_P <ATC_RAT_PIT_P__AC_AttitudeControl_Multi>`
b. :ref:`ATC_RAT_PIT_I <ATC_RAT_PIT_I__AC_AttitudeControl_Multi>`
c. :ref:`ATC_RAT_PIT_D <ATC_RAT_PIT_D__AC_AttitudeControl_Multi>`
d. :ref:`ATC_RAT_RLL_P <ATC_RAT_RLL_P__AC_AttitudeControl_Multi>`
e. :ref:`ATC_RAT_RLL_I <ATC_RAT_RLL_I__AC_AttitudeControl_Multi>`
f. :ref:`ATC_RAT_RLL_D <ATC_RAT_RLL_D__AC_AttitudeControl_Multi>`

This process is repeated until the aircraft can hover without oscillations being detectable visually or audibly.

If the aircraft has very long or flexible landing gear then you may need to leave the ground before ground resonance stops.

Be aware that in this state the aircraft may be very slow to respond to large control inputs and disturbances. The pilot should be extremely careful to put minimal stick inputs into the aircraft to avoid the possibility of a crash.

Test AltHold
-------------

This test will allow to test the altitude controller and ensure the stability of your aircraft.

1. Check :ref:`MOT_HOVER_LEARN <MOT_HOVER_LEARN>` is set to 2. This will allow the controller to learn by itself the correct hover value when flying.

2. Take off in STABILIZE and increase altitude to 5m. Switch to AltHold and be ready to switch back to STABILIZE. If the aircraft is hovering at a very low hover throttle value you may hear a reasonably fast oscillation in the motors. Ensure the aircraft has spent at least 30 seconds in hover to let the hover throttle parameter converge to the correct value. Land and disarm the aircraft.

3. Set these parameters on ground and preferably disarm  (A confident pilot could set them in flight with GCS or CH6 tuning knob):

  - :ref:`PSC_ACCZ_I <PSC_ACCZ_I>` to 2 x :ref:`MOT_THST_HOVER <MOT_THST_HOVER>`
  - :ref:`PSC_ACCZ_P <PSC_ACCZ_P>` to :ref:`MOT_THST_HOVER <MOT_THST_HOVER>`

if AltHold starts to oscillate up and down the position and velocity controllers may need to be reduced by 50%. These values are: :ref:`PSC_POSZ_P <PSC_POSZ_P>` and :ref:`PSC_VELZ_P <PSC_VELZ_P>`.

Harmonic Notch Filtering
========================

After you have a hover without oscillations the next step is to
get get a good notch filter setup to reduce noise to the  PID
controllers. A good set of notch filtering parameters is critical to a
good tune.

To get a notch filter setup you need to hover your vehicle for at least 30 seconds with no pilot input and with :ref:`INS_LOG_BAT_MASK<INS_LOG_BAT_MASK>` set to 1. This will enable FFT logging which will
guide the correct setup of the notch filters. You should then carefully read the :ref:`common-imu-notch-filtering` documentation and
setup a harmonic notch to remove the noise from your gyros.

Eliminating noise with the notch filters will dramatically improve the quality
of your tune.