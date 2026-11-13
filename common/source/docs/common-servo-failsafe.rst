.. _common-servo-failsafe:

[copywiki destination="copter,plane,rover,sub"]

==========================
Servo Failsafe Positions
==========================

.. note:: This feature is available in firmware versions 4.8 and later.

Selected outputs can be driven to a fixed position while a failsafe is active, for example to close a payload release, point a camera down or turn on a light. Each output has its own failsafe position, set by ``SERVOx_FSPWM``, and ``FS_SERVO_MASK`` selects which failsafes use these positions.

The positions are applied in addition to the action each failsafe takes. They do not change the mode or the failsafe action.

Setup
=====

#. Set ``SERVOx_FSPWM`` to the PWM the output should be driven to during a failsafe, for each output that should move. ``0`` (the default) leaves that output unchanged during failsafes.
#. Set ``FS_SERVO_MASK`` to choose which failsafes apply the positions:

[site wiki="copter"]
   ====    =============================================================
   Bit     Failsafe
   ====    =============================================================
   0       :ref:`Radio <radio-failsafe>`
   1       :ref:`Battery <failsafe-battery>`
   2       :ref:`GCS <gcs-failsafe>`
   3       :ref:`EKF <ekf-inav-failsafe>`
   4       Terrain data loss
   5       ADSB
   6       :ref:`Dead Reckoning <deadreckoning-failsafe>`
   ====    =============================================================
[/site]
[site wiki="plane"]
   ====    =============================================================
   Bit     Failsafe
   ====    =============================================================
   0       Radio
   1       Battery
   2       GCS
   5       ADSB
   ====    =============================================================

   See :ref:`apms-failsafe-function` for these failsafes. The radio and GCS failsafes are each checked on their own, so either one applies the positions while the other is also active.
[/site]
[site wiki="rover"]
   ====    =============================================================
   Bit     Failsafe
   ====    =============================================================
   0       Radio
   1       Battery
   2       GCS
   3       EKF
   ====    =============================================================

   See :ref:`rover-failsafes` for these failsafes. A radio or GCS failsafe applies the positions once it has lasted ``FS_TIMEOUT``, in any mode, including Hold where no failsafe action is taken.
[/site]
[site wiki="sub"]
   ====    =============================================================
   Bit     Failsafe
   ====    =============================================================
   0       :ref:`Radio <radio-failsafe>`
   1       :ref:`Battery <failsafe-battery>`
   2       :ref:`GCS <gcs-failsafe>`
   3       :ref:`EKF <ekf-inav-failsafe>`
   4       Terrain
   7       :ref:`Pilot input <pilot-control-failsafe>`
   8       :ref:`Leak <internal-leak-failsafe>`
   9       :ref:`Internal pressure <internal-pressure-failsafe>`
   10      :ref:`Internal temperature <internal-temperature-failsafe>`
   11      Crash
   12      Sensor health
   ====    =============================================================
[/site]

   The default is 7 (radio, battery and GCS). Set it to 0 to disable the failsafe positions entirely.

How it works
============

- The positions are only applied while armed. Nothing moves during a failsafe while disarmed.
- While any failsafe selected by ``FS_SERVO_MASK`` is active, each output with a non-zero ``SERVOx_FSPWM`` is driven to that PWM. ``SERVOx_FSPWM`` can be set from 0 to 2500, and values from 1 to 499 are treated as 500.
- When the failsafe clears, the output returns to the value it would otherwise have, including outputs set by a ``DO_SET_SERVO`` command, a gripper or a camera trigger.
- Outputs that control the vehicle are never moved, even if ``SERVOx_FSPWM`` is set: motors, throttle and the other outputs :ref:`motor emergency stop <common-auxiliary-functions>` applies to, control surfaces (including flaps, spoilers and airbrakes), steering, tilt and sail outputs, the attitude controller roll, pitch, thrust and yaw outputs, engine ignition and parachute release. The failsafe action and the engine and parachute logic keep control of them, and motor emergency stop always takes precedence.
- The position is not limited by ``SERVOx_MIN`` and ``SERVOx_MAX`` and does not depend on ``SERVOx_TRIM``, so check that the servo can safely reach it.

.. note:: This is different from :ref:`SERVO_RC_FS_MSK<SERVO_RC_FS_MSK>`, which only applies to RC passthrough outputs during a radio failsafe and sets them as if their RC input had gone to its trim value.

.. note:: PiccoloCAN servos are configured per output function, so PiccoloCAN servos that share the same function all use the ``SERVOx_FSPWM`` of the first output with that function. Other output types, such as DroneCAN, SBUS output, Volz and Robotis servos, use each output's own setting.

Testing
=======

[site wiki="copter,plane"]
Most failsafes disarm the vehicle immediately when it is on the ground, or cannot be reached without flying, so the positions cannot normally be seen on the bench. Check the setup in :ref:`SITL <dev:sitl-simulator-software-in-the-loop>` first, by flying the simulated vehicle and triggering a selected failsafe, for example by setting ``SIM_RC_FAIL`` to 1 for a radio failsafe, and checking that each output moves to its ``SERVOx_FSPWM`` and returns when the failsafe clears.
[/site]
[site wiki="rover"]
With the wheels off the ground, arm in Manual mode and trigger a selected failsafe, for example by turning off the transmitter for a radio failsafe, and check that each output moves to its ``SERVOx_FSPWM`` and returns when the failsafe clears. The setup can also be checked in :ref:`SITL <dev:sitl-simulator-software-in-the-loop>` by setting ``SIM_RC_FAIL`` to 1.
[/site]
[site wiki="sub"]
With the vehicle secured, arm and trigger a selected failsafe, for example by closing the ground station for a GCS failsafe, and check that each output moves to its ``SERVOx_FSPWM`` and returns when the failsafe clears. The setup can also be checked in :ref:`SITL <dev:sitl-simulator-software-in-the-loop>`.
[/site]
