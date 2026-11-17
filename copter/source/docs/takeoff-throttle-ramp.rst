.. _takeoff-throttle-ramp:

=====================
Takeoff Throttle Ramp
=====================

When a takeoff is started in a mode where the autopilot controls the altitude, the throttle is not handed straight to the altitude controller. Instead, while the vehicle is still considered landed, the throttle is increased at a fixed rate and the autopilot watches for signs that the vehicle has left the ground. Once takeoff is detected, the altitude controller takes over and climbs to the requested altitude or at the requested climb rate. This makes the start of the takeoff consistent and largely independent of the altitude controller tune and of the payload being carried.

The ramp is used for:

- **Pilot takeoffs**: in AltHold, Loiter, PosHold, Sport, FlowHold and ZigZag, when the throttle stick is raised above the mid-stick deadzone, or when a takeoff command is received from a ground station in these modes.
- **Auto and Guided takeoffs**: a ``NAV_TAKEOFF`` mission command in Auto, or a takeoff command in Guided.

It is not used in Stabilize or Acro, where the pilot controls the throttle directly.

Before the ramp starts
======================

For throttle stick takeoffs and for Auto and Guided takeoffs, the ramp only starts once the motors have finished spooling up. On a multicopter this means:

- any :ref:`MOT_IDLE_SEC<MOT_IDLE_SEC>` delay at idle has passed (see :ref:`set-motor-range`)
- the motors have spun up over :ref:`MOT_SPOOL_TIME<MOT_SPOOL_TIME>`
- if ESC RPM telemetry is used, the :ref:`TKOFF_RPM_MIN<TKOFF_RPM_MIN>` and :ref:`TKOFF_RPM_MAX<TKOFF_RPM_MAX>` checks have passed (see :ref:`common-tkoff-rpm-min`)

On a traditional helicopter the ramp starts once the rotor run-up has completed, and the ramp is applied to collective.

A takeoff command sent from a ground station in AltHold, Loiter, PosHold, Sport, FlowHold or ZigZag does not wait for this: the ramp starts as soon as the command is accepted, while the motors are still spooling up. On a multicopter without motor interlock, in AltHold, Loiter, PosHold, Sport or ZigZag the throttle stick must also be within or above the mid-stick deadzone when the command is sent. If it is below the deadzone, the ramp runs with the motors held at ground idle and the vehicle does not take off. When the ramp ends (about 1.8 seconds after the command with the defaults) the motors then briefly surge above hover throttle before returning to idle.

Takeoff detection
=================

During the ramp, takeoff is declared as soon as any one of the following is true:

- the vertical acceleration is more than 50% of the maximum vertical acceleration
- the climb rate is more than:

  - for pilot takeoffs, the climb rate requested by the pilot, but no less than 10% and no more than 50% of the maximum climb rate
  - for Auto and Guided takeoffs, 10% of the maximum climb rate

- the vehicle has climbed more than:

  - for pilot takeoffs, half of the takeoff altitude (:ref:`PILOT_TKO_ALT_M<PILOT_TKO_ALT_M>`, or the altitude in a takeoff command)
  - for Auto and Guided takeoffs, :ref:`WP_NAVALT_MIN<WP_NAVALT_MIN>`

  This check is only made when that altitude is above zero. Both parameters default to zero.

- the throttle reaches :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>`

The maximum vertical acceleration and climb rate are :ref:`PILOT_ACC_Z<PILOT_ACC_Z>` and :ref:`PILOT_SPD_UP<PILOT_SPD_UP>` for pilot takeoffs, and :ref:`WP_ACC_Z<WP_ACC_Z>` and :ref:`WP_SPD_UP<WP_SPD_UP>` for Auto and Guided takeoffs. Because the detection thresholds are based on these limits, a vehicle configured with high limits will need more thrust before takeoff is detected.

.. warning:: Once the ramp has started, lowering the throttle stick does not stop it. The ramp continues until takeoff is detected. To abort a takeoff before then, disarm, or lower the throttle stick fully and then change to Stabilize (on a multicopter, while the vehicle is still landed, the change to Stabilize is refused with "throttle too high" if the throttle stick is raised).

Parameters
==========

- :ref:`TKOFF_SLEW_TIME<TKOFF_SLEW_TIME>` sets the rate of the ramp. It is the time in seconds the throttle would take to increase from zero to full (default 2 seconds, range 0.25 to 5). Because the ramp ends at :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` at the latest, the longest it can last is :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` x :ref:`TKOFF_SLEW_TIME<TKOFF_SLEW_TIME>` seconds (1.8 seconds with the defaults).

  Increase it for a gentler start to the takeoff, or decrease it for a quicker one. Very short values can apply significantly more thrust than is needed before takeoff is detected. Very long values leave the vehicle sitting on the ground at part thrust for longer, which can cause it to tip over on sloping or uneven ground or in wind.

- :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` is the throttle at which takeoff is declared even if no climb has been detected (default 0.9, range 0 to 0.9). It uses the same scale as the hover throttle: :ref:`MOT_THST_HOVER<MOT_THST_HOVER>` on multicopters (see :ref:`ac_throttlemid`), or the hover collective :ref:`H_COL_HOVER<H_COL_HOVER>` on traditional helicopters. It does not change the rate of the ramp, it only limits how far the ramp can go.

  - On vehicles with high vertical acceleration limits, or a short :ref:`TKOFF_SLEW_TIME<TKOFF_SLEW_TIME>`, setting it slightly above hover throttle stops the ramp applying much more thrust than is needed to lift off.
  - If it is set below hover throttle, the ramp ends before the vehicle can lift off. The vehicle is then treated as flying while still on the ground, and the altitude controller increases the throttle until it lifts off. The takeoff then depends on the altitude controller tune.
  - Setting it to zero skips the ramp entirely and hands the takeoff straight to the altitude controller.

  Only set it relative to the hover throttle after it has been learned (see :ref:`MOT_HOVER_LEARN<MOT_HOVER_LEARN>` on multicopters, or :ref:`H_HOVER_LEARN<H_HOVER_LEARN>` on traditional helicopters).
