.. _quadplane-flying:

==================
Flying a QuadPlane
==================

While flying a QuadPlane can actually be easier than flying a
conventional fixed wing aircraft there are some things you need to
understand. Please read the following sections carefully.


.. toctree::
    :maxdepth: 1

    Transitions: VTOL/FW <quadplane-transitions>
    Assisted Fixed Wing Flight <assisted_fixed_wing_flight>
    Return to Launch <quadplane_rtl>

VTOL vs Fixed-Wing Level Trim
=============================

Often fixed wing "level" trim, which is the pitch attitude stabilization modes attempt to maintain, is set to be several degrees positive with respect to the wing chord line in order to provide lift while cruising. See :ref:`common-accelerometer-calibration` and :ref:`tuning-cruise` for more details.

However, when in VTOL modes, this can result in the vehicle leaning "backward" a few degrees, building in a tendency to drift backwards. This can be eliminated by setting the :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` parameter to correct this. This can also be used to correct minor CG imbalances caused by VTOL motor placement not being exactly balanced around the CG.

Manual Forward Throttle in VTOL Modes
=====================================

By setting an RC channel option (``RCx_OPTION``) to "209", that channel can provide a separate throttle input to the forward motor(s) in QSTABILIZE, QACRO, and QHOVER VTOL modes. This allows forward movement without having to tilt the QuadPlane forward requiring throttle stick repositioning in QSTABILIZE and QACRO to maintain altitude, and present more forward flat plate resistance to forward movement in all modes. The maximum percentage throttle that will be applied by this channel is set by :ref:`Q_FWD_MANTHR_MAX<Q_FWD_MANTHR_MAX>`.

.. _quadplane_failsafe:

Radio or Throttle Failsafe
==========================

If flying in a plane mode or AUTO, behaviour is determined by the :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` and :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` parameter settings (see Plane Failsafe Function). QuadPlanes can be set such that instead of normal plane behaviour on Failsafe induced RTLs, to transition to QRTL and land once at the rally point or home, if  :ref:`Q_RTL_MODE<Q_RTL_MODE>` =1. If :ref:`Q_RTL_MODE<Q_RTL_MODE>` =2, then a fixed wing approach followed by a loiter to alt and QRTL will be executed, similar to that described in the "AUTO VTOL Landing" section of :ref:`quadplane-auto-mode`.

If lying in any VTOL mode (QHOVER,QSTAB,etc.) and not flying a mission, failsafe will evoke QLAND , QRTL or RTL, depending on how :ref:`Q_OPTIONS<Q_OPTIONS>`, bits 5 and 20, are set.

.. _what-will-happen:

What Will Happen?
=================

Understanding hybrid aircraft can be difficult at first, so below are
some scenarios and how the ArduPilot code will handle them.

I am hovering in QHOVER/QLOITER and switch to FBWA mode
-------------------------------------------------------

The aircraft will continue to hover, setting forward thrust/throttle at whatever the throttle stick position dictates and gaining speed. If you zero throttle during the transition, the aircraft will continue to hold the current height and hold itself level, slowing to a halt. It will drift
with the wind as it is not doing position hold.

If you advance the throttle stick then the forward motor will throttle-up and
the aircraft will start to move forward. The quad motors will continue
to provide both lift and stability while the aircraft is moving slowly.
You can control the attitude of the aircraft with roll and pitch stick
input. When you use the pitch stick (elevator) that will affect the
climb rate of the quad motors. If you pull back on the elevator the quad
motors will assist with the aircraft climb. If you push forward on the
pitch stick the power to the quad motors will decrease and the aircraft
will descend.

The roll and pitch input also controls the attitude of the aircraft, so
a right roll at low speed will cause the aircraft to move to the right.
It will also cause the aircraft to yaw to the right (as the QuadPlane
code interprets right aileron in fixed wing mode as a commanded turn).

Once the aircraft reaches an airspeed of :ref:`AIRSPEED_MIN <AIRSPEED_MIN>`
(or :ref:`Q_ASSIST_SPEED <Q_ASSIST_SPEED>` if that is set and is greater than :ref:`AIRSPEED_MIN <AIRSPEED_MIN>`)
the amount of assistance the quad motors provide will decrease over 5
seconds. After that time the aircraft will be flying purely as a fixed wing.

I am flying fast in FBWA mode and switch to QHOVER mode
-------------------------------------------------------

The quad motors will immediately engage and will start by holding the
aircraft at the current height. The climb/descent rate is now set by the
throttle stick, with a higher throttle stick meaning climb and a lower
throttle stick meaning descend. At mid-stick the aircraft will hold
altitude.

The forward motor will stop, but the aircraft will continue to move
forward due to its momentum. The drag of the air will slowly bring it to
a stop. The attitude of the aircraft can be controlled with roll and
pitch sticks (aileron and elevator). You can yaw the aircraft with
rudder.

I switch to RTL mode while hovering
-----------------------------------

The aircraft will generally transition to fixed wing flight. The quad motors will
provide assistance with lift and attitude while the forward motor starts
to pull the aircraft forward. Depending on the :ref:`Q_RTL_MODE<Q_RTL_MODE>`,
different behaviors can be selected as it returns to the return point (rally or home).
See :ref:`quadplane_rtl` for details.

If you have :ref:`RTL_AUTOLAND <RTL_AUTOLAND>`
setup then the aircraft will follow the mission configuration.


I switch into QRTL close to HOME
--------------------------------

If closer than 1.5X the larger of either :ref:`RTL_RADIUS<RTL_RADIUS>` or :ref:`WP_LOITER_RAD<RTL_RADIUS>`, then the vehicle will proceed toward home in VTOL mode and land. If greater, it will transition to fixed wing, climbing toward :ref:`RTL_ALTITUDE<RTL_ALTITUDE>` and executing a normal QRTL. Depending on how far from home, the vehicle may only briefly climb and then switch back to approach or airbrake phases. The further away, the higher the climb as it flies back toward home. If the approach behavior has ben disabled with :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16, then it will just switch to VTOL (if not already in that mode, navigate to home and land).

I have an EKF Failsafe
----------------------

ArduPilot provides a failsafe mechanism to protect VTOL operation in case the EKF becomes unhealthy. In normal fixed wing operation, ArduPilot "falls back" to another inertial guidance filter, DCM, if the EKF becomes unhealthy. However, if operating in AUTO mode in a VTOL mode when an EKF failure occurs, QuadPlanes will switch modes to QLAND, and to QHOVER in all other position control VTOL modes (QLOITER, QRTL, QLAND, QAUTOTUNE).

Typical Flight
==============

A typical test flight would be:

-  VTOL takeoff in :ref:`QLOITER<qloiter-mode>` or :ref:`QHOVER<qhover-mode>`
-  switch to :ref:`FBWA <fbwa-mode>` mode and advance throttle over 50% and start
   flying fixed wing
-  switch to :ref:`QHOVER<qhover-mode>` mode to go back to quad mode and reduce throttle back to 50% for hover.

