.. _quadplane-tips:

====================
QuadPlane Setup Tips
====================

There are a few helpful hints and tips for first-time QuadPlane users.

Heavy commercial QuadPlane operators should also read:

.. toctree::
   :maxdepth: 1

   High Reliability QuadPlane Operation <quadplane-reliability>
   ESC Signalling Issues <common-esc-issues>

Motor Alignment
===============

Motor alignment is always critical for any Plane's performance, but it is especially so for QuadPlanes. 
Miss-aligned VTOL motors can greatly affect basic VTOL performance, especially yaw trim and effectiveness, as well as fixed-wing flight performance.

Checking motor alignment on the bench can be done by blocking the vehicle in a level position and measuring prop to table distances for all motors with props in fore-aft and side-to-side positions. Measurement differences easily reveal miss-alignments. The arc-sine of the prop tip end differences divided by the prop diameter will give the tilt angles. A degree or two miss-alignment, while not disastrous for pitch or roll, can be significant in non-vectored yaw configurations with regards to extreme yaw miss-trim. 

Miss-alignment can also be seen in the dataflash log RCOUT values in a hover for the VTOL motors. Yaw problems appear as large separations between diagonal motor pairs.

For vectored thrust frames, fixed-wing mode alignment is also critical. Very small differences between motor thrust angles in forward flight will generate undesirable, and sometimes uncontrollable, roll trim tendencies. Careful bench checking and adjustment of forward fixed-wing mode positions with respect to airframe and one another is required.

.. _h-vs-x-mixing:

X vs H Quad Mixing
==================

If the airframe is perfectly rigid, the performance difference between X and H mixing schemes is negligible. However, quadplanes structures are often less rigid than traditional multirotors and torsional effects must be considered. When a yaw input results in most of a quadplane's weight being supported by a diagonal pair of motors, the fuselage or wing of a quadplane may twist and distort the thrust vectors of some or all motors. This distortion often induces a yaw moment that can reduce or even completely disable yaw effectiveness unless the appropriate mixing scheme is used. While this effect is airframe dependent, it usually can be summarized in one of two ways:

- Airframes with quad motors mounted to arms extending forward and aft of the wings, aligned with the chord direction. X mixing scheme should be used such that any induced twist complements the desired yaw. If the H mixing scheme is used with this configuration, yaw inputs may induce wing twist that counteracts the desired yaw.
- Airframes with quad motors mounted to arms extending left and right from the fuselage, usually attached near the nose and tail. H mixing scheme should be used such that any induced twist complements the desired yaw. If the X mixing scheme is used with this configuration, yaw inputs can induce fuselage twist that counteracts the desired yaw.

Increasing YAW Authority
========================

In non-vectored yaw configurations, where torque deltas between motors is the only means of inducing/maintaining yaw, you may find that the yaw authority is too weak. In some cases, it may be so weak as to be inadequate even to maintain yaw directional stability in a hover due to imperfections in motor alignment or frame twist, even after careful trimming of motor alignment.

However, by intentionally inducing the appropriate motor tilt into each mount, YAW authority can be greatly increased at very small cost to total vertical lifting capability of the VTOL motors.

For H mixing, simply tilt all the motors inward toward the plane by 2-3 degrees, either by shimming or intentional design of the motor mounts. This provides a rotational thrust boost in addition to the torque differential, as the diagonal motor pairs are sped up/slowed down. With additional YAW authority, small motor angle imperfections can be automatically overcome by the autopilot, as well as giving the pilot additional YAW authority.

.. note:: Be aware that QuadPlanes rarely have the same yaw authority as multicopters due to the greater mass and surface area to wind that a plane presents. Expect to be only able to face into the wind if it's not relatively calm.

"LEVEL" Trim
============

When you setup a QuadPlane, the "level" calibration should be done with the plane in its normal cruising attitude (ie angle of attack of the wings). In a non-tailsitter quadplane, if the vertical position of the VTOL motors is not 90 degrees to this attitude, then the QuadPlane will tend to move forward or backward when hovering, requiring stick input to modify pitch. Or, in a tailsitter, if the "trim" position of the motors, and/or vertical CG is not through the thrust line when vertical, it will also drift.

Fortunately, there is a parameter which allows the independent adjustment of the VTOL AHRS "level" attitude. This is :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` in degrees to be added, negative or positive, to the AHRS "level". 

However, there is a better way for non-tailsitter QuadPlanes. If mechanically possible, trim each motor to be vertical when the QuadPlane is in fixed wing "level" mode, so that using :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` is not required. Usually, this requires 3-5 degrees of forward tilt. This has the advantage of having the wings generate lift while hovering in the wind, reducing the load on the VTOL motors and preventing sudden pitch changes during transitions back to VTOL mode from fixed-wing flight.

.. note:: Those fabricating their mounts via a 3D printer, this is very easy to do, as well as tilt for yaw enhancement discussed above.

.. note:: Fixed wing pitch "level" trim is set by the AHRS "level" done during accelerometer calibration, which adjusts the :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>` values, PLUS any :ref:`PTCH_TRIM_DEG<PTCH_TRIM_DEG>` (in degrees). See :ref:`common-accelerometer-calibration` and :ref:`tuning-cruise` for more details.

Trimming VTOL "Level" thru Accelerometer LeveL only Calibration
---------------------------------------------------------------

There is another, slightly more complex, way to set the VTOL stance pitch trim without using :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>`, in tailsitters only:

- Do the normal fixed wing accelerometer calibration. But read and save the :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>` value after the calibration.
- Place the vehicle in VTOL stance (nose pointing veritically) and set the mode to QSTABILIZE. Be sure that  :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` = "0" or the next step will fail.
- Do a "LEVEL only" accelerometer calibration. In Mission Planner there is a button for this under the Accelerometer calibration. In MAVProxy, its the "ahrstrim" command.
- Restore the previously noted and saved :ref:`AHRS_TRIM_Y<AHRS_TRIM_Y>` value to restore the fixed wing attitude pitch trim.

In some cases, :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` may still need to be adjusted if the calibration stance used is not the true hovering attitude.

Copter Motors vs Servos
=======================

If you are using a configuration where there are copter motors involved (almost all QuadPlanes do except single/dual motor tailsitters), the motor outputs are assigned as PWM protocol unless changed (see :ref:`common-brushless-escs` ). But the PWM is at a 400Hz update rate, not 50Hz. If a servo is used in an  output group sharing the same timer as one of the motor outputs, then it too will be operating at 400HZ and must be able to tolerate that. Most analog servos will overheat or burn up. Most digital servos can tolerate that rate. Check your autopilot's :ref:`hardware description <common-autopilots>` page for the DShot output groups, which indicates which outputs share the same timer.

.. _tilt-rotor-setup:

Tilt Rotor Servo Setup
======================

Adjustment of the tilt servo's max,min,trim output values for conventional and tailsitter tilt-rotors are show below. Whether an endpoint is adjusted by SERVOx_MAX or MIN, depends on if it has been reversed to get proper directional operation.

.. image:: ../../../images/tiltrotor-setup.jpg

--------------------------------------------------------------

.. image:: ../../../images/tailsitter-tilt-setup.jpg

--------------------------------------------------------------

.. image:: ../../../images/Bicopter-tilt-setup.jpg


.. note:: Setting :ref:`Q_OPTIONS<Q_OPTIONS>` bit 10 (Disarmed Yaw Tilt) allows the motors to tilt in response to rudder input while disarmed to facilitate adjustment of parameters.

Battery Sag
===========

Many QuadPlanes are targeted for duration and therefore may utilize high capacity to weight ratio, but low "C" , batteries. Battery sag in the initial phase of VTOL to fixed-wing transitions can be a real issue. Be sure to read the warning in :ref:`quadplane-flying` and to set :ref:`BATT_WATT_MAX<BATT_WATT_MAX>`, :ref:`Q_M_BAT_CURR_MAX<Q_M_BAT_CURR_MAX>`, and the :ref:`Q_M_BAT_VOLT_MIN<Q_M_BAT_VOLT_MIN>` parameters appropriately.

Optional Features Sometimes Overlooked
======================================

In case you missed them:

- Various QuadPlane Options: :ref:`Q_OPTIONS<Q_OPTIONS>`, bits 0,and 3-5 being especially interesting
- RTL Behaviour; :ref:`Q_RTL_MODE<Q_RTL_MODE>`
- :ref:`Assisted Flight<assisted_fixed_wing_flight>`
- :ref:`Weathervaning<quadplane-weathervaning>`
