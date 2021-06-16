.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================

PLANE
=====



On Airspeed Calibration Page
----------------------------

Add section at end:

Miss-calibration Safeguards
===========================

In order to help prevent Airspeed sensor use when its been miss-calibrated either during ground static calibration during the power up sequence, or by accidental parameter changes to offset or ratio, three parameters are available. If the ground speed is consistently lower than the reported airspeed for a few seconds by :ref:`ARSPD_WIND_MAX<ARSPD_WIND_MAX>`, i.e. the apparent wind speed is greater than that amount, the sensor can be disabled to avoid erroneous reporting. It can be allowed to re-enable if the apparent wind falls back below that value. These actions are controlled by :ref:`ARSPD_OPTIONs<ARSPD_OPTIONs>`.

You can also send a warning to the Ground Control Station if the apparent wind exceeds :ref:`ARSPD_WIND_WARN<ARSPD_WIND_WARN>`. This can be used instead of, or together with the above.

On Quadplane Tips Page:
-----------------------

Under Tilt Rotor Servo Setup, add:

Note that setting :ref:`Q_OPTIONS<Q_OPTIONS>` bit 10 (Disarmed Yaw Tilt) allows the motors to tilt in response to rudder input while disarmed to facilitate adjustment of parameters.

On Flight Options Page:
 add to table

=====================================   ======================
:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>`   Function
=====================================   ======================
4                                       Climb to :ref:`ALT_HOLD_RTL<ALT_HOLD_RTL>` altitude before turning toward home in RTL
=====================================   ======================

