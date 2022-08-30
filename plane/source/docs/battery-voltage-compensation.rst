.. _battery-voltage-compensation:


============================
Battery Voltage Compensation
============================

As a flight progresses the battery voltage will gradually lower as power is consumed. This affects the maximum climb rate, and the cruising speed in systems with no airpeed sensor. Plane provides two mechanisms to cope with this, one for forward motor(s) and another for VTOL motors.

Forward Motor Compensation
==========================

This is provided using:

- :ref:`FWD_BAT_VOLT_MAX<FWD_BAT_VOLT_MAX>` : This sets the voltage from which lower voltages will result in compensating the actual applied forward throttle upwards. A value of "0", disables this function. Setting this to 4.2V * number of cells is a good value.
- :ref:`FWD_BAT_VOLT_MIN<FWD_BAT_VOLT_MIN>` : Sets the lower limit at which throttle compensation will be done to prevent dragging the battery down excessively due to increased throttle scaling. Setting this to 3.3V * number of cells is a good value.
- :ref:`FWD_BAT_IDX<FWD_BAT_IDX>` : Determines which battery monitor voltage is used (index 0-9). Index "3" corresponds to Battery Monitor 4, etc.

For battery voltages below :ref:`FWD_BAT_VOLT_MAX<FWD_BAT_VOLT_MAX>`, before the throttle value that the autopilot determines should be output is applied, it is scaled by a ratio of the this parameter versus the measured voltage. This allows all modes (except MANUAL, which never has this applied) to maintain roughly constant cruise speeds at a given throttle stick position.

If an airspeed sensor is used and healthy, then for speed controlled modes, such as CRUISE and AUTO, it will determine the throttle required instead. However, the :ref:`THR_MAX<THR_MAX>` and :ref:`THR_MIN<THR_MIN>` are compensated in all modes except MANUAL, allowing speed controlled modes greater range of actual throttle control.

:ref:`FWD_BAT_VOLT_MIN<FWD_BAT_VOLT_MIN>` sets a lower limit on compensation to prevent excessive sag.

QuadPlane VTOL Motor Compensation
=================================

This is provided using in a similar manner:

- :ref:`Q_M_BAT_VOLT_MAX<Q_M_BAT_VOLT_MAX>` :This sets the voltage from which lower voltages will result in compensating the actual applied VTOL motors throttle upwards. A value of "0", disables this function. Setting this to 4.2V * number of cells is a good value.
- :ref:`Q_M_BAT_VOLT_MIN<Q_M_BAT_VOLT_MIN>` :Sets the lower limit at which throttle compensation will be done to prevent dragging the battery down excessively due to increased throttle scaling. Setting this to 3.3V * number of cells is a good value.
- :ref:`Q_M_BAT_IDX<Q_M_BAT_IDX>`: Determines which battery monitor voltage is used (index 0-9). Index "3" corresponds to Battery Monitor 4, etc.

If :ref:`Q_M_BAT_VOLT_MAX<Q_M_BAT_VOLT_MAX>` is non-zero, then voltage compensation is applied to throttle to offset battery voltage variations on thrust as above for the forward motor(s)

:ref:`Q_M_BAT_VOLT_MIN<Q_M_BAT_VOLT_MIN>` sets a lower limit on battery voltage sag by predicting the throttle value that would cause this, using an estimate of battery internal resistance calculated during flight. It will limit current applied to the VTOL motors to prevent this from occurring. If :ref:`Q_M_BAT_CURR_MAX<Q_M_BAT_CURR_MAX>` is also non-zero, then it will use the lower of the two limits, the one set by :ref:`Q_M_BAT_CURR_MAX<Q_M_BAT_CURR_MAX>` or predicted internally using :ref:`Q_M_BAT_VOLT_MIN<Q_M_BAT_VOLT_MIN>` as the minimum voltage sag target.


