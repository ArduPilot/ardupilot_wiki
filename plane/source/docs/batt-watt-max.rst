.. _batt-watt-max:

===================================
Limiting Maximum Battery Power Draw
===================================

Often, long range vehicles utilize lower C rating batteries to maximize battery power to weight ratio. Unfortunately, some situations, especially QuadPlane transitions which have full throttle power applied to forward motors during the transition, can cause excessive battery voltage sag with lower C rated batteries. This can lead to permanent damage in some kinds of batteries (LiPo for example) if the cell voltage drops too low. In addition, this can cause premature battery failsafes, or even crashes. This topic discusses ways to limit the maximum power draw from batteries.

Methods to Limit Maximum Power Draw
===================================

The first and simplest is to restrict :ref:`THR_MAX<THR_MAX>` to a lower value. This limits the maximum throttle applied in throttle controlled modes (AUTO, FBWB, CRUISE, etc.). Unfortunately, while this could be used to limit maximum power to the forward motors in a QuadPlane transition, it will also limit it in normal fixed wing throttle controlled modes also, when the VTOL motors are not running and adding to the total instantaneous battery power being drawn.

A better way is to use one or more of the power limiting parameters:

-  :ref:`BATT_WATT_MAX<BATT_WATT_MAX>` 

The :ref:`BATT_WATT_MAX<BATT_WATT_MAX>` parameter (``BATTx_WATT_MAX``, for any additional batteries) to limit the total instantaneous power that can be drawn from the battery(s) by the forward motors. This will help issues during QuadPlane transitions.

If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (:ref:`THR_MAX<THR_MAX>` , :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` and :ref:`THR_MIN<THR_MIN>` for reverse thrust) to be below :ref:`BATT_WATT_MAX<BATT_WATT_MAX>`. If power demand is reduced, the max throttle will slowly grow back to :ref:`THR_MAX<THR_MAX>` (or :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` ) and/or :ref:`THR_MIN<THR_MIN>` , even if demanding the current max but as long as power remains under the watt max. Use 0 (default) to disable this feature.

Since this limit is applied to the throttle slowly (~10%/second), it may not react fast enough in some situations to prevent battery sag at the beginning of transition. Setting the :ref:`THR_SLEWRATE<THR_SLEWRATE>` to 50% or less per second to prevent rapid application of maximum forward throttle at the beginning of transition, will help prevent this.

.. note:: This only affects the forward motors, VTOL motors are unaffected by :ref:`BATT_WATT_MAX<BATT_WATT_MAX>`.

-  :ref:`Q_M_BAT_CURR_MAX<Q_M_BAT_CURR_MAX>`

This can be used to limit the maximum current when the VTOL motors are active. Power will be reduced to the VTOL motors to prevent exceeding this value.

-  :ref:`Q_M_BAT_VOLT_MIN<Q_M_BAT_VOLT_MIN>` 

This has two effects if non-zero. First, it sets the lower limit for thrust scaling versus voltage. If :ref:`Q_M_BAT_VOLT_MAX<Q_M_BAT_VOLT_MAX>` is non-zero, then voltage compensation is applied to throttle to offset battery voltage variations on thrust. Second, it sets a lower limit on battery voltage sag by predicting the throttle value that would cause this, using an estimate of battery internal resistance calculated during flight. It will limit current applied to the VTOL motors to prevent this from occurring. If :ref:`Q_M_BAT_CURR_MAX<Q_M_BAT_CURR_MAX>` is also  non-zero, then it will use the lower of the two limits, the one set by :ref:`Q_M_BAT_CURR_MAX<Q_M_BAT_CURR_MAX>` or predicted internally using :ref:`Q_M_BAT_VOLT_MIN<Q_M_BAT_VOLT_MIN>`  as the minimum voltage sag target.

.. note:: the battery used for monitoring and the battery resistance estimate is determined by  :ref:`Q_M_BAT_IDX<Q_M_BAT_IDX>`

.. tip:: Setting :ref:`Q_M_BAT_CURR_MAX<Q_M_BAT_CURR_MAX>` to 150% of hover current is a good starting point, while 3.3V * number of cells is a good value for :ref:`Q_M_BAT_VOLT_MIN<Q_M_BAT_VOLT_MIN>` .

How to Set Max Watts
--------------------

By examining a post flight log, and noting anywhere excessive battery voltage sag first occurs, you can take the battery voltage and current at that point, multiply to obtain the power, and set the :ref:`BATT_WATT_MAX<BATT_WATT_MAX>` to below that value. 80% to 70% of the value noted at the sag point would be a good value to start with.
