.. _batt-watt-max:

===================================
Limiting Maximum Battery Power Draw
===================================

Sometimes power needs to be limited to prevent issues with battery sag or overheating. This can occur because of the use of lower C rated batteries, high power motors, high ambient temperatures, uncooled engines or ESCs, or other reasons. This topic discusses ways to limit the maximum power draw from batteries.

Methods to Limit Maximum Power Draw
===================================

The first and simplest is to restrict :ref:`MOT_THR_MAX<MOT_THR_MAX>` to a lower value. This limits the maximum throttle controlled allowed. Unfortunately, this will not set an exact power limit and might require some trial and error to find the right setting.

A better way is to use a power limiting parameter (shown for first battery monitor):

- :ref:`BATT_WATT_MAX<BATT_WATT_MAX>`

The :ref:`BATT_WATT_MAX<BATT_WATT_MAX>` parameter can be used to limit the total wattage that can be drawn from the battery(s). If battery power exceeds this value then the system will reduce max throttle (:ref:`MOT_THR_MAX<MOT_THR_MAX>`) to lower the power draw. If power demand is reduced, the max throttle will slowly grow back to :ref:`MOT_THR_MAX<MOT_THR_MAX>`. Use 0 (default) to disable this feature.

The power limiting feature uses a low-pass filter to smooth out the power readings and prevent rapid fluctuations in throttle. In some cases this can lead to a delay in power limiter activating or deactivating. The :ref:`MOT_BAT_WATT_TC <MOT_BAT_WATT_TC>` parameter can be adjusted to change the time constant of this filter and improve the responsiveness of the power limiter. The default value is 2.

The following table shows the effect of different :ref:`MOT_BAT_WATT_TC <MOT_BAT_WATT_TC>` values on the responsiveness of the power limiter. The power ratio is the ratio of actual power draw to the :ref:`BATT_WATT_MAX<BATT_WATT_MAX>` limit. The percentage values show how much the max throttle is reduced per second at different power ratios for different time constants.

+--------------+------------------------+------------------------+-----------------------+
| Power Ratio  | ``MOT_BAT_WATT_TC=1``  | ``MOT_BAT_WATT_TC=2``  | ``MOT_BAT_WATT_TC=5`` |
+==============+========================+========================+=======================+
| 1.1          | reduced 9%/sec         | reduced 4%/sec         | reduced 1%/sec        |
| 1.5          | reduced 49%/sec        | reduced 24%/sec        | reduced 9%/sec        |
| 2            | reduced 98%/sec        | reduced 49%/sec        | reduced 19%/sec       |
| 5            | reduced 392%/sec       | reduced 198%/sec       | reduced 79%/sec       |
| 10           | reduced 882%/sec       | reduced 445%/sec       | reduced 179%/sec      |
+--------------+------------------------+------------------------+-----------------------+

For example, if the power draw is 50% above the :ref:`BATT_WATT_MAX<BATT_WATT_MAX>` limit (power ratio of 1.5) and :ref:`MOT_BAT_WATT_TC <MOT_BAT_WATT_TC>` is set to 2, then the max throttle will be reduced by 24% per second. If the power draw is 10 times above the limit (power ratio of 10) and :ref:`MOT_BAT_WATT_TC <MOT_BAT_WATT_TC>` is set to 5, then the max throttle will be reduced by 179% per second.
