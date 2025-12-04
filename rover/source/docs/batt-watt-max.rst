.. _batt-watt-max:

===================================
Limiting Maximum Battery Power Draw
===================================

Sometimes power needs to be limited to prevent issues with battery sag or overheating. This can occur because of the use of lower C rated batteries, high power motors, high ambient temperatures, uncooled engines or ESCs, or other reasons. This topic discusses ways to limit the maximum power draw from batteries.

Methods to Limit Maximum Power Draw
===================================

The first and simplest is to restrict :ref:`MOT_THR_MAX<MOT_THR_MAX>` to a lower value. This limits the maximum throttle controlled allowed. Unfortunately, this will not set an exact power limit and might require some trial and error to find the right setting.

A better way is to use a power limiting parameters:

- ``WATT_MAX``

The ``WATT_MAX`` parameter can be used to limit the total wattage that can be drawn from the battery(s). If battery power exceeds this value then the system will reduce max throttle (:ref:`MOT_THR_MAX<MOT_THR_MAX>`) to lower the power draw. If power demand is reduced, the max throttle will slowly grow back to :ref:`MOT_THR_MAX<MOT_THR_MAX>`. Use 0 (default) to disable this feature.

The ``BAT_WATT_TC`` parameter can be used to adjust the rate at which the max throttle is reduced when the battery power exceeds the ``WATT_MAX`` limit. This can help to fine-tune the response to high power situations and prevent sudden drops in throttle. Smaller values will result in a faster response, while larger values will provide a more gradual adjustment. The default value is 5.