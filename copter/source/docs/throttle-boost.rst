.. _throttle_boost:

==============
Throttle Boost
==============

Even well tuned Multicopters can momentarily lose tight attitude control with sudden, large throttle changes. Dips and Rolls can occur on "punch-outs" or "throttle chops", especially on light vehicles with high thrust to weight ratios.

In order to reduce this effect, the :ref:`ATC_THR_G_BOOST<ATC_THR_G_BOOST>` parameter can be adjusted. Normally inactive at its default value of "0", setting this to a non-zero number will provide PID gain multiplication anytime the throttle is changed rapidly, either up or down. At the :ref:`ATC_THR_G_BOOST<ATC_THR_G_BOOST>` parameter's maximum value of one (1), the  rate PIDs will be increased by a factor of 2 and the Angle PIDs will be increased by a value of 4 during the time the throttle is being rapidly changed. Lower values will result in lower gain multiplication.

The less tightly tuned the vehicle is, the more it will require Throttle Boost.