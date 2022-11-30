.. _manual-mode:

===========
Manual mode
===========

In Manual mode, the transmitter's steering and throttle sticks directly control the vehicle's throttle and steering output.

- This mode does not require a position estimate (i.e. GPS is not required).
- It is recommended to always have one :ref:`transmitter mode switch <common-rc-transmitter-flight-mode-configuration>` position assigned to this mode.
- The throttle slew still applies. See the bottom of the :ref:`Tuning Speed and Throttle <rover-tuning-throttle-and-speed>` page for more details about the throttle slew limit.
- Steering angle speed scaling can be enabled via the first bit of :ref:`MANUAL_OPTIONS<MANUAL_OPTIONS>`. This requires a ground speed estimate from one of many types of sensors.
- Steering expo can be applied using :ref:`MANUAL_STR_EXPO<MANUAL_STR_EXPO>` to decrease transmitter steering control sensitivity around neutral.
