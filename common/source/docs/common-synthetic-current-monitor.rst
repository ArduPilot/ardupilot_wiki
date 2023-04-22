.. _common-synthetic-current-monitor:
[copywiki destination="plane,copter,rover"]

===============================================
Synthetic Current Sensor/Analog Voltage Monitor
===============================================

Most autopilots have provisions for an external current sensor (normally as part of an external power module or ESC) or an integrated current sensor. For those using an external sensor, either using a power module or esc with current sensor or telemetry is not desireable due to weight or other considerations. In those cases, it may still be desired to obtain a measure of the current for either consumption reporting or other uses. This battery monitor backend provides a current estimate based on throttle position as well as the normal analog voltage reporting.

.. note:: currently this estimate is accurate only for Plane, Blimp, and Rover primary motors. Multicopter and QuadPlane VTOL mode current estimates are inaccurate and not recommended for use. Future improvements to provide accurate Multicopter and QuadPlane VTOL modes estimates are planned.

Setup
=====
the following examples are for the first Battery Monitor. The parameter names will be ``BATTx`` prefaced for other monitors usage.

- :ref:`BATT_MONITOR <BATT_MONITOR>` = 25 

For Voltage, the normal Analog Sensor parameters apply:

- :ref:`BATT_VOLT_PIN <BATT_VOLT_PIN>` The autopilot pin connected to the power module's voltage pin
- :ref:`BATT_VOLT_MULT <BATT_VOLT_MULT>` converts the analog voltage received from the power module's voltage pin to the battery's voltage. Provided on the :ref:`autopilot's hardware page:<common-autopilots>` or manufacturers page.

For the Current, some of the normal analog current sensor parameters have been re-tasked to save flash and their names are therefore a bit misleading in this application:

- :ref:`BATT_AMP_PERVLT <BATT_AMP_PERVLT>` is the MAXIMUM current at full battery voltage in amps.
- :ref:`BATT_AMP_OFFSET <BATT_AMP_OFFSET>` is the idle current (zero throttle) of the system in amps accounting for peripherals, video,etc. Normally in the 0.3 to 0.5 range.
- :ref:`BATT_MAX_VOLT<BATT_MAX_VOLT>` is the MAXIMUM fresh battery voltage. Used to scale the estimate as the battery voltage decreases during use.

Calibration of the :ref:`BATT_AMP_PERVLT <BATT_AMP_PERVLT>` value can either be done on the bench with a current meter, or iteratively by estimating an initial value, operating for while, measuring the amount of current needed to restore the battery to full charge, and using that value and a flight log's or OSD's total consumed mah for the flight to adjust the parameter value:

new value = old value * (mah returned to batt/log or OSD value of consumed mah)

one or two iterations of the above may be required.