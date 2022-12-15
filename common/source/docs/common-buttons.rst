.. _common-buttons:

=======
Buttons
=======

Up to four external buttons or switches connected to the autopilot can be configured to trigger :ref:`Auxiliary Functions <common-auxiliary-functions>` similar to how RC channel switches can trigger them. These button inputs can be configured to use either digital logic level voltage inputs (pull-to-ground or pull-to-high) or PWM signal inputs.

.. note::

    Some :ref:`auxiliary function <common-auxiliary-functions>` have a middle position (e.g "Parachute 3 Pos Switch") but the middle position cannot be triggered from a button

Basic Setup
-----------

In order to use a button, an autopilot pin must be configured as a GPIO. Many autopilots have dedicated pins for this, but some do not. In those cases, a normal SERVO/MOTOR output must be configured as a GPIO pin instead of an :ref:`common-rcoutput-mapping`.

For autopilots with an IOMCU (e.g. Pixhawk, CubeOrange) the external button or switch should be connected to the autopilot's AUX outputs (not the MAIN outputs).  To ease configuration and avoid timer conflicts it may be best to use AUX outputs at the high end (e.g. AUX5 or AUX6) far from outputs used for motors and servos.  For autopilots without an IOMCU (e.g. pixracer, etc) the normal SERVO/MOTOR outputs may be used.

- Set :ref:`BTN_ENABLE<BTN_ENABLE>` = 1 (Enable) and refresh parameters
- If using 4.2 (or later) set the ``SERVOx_FUNCTION`` = -1 (GPIO) where "x" is the servo output channel connected to the button . For example, if the AUX6 output on an autopilot with an IOMCU is to be used as a GPIO, set :ref:`SERVO14_FUNCTION<SERVO14_FUNCTION>` = -1.
- If using 4.1 (or earlier) on an autopilot with an IOMCU (e.g. Pixhawk, CubeOrange) set ``BRD_PWM_COUNT`` low enough so that the servo desired and all other higher outputs may be used for a GPIO input.  E.g. ``BRD_PWM_COUNT`` = 4 means only AUX outputs 5 and 6 can be used as GPIOs, BRD_PWM_COUNT = 0 means AUX outputs 1 to 6 can be used. For boards not using an IOMCU, ``BRD_PWM_COUNT`` less than the total output number, will configure those higher outputs as GPIOs.

Digital Logic/Analog Voltage Setup
----------------------------------

Digital logic level analog voltages can be used for the button input.  The parameter changes listed below are for the 1st button but the 2nd, 3rd and 4th buttons can be setup similarly, simply replace the "1" in the parameter names with "2", "3" or "4". In order to determine which GPIO pin number corresponds to which autopilot SERVO/MOTOR output pin or dedicated GPIO pin, the autopilot's hwdef.dat file must be consulted. See :ref:`common-gpios` for how to determine this.

- Set :ref:`BTN_PIN1<BTN_PIN1>` to the number of the GPIO connected to the button/switch (e.g. "55" = "AUXOUT6")
- Set :ref:`BTN_OPTIONS1<BTN_OPTIONS1>` = set to zero (no bits set in bitmask) to trigger on high logic level voltage (>2.4V) and  be interpreted as a "HIGH" position by the Auxiliary function. Setting bit 1 (PWM input) detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Setting bit 2 (Invert) changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
- Set :ref:`BTN_FUNC1<BTN_FUNC1>` to the :ref:`Auxiliary Function <common-auxiliary-functions>` to be triggered.  If set to 0, no function is to be triggered.

The pin is automatically configured with an internal pullup meaning the button or switch is only required to pull the voltage low.

.. warning::

    Some autopilots pull pins high or low during startup and this may lead to the auxiliary function triggering soon after startup

PWM Input Setup
---------------

PWM signals can be used as button inputs.  PWM values >1800us or <1200us can be used to trigger the Auxiliary Function. The parameter changes listed below are for the 1st button, but the 2nd, 3rd and 4th buttons can be setup similarly. As above, to determine which GPIO pin number corresponds to which autopilot SERVO/MOTOR output pin or dedicated GPIO pin, the autopilot's hwdef.dat file must be consulted. See :ref:`common-gpios` for how to determine this.

- Set :ref:`BTN_PIN1<BTN_PIN1>` to the number of the GPIO connected to the button/switch (e.g. "55" = "AUXOUT6")
- Set :ref:`BTN_OPTIONS1<BTN_OPTIONS1>` = to "1" to be interpreted as a "HIGH" position by the auxiliary function with a PWM > 1800us
- Set :ref:`BTN_OPTIONS1<BTN_OPTIONS1>` = to "3" to be interpreted as a "HIGH" position by the auxiliary function with a PWM < 1200us, or missing a signal input. PWM >1800us will be interpreted as the "LOW" position.
- Set :ref:`BTN_FUNC1<BTN_FUNC1>` to the :ref:`Auxiliary Function <common-auxiliary-functions>` to be triggered.  If set to 0, no function is to be triggered.

.. note::

    The valid PWM input range is 800us to 2200us.  If the PWM value falls outside this range, or is missing, it is equivalent applying <1200us.  Normally this means the auxiliary function will not be triggered unless the invert option of ``BTN_OPTIONSx`` is set.

Additional Setup
----------------

:ref:`BTN_REPORT_SEND<BTN_REPORT_SEND>` sets the length of time that a `BUTTON_CHANGE <https://mavlink.io/en/messages/common.html#BUTTON_CHANGE>`__ mavlink2 message is sent to the ground control station each time the button state changes.  At the time of this writing we do not know of any GCS that makes use of this message.
