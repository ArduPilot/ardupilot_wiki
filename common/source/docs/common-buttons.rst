.. _common_buttons:

=======
Buttons
=======

Up to four button inputs to the autopilot may be defined that will execute an :ref:`Auxiliary Function <common-auxiliary-functions>` as if an RC channel switch had been used to execute it. The button input can be either a switch to ground, or a PWM input signal.

:ref:`BTN_ENABLE<BTN_ENABLE>` must be set to 1 to enable this functionality and see the ``BTN`` parameters.

Setup
=====

A GPIO pin (see :ref:`common-gpios`) can be designated as a button input by setting one of the four ``BTN_PINx`` parameters to that GPIO pins' number, eg. :ref:`BTN_PIN1<BTN_PIN1>` = 51. Up to four pins can be assigned to buttons.

These pins will have an internal pullup assigned to them automatically, requiring only a switch to ground to change state from high to low. Open or high on the pin will correspond to the ``RCx_OPTION`` function it controls as being HIGH. Ground or logic low on the pin corresponds to LOW. There is no middle option.

Alternatively, a PWM signal can be used on the pin for switching, just like the PWM value for an RC channel option switch, by setting bit 0 of the button's ``BTNx_OPTIONSx<BTN_OPTIONSx>`` parameter. PWM values >1800us represent HIGH, and <1200us represent LOW.

There is no MIDDLE position for the function, if it has one, using a button.

The effective state of the button can be inverted by setting its ``BTN_OPTIONSx<BTN_OPTIONSx>`` parameter bit 1. This applies to both PWM and switch control of the pin.

The function that will be controlled by the button is determined by its ``BTN_FUNCx`` parameter. If set to -1, no function is performed. Otherwise, the number corresponds to the :ref:`Auxiliary Function <common-auxiliary-functions>` number.

The :ref:`BTN_REPORT_SEND<BTN_REPORT_SEND>` sets the length of time that a button state message is sent to the ground control station for display purposes, each time the button changes state. This is a MAVLink2 message.