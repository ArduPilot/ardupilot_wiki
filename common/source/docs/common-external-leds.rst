.. _common-external-leds:

=============
External LEDs
=============

An external LED or LED Display can be added by connecting it to the autopilot's
I2C port or, in 4.0 and later versions, if a serially programmed device, via an output using the SERVOx_FUNCTION for that output. UAVCAN LEDs on the CANBUS are also supported (See :ref:`UAVCAN Setup <common-uavcan-setup-advanced>` ).

.. note:: Note that the same grouping restrictions which apply to setting outputs to different PWM rates and/or DShot, apply to LEDs also since the timer associated with each group of outputs cannot be used for different rates. See also  :ref:`common-dshot` 

RGB LEDs/Displays with I2C Connection
=====================================

Two types of I2C devices are supported: IC2 connected RGB LEDS using an LED driver chip and OLED displays.

RGB LEDs/drivers supported are:

- Toshiba LEDs (no longer manufactured)
- PCA9685 driver
- NCP5623 driver

OLED Displays supported are SSH1106 and SSD1306 with 128x64 pixel displays. See :ref:`common-display-onboard` for more information.

.. image:: ../../../images/ssh1106.jpg
    :width: 450px


Because most users use an external :ref:`GPS and Compass module <common-positioning-landing-page>`, an I2C splitter is recommended, such as shown below. The :ref:`NTF_LED_TYPES<NTF_LED_TYPES>` parameter should be set to the controller type used.

.. image:: ../../../images/ExternalLED_PixhawkLED.jpg
    :target: ../_images/ExternalLED_PixhawkLED.jpg

Serially Connected Devices
==========================

Currently, Ardupilot supports the Neopixel style RGB LED and strings. They can be used for NTF notifications from the autopilot on status and warnings like other RGB LEDs, or be programmed in unlimited ways using LUA scripts on the autopilot's SD card. See :ref:`common-lua-scripts` for more use examples using LUA scripts.


.. image:: ../../../images/neopixel-string.jpg

A "NeoPixel" style (WS2812B compatible) RGB LED can be attached to any PWM output by setting its SERVOx_FUNCTION to one of the ``NeoPixelx`` output functions and setting :ref:`NTF_LED_TYPES<NTF_LED_TYPES>` parameter to ``NeoPixel``. Multiple ``NeoPixelx`` output functions are provided for connecting multiple strings (up to 4).

.. note:: Be sure the output is configures as normal PWM instead of a GPIO output. See :ref:`common-gpios`

If used for notification purposes, be sure to set :ref:`NTF_LED_TYPES<NTF_LED_TYPES>` to "Neopixel" (8).

.. warning:: Most WS2812 style LED and strings will operate correctly when connected to the autopilot. However, if you get intermittent or non-operation, you may need to implement one of the configurations below. This is due to the fact that the autopilot outputs swing to 3.3V but the worst case input signal high spec for the LED is 4.3V at a 5V supply. So at extremes of tolerance/manufacturing spec, you can get a combination which will not work correctly. In that case the easiest solution is to lower the LED supply as shown below.


.. image:: ../../../images/neopixel-fix.png

But this causes a small loss in LED brilliance. An easy way to avoid this, if an LED can be isolated in the string, is to use an LED to level shift the signal for you so that the rest of the string can be powered by the full 5V.

.. image:: ../../../images/ws-levelshift.png

LED Meaning
===========

The meaning of the colors and flash patterns are shown in :ref:`common-leds-pixhawk` if the ``standard`` default protocol is set for :ref:`NTF_LED_OVERRIDE<NTF_LED_OVERRIDE>`. 

The brightness of the LED can be controlled by modifying the :ref:`NTF_LED_BRIGHT <NTF_LED_BRIGHT>`

