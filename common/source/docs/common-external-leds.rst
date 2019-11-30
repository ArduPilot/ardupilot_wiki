.. _common-external-leds:

=============
External LEDs
=============

An external LED can be added by connecting it to the autopilot's
I2C port or, in 4.0 and later versions, via a GPIO capable pin using the SERVOx_FUNCTION for that output.

.. note:: On Pixhawk style architecture autopilots which use an IOMCU for the PWM1-8 outputs, only the "AUX" outputs are GPIO capable. 

Connection
==========

Usually, using the I2C port requires an LED driver chip connected to the LEDs. Because most users use a :ref:`Ublox GPS and Compass module <common-positioning-landing-page>`, an I2C splitter is recommended, such as shown below. The :ref:`NTF_LED_TYPES<NTF_LED_TYPES>` parameter should be set to the controller type used.

.. image:: ../../../images/ExternalLED_PixhawkLED.jpg
    :target: ../_images/ExternalLED_PixhawkLED.jpg

A "NeoPixel" style (WS2812B compatible) RGB LED can be attached to any GPIO capable output by setting its SERVOx_FUNCTION to one of the ``NeoPixelx`` output functions and setting :ref:`NTF_LED_TYPES<NTF_LED_TYPES>` parameter to ``NeoPixel``. Multiple ``NeoPixelx`` output functions are provided for future expansion, but currently they all behave exactly the same.

LED Meaning
===========

The meaning of the colors and flash patterns are shown in :ref:`common-leds-pixhawk` if the ``standard`` default protocol is set for :ref:`NTF_LED_OVERRIDE<NTF_LED_OVERRIDE>`. 

The brightness of the LED can be controlled by modifying the :ref:`NTF_LED_BRIGHT <NTF_LED_BRIGHT>`

