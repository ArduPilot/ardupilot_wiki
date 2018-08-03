.. _common-external-leds:

=============
External LEDs
=============

An external LED (aka "Toshiba LED") can be added by connecting it to the flight controller's
I2C port. Because most users use a :ref:`Ublox GPS and Compass module <common-positioning-landing-page>`,
an I2C splitter is recommended.  If using a Pixhawk flight controller, once connected the LED on the center of the board may no longer light up (recent versions of ArduPilot will light up both LEDs).

.. image:: ../../../images/ExternalLED_PixhawkLED.jpg
    :target: ../_images/ExternalLED_PixhawkLED.jpg

The brightness of the LED can be controlled by modifying the :ref:`NTF_LED_BRIGHT <NTF_LED_BRIGHT>`

[site wiki="copter" heading="off"]

[/site]
