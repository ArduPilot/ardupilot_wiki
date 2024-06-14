.. _common-camera-shutter-with-servo:

============================
Camera Shutter Configuration
============================

ArduPilot allows you to configure a :ref:`servo output<common-servo>` or :ref:`relay<common-relay>` to control a camera trigger from :ref:`an RC transmitter, a ground station or during a mission <common-camera-controls>`.

This article explains in general what connections and settings are required but in most cases the autopilot's servo output or relay voltage will be converted into a format understood by the particular camera used and the setup details will depend upon what hardware is used (infrared, cable, etc).  These two pages may provide more details

- :ref:`DIY article for a Pixhawk using a relay based trigger<common-pixhawk-camera-trigger-setup>`

Servo control connection and configuration
==========================================

Connect one of the autopilot's PWM output (aka servo outputs) to the camera or the camera control cable that accepts PWM input.
As mentioned on the :ref:`servos wiki page <common-servo>` there are some autopilot specific limitations on which PWM outputs may be used:

- For autopilots with IOMCU (e.g. label outputs as MAIN OUT and AUX OUT), it is easiest to use AUX OUT 1 to 6.
- For autopilots without an IOMCU any output may be used

Configure the autopilot by setting these parameters:

- Set `SERVOx_FUNCTION` = 10 (CameraTrigger) where "x" is the PWM output connected to the camera
- Set :ref:`CAM1_TYPE<CAM1_TYPE>` = 1 (Servo) and reboot the autopilot
- Set :ref:`CAM1_SERVO_ON<CAM1_SERVO_ON>` to the pwm value to output to trigger taking a picture (e.g. 1300)
- Set :ref:`CAM1_SERVO_OFF<CAM1_SERVO_OFF>` to the pwm value to output when *not* taking a picture (e.g. 1100)
- Set :ref:`CAM1_DURATION<CAM1_DURATION>` to the time (in seconds) that the pwm value should remain high (e.g 0.1)

Relay control connection and configuration
==========================================

Connect one of the autopilot's GPIO pins to the camera or the camera control cable that accepts high/low voltage input.  As mentioned on the :ref:`relay wiki page<common-relay>` the autopilot's servo/motor outputs can normally be used but, in addition, some autopilots have dedicated pins purely for use as relays/GPIOs.

Configure the autopilot by setting these parameters:

- If using 4.5 (or earlier) only the 1st relay can be used so set ``RELAY_PIN`` to the GPIO pin used
- If using 4.6 (or higher) any relay can be used.  Below are settings if the first relay is used.

  - Set :ref:`RELAY1_FUNCTION<RELAY1_FUNCTION>` = 4 (Camera) 
  - Set :ref:`RELAY1_PIN<RELAY1_PIN>` to the GPIO pin used

- Set ``SERVOx_FUNCTION`` = -1 (GPIO) where "x" is the servo output channel used.  This is not required in the rare case where an autopilot's dedicated GPIO pins are used.
- Set :ref:`CAM1_DURATION<CAM1_DURATION>` to the time (in seconds) that the relay is held high
- Set :ref:`CAM1_RELAY_ON<CAM1_RELAY_ON>` = 0 to swap the relay voltage.  E.g. GPIO output would now be normally high by default, and then low to trigger the camera shutter

.. _common-camera-shutter-with-servo_enhanced_camera_trigger_logging:

Camera shutter feedback logging
===============================

ArduPilot logs TRIG messages when it *triggers* the camera.  If the camera provides a GPIO output (e.g. camera flash hotshoe) then this can be used to also log CAM messages at the exact moment that pictures are taken.

Connect the camera's GPIO output  to one of the autopilot's :ref:`GPIO pins <common-gpios>` (e.g. AUX OUT).  As mentioned above there are restrictions on which pins may be used.

Set the following parameters:
 
- Set `SERVOx_FUNCTION` = -1 (GPIO) where "x" is the servo output channel used.  This is not required in the rare case where the autopilot's dedicated GPIO pins are useds
- Set :ref:`CAM1_FEEDBAK_PIN<CAM1_FEEDBAK_PIN>` to the pin number connected to the hotshoe
- Set :ref:`CAM1_FEEDBAK_POL<CAM1_FEEDBAK_POL>` = 0 if the hotshoe voltage goes low when a picture is taken or 1 if the voltage goes high

See :ref:`digital input pin <common-pixhawk-overview_pixhawk_digital_outputs_and_inputs_virtual_pins_50-55>` for more details.

Consider using the `Seagulls SYNC2 Shoe Horn Adapter <https://www.seagulluav.com/product/seagull-sync2/>`__
