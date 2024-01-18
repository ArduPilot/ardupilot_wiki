.. _common-camera-shutter-with-servo:

============================
Camera Shutter Configuration
============================

ArduPilot allows you to configure a particular port (servo or relay) as
the camera trigger, which will then be activated when :ref:`camera commands are specified in missions <common-camera-control-and-auto-missions-in-mission-planner>`.

This article explains what settings you need to configure for both servos and relays. For a typical relay based trigger, see this DIY article using a Pixhawk as an example autopilot: :ref:`common-pixhawk-camera-trigger-setup`.

.. note::

   The servo or relay port signal must be converted into a format
   (infrared, cable or whatever) understood by your particular camera. The
   configuration settings will depend on the hardware that is used to
   perform this conversion. Some useful hardware configurations and
   settings are linked from the section 
   :ref:`Setting values for different cameras <common-camera-shutter-with-servo_setting_values_for_different_cameras>`. 

Shutter configuration with Pixhawk or IOMCU Equipped Autopilots
===============================================================

.. note:: on autopilots not using an IOMCU (most that do, label outputs as MAIN/AUX), ANY output can be used for a relay or servo. See :ref:`common-gpios` for how to designate an output as a GPIO for relay use.

Pixhawk has 6 AUX Ports (AUX1-AUX6, referred to as SERVO9-SERVO14 in *Mission
Planner*) that can be configured as :ref:`servos <common-servo>`,
:ref:`relays <common-relay>`, or 
:ref:`digital inputs or outputs <common-pixhawk-overview_pixhawk_digital_outputs_and_inputs_virtual_pins_50-55>`.
The image and configuration below is for the Pixhawk with SERVO10 (labeled RC10/AUX2 on this autopilot) connected to camera control hardware and configured as either a servo or relay.

.. figure:: ../../../images/Pixhawkdetailview.jpg
   :target: ../_images/Pixhawkdetailview.jpg

   Pixhawk Detail View highlighting AUXPorts


.. tip::

   You can monitor and log *exactly* when the camera was triggered. For more information see the :ref:`Enhanced camera trigger logging <common-camera-shutter-with-servo_enhanced_camera_trigger_logging>` section below.

The actual output used for the shutter is set and configured in the SETUP/Optional Hardware/Camera Gimbal screen:

   .. figure:: ../../../images/missionplannercameragimbalscreen.jpg
      :target: ../_images/missionplannercameragimbalscreen.jpg

      Mission Planner: Camera Gimbal Configuration Screen

-  The **Shutter** drop-down list is used to set the connected output for camera
   trigger. Here we have selected SERVO10, which corresponds to AUX2
   on the Pixhawk.
-  The **Duration** setting specifies how long the servo/relay
   will be held in the **Pushed** state when the shutter is activated, in
   tenths of a second. Above the value is 10, so the pushed state is
   held for one second. **Not Pushed** when the shutter is not active.
-  **For Servos only (settings ignored for relay outputs):**

   -  The Shutter *Pushed* and *Not Pushed* settings are the PWM signal
      values that will be sent when the servo is in those states.
   -  The *Servo Limits* setting specifies the range of PWM signal
      values within which the servo will not bind.
      
.. note:: Mission Planners screen is not up-to-date in that the request to "Please set the Ch7 Option to Camera Trigger" is out of date, and the "transistor" selection in the *Shutter* drop-down list does nothing.

- To set which RC Channel will control the manual shutter release, configure its ``RCx_OPTION`` in the CONFIG/Full Parameter List or CONFIG/User Params to "Camera Trigger".

.. _common-camera-shutter-with-servo_enhanced_camera_trigger_logging:

Enhanced camera trigger logging
===============================

ArduPilot logs TRIG messages when it *triggers* the camera.  You can additionally set up ArduPilot to log CAM messages when the camera has actually fired, by connecting a :ref:`digital input pin <common-pixhawk-overview_pixhawk_digital_outputs_and_inputs_virtual_pins_50-55>` on the autopilot to the camera's hot shoe (consider using `Seagulls SYNC2 Shoe Horn Adapter <https://www.seagulluav.com/product/seagull-sync2/>`__).  This more accurately logs the exact time that pictures are recorded.

You will need to configure one of the AUX pins as a digital GPIO
output/input, and connect it to the camera flash hotshoe (a universal
camera hot shoe is required). The pin should be held for at least 2
milliseconds for reliable trigger detection.

The main steps are (example for Camera1 instance):

#. Open *Mission Planner* and then click on **CONFIG/TUNING/Full
   Parameters List**
#. Set at least two of the output pins as digital GPIO output/inputs as described in 
   :ref:`GPIOs <common-gpios>`.
#. Set :ref:`CAM1_FEEDBAK_PIN<CAM1_FEEDBAK_PIN>` to the pin number connected to the hotshoe.
#. Set :ref:`CAM1_FEEDBAK_POL<CAM1_FEEDBAK_POL>` to indicate whether the feedback pin (hotshoe voltage) goes high or low when the picture is taken.

.. _common-camera-shutter-with-servo_setting_values_for_different_cameras:

Setting values for different cameras
====================================

The actual values needed for servo/relay settings depends on what
hardware is used to send the shutter signal to the camera. The following
topics describe the hardware setup and configuration settings for a
number of specific cameras/camera types:

-  :ref:`Camera Triggering using Stratosnapper <common-camera-trigger-stratosnapperv2>` -
   shows how to connect to a camera with an IR interface. The
   Stratosnapper can also be used to connect to cameras using other
   cables and protocols
-  :ref:`Camera Shutter with Relay and CHDK on APM <common-apm-to-chdk-camera-link-tutorial>` - shows how to set
   up a relay port to send a signal to a Canon camera running CHDK (on
   APM2.x)

If these aren't suitable for your hardware configuration, we recommend
you check your hardware manual for information about servo/relay inputs
that are accepted.

.. note::

   The :ref:`CHDK Camera Control Tutorial <common-chdk-camera-control-tutorial>` is not a good
   example of integrating with the camera shutter, because it does not use
   the standard shutter configuration explained in this article. This is
   however a good example of how you can access other features of a Canon
   camera using CHDK (for example, the zoom).
