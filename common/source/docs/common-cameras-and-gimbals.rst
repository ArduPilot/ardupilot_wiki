.. _common-cameras-and-gimbals:

===================
Cameras and Gimbals
===================

Copter, Plane and Rover support up to 3-axis gimbals, including advance
features like automated aiming of the camera at a Region of Interest
(ROI), and automatic triggering of a camera shutter.  Follow the links below
to explanations of how to configure gimbals and shutter triggering.

.. image:: ../../../images/Cameras_Gimbals.jpg
    :target: ../_images/Cameras_Gimbals.jpg

Gimbals and gimbal controllers
==============================

ArduPilot supports both brushless direct drive gimbals (Tarot, SimpleBGC, SToRM32)
that have their own self-stabilization controllers and the simpler servo-driven
gimbals in which ArduPilot controls the stabilisation.

-  :ref:`Tarot Gimbal <common-tarot-gimbal>` — an inexpensive 2-axis brushless gimbal controller that is controlled using PWM signals.
-  :ref:`SimpleBGC (aka AlexMos) Gimbal <common-simplebgc-gimbal>` - a popular 2-axis or 3-axis brushess gimbal controller which uses a custom serial interface.
-  :ref:`SToRM32 Gimbal Controller <common-storm32-gimbal>` — an inexpensive 2-axis or 3-axis brushless gimbal controller which responds to MAVLink commands (a richer format than PWM) over a serial interface.
-  :ref:`Servo Gimbals <common-camera-gimbal>` — older-style servo-driven gimbal where ArduPilot provides stabilisation.

.. _common-cameras-and-gimbals_camera_shutter_triggering:

Camera shutter triggering
=========================

ArduPilot allows you to :ref:`configure the camera shutter output port <common-camera-shutter-with-servo>` (servo, relay). In :ref:`camera mission planning <common-camera-control-and-auto-missions-in-mission-planner>`
you can specify when the camera shutter should trigger, or a distance
that the vehicle should travel between shots.

Camera manufacturers use their own mechanisms for remote control of the
camera (including its shutter). The topics explain how to configure the
camera shutter, and list a number of different approaches for converting
the output signal into the form expected by your particular camera:

-  :ref:`Camera Shutter Configuration in Mission Planner <common-camera-shutter-with-servo>`
-  :ref:`Camera Shutter Triggering using CHDK <common-apm-to-chdk-camera-link-tutorial>`
-  :ref:`CHDK Camera Control Tutorial <common-chdk-camera-control-tutorial>` (non-standard
   integration)
-  :ref:`3DR Camera Control Board <common-camera-control-board>` (prototype
   - many cameras)

Detail topics
=============

.. toctree::
    :maxdepth: 1

    Tarot Brushless Gimbal <common-tarot-gimbal>
    SToRM32 Gimbal Controller <common-storm32-gimbal>
    SimpleBGC Gimbal <common-simplebgc-gimbal>
    Gimbal with Servos <common-camera-gimbal>
    Camera Shutter Configuration <common-camera-shutter-with-servo>
    Camera Triggering using CHDK <common-apm-to-chdk-camera-link-tutorial>
    Camera Shutter Trigger Boards for Purchase <common-camera-shutter-triggering-cables-for-purchase>
    CHDK Camera Control <common-chdk-camera-control-tutorial>

