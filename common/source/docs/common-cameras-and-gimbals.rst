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

Gimbals
=======

ArduPilot supports both brushless direct drive gimbals (Tarot, SimpleBGC, SToRM32)
that have their own self-stabilization controllers and the simpler servo-driven
gimbals in which ArduPilot controls the stabilisation.

-  :ref:`Gremsy Pixy U <common-gremsy-pixyu-gimbal>` - a high quality 3-axis gimbal
-  :ref:`Servo Gimbals <common-camera-gimbal>` — older-style servo-driven gimbal where ArduPilot provides stabilisation
-  :ref:`SimpleBGC (aka AlexMos) Gimbal Controller <common-simplebgc-gimbal>` - a popular 2-axis or 3-axis brushess gimbal controller which uses a custom serial interface
-  :ref:`SToRM32 Gimbal Controller <common-storm32-gimbal>` — an inexpensive 2-axis or 3-axis brushless gimbal controller which responds to MAVLink commands (a richer format than PWM) over a serial interface
-  :ref:`Tarot 2D Gimbal <common-tarot-gimbal>` — low cost 2-axis brushless gimbal

Cameras with MAVLink interfaces
===============================

-  :ref:`FLIR Vue Pro Thermal Camera <common-flir-vue-pro>`

.. _common-cameras-and-gimbals_camera_shutter_triggering:

Camera Control and GeoTagging
=============================

ArduPilot allows you to :ref:`configure the camera shutter output port <common-camera-shutter-with-servo>` (servo, relay). In :ref:`camera mission planning <common-camera-control-and-auto-missions-in-mission-planner>`
you can specify when the camera shutter should trigger, or a distance
that the vehicle should travel between shots.

Camera manufacturers use their own mechanisms for remote control of the
camera (including its shutter). The topics explain how to configure the
camera shutter, and list a number of different approaches for converting
the output signal into the form expected by your particular camera:

-  :ref:`Airpixel Entire Geotagger <common-geotagging-airpixel-entire>`
-  :ref:`DROTAG x Geotagger  <common-geotagging-drotagx>`
-  :ref:`Seagull IR Camera Trigger <common-camera-trigger-seagull-ir>`
-  :ref:`Seagull MAP2 Camera Trigger <common-camera-trigger-seagull-map2>`
-  :ref:`Seagull MAP-X2 Camera Trigger and Logger <common-camera-trigger-seagull-mapx2>`
-  :ref:`Seagull REC Camera Trigger <common-camera-trigger-seagull-rec>`
-  :ref:`Skysight Mono Camera Trigger <common-camera-trigger-skysight-mono>`
-  :ref:`StratosnapperV2 Camera Trigger <common-camera-trigger-stratosnapperv2>`
-  :ref:`Camera Triggering Directly from AUX Ports <common-pixhawk-camera-trigger-setup>`
-  :ref:`Camera Triggering Configuration <common-camera-shutter-with-servo>`
-  :ref:`Camera Triggering using CHDK <common-apm-to-chdk-camera-link-tutorial>`
-  :ref:`Camera Triggering using CHDK Tutorial <common-chdk-camera-control-tutorial>` (non-standard integration)
-  :ref:`RunCam Camera Control <common-camera-runcam>`

Detail topics
=============

.. toctree::
    :maxdepth: 1

    Gremsy Pixy U Gimbal <common-gremsy-pixyu-gimbal>
    Servo Gimbal <common-camera-gimbal>
    SimpleBGC Gimbal Controller <common-simplebgc-gimbal>
    SToRM32 Gimbal Controller <common-storm32-gimbal>
    Tarot 2D Gimbal <common-tarot-gimbal>
    FLIR Vue Pro Thermal Camera <common-flir-vue-pro>
    Airpixel Entire Geotagger <common-geotagging-airpixel-entire>
    DROTAG x Geotagger <common-geotagging-drotagx>
    Seagull IR Camera Trigger <common-camera-trigger-seagull-ir>
    Seagull MAP2 Camera Trigger <common-camera-trigger-seagull-map2>
    Seagull MAP-X2 Camera Trigger and Logger <common-camera-trigger-seagull-mapx2>
    Seagull REC Camera Trigger <common-camera-trigger-seagull-rec>
    Skysight Mono Camera Trigger <common-camera-trigger-skysight-mono>
    StratosnapperV2 Camera Trigger <common-camera-trigger-stratosnapperv2>
    Camera Trigger Directly from AUX Ports <common-pixhawk-camera-trigger-setup>
    Camera Triggering Configuration <common-camera-shutter-with-servo>
    Camera Triggering using CHDK <common-apm-to-chdk-camera-link-tutorial>
    Camera Triggering using CHDK Tutorial <common-chdk-camera-control-tutorial>
    RunCam Camera Control <common-camera-runcam>
